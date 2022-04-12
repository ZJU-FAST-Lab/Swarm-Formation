#include <vrb_planner/virtual_structure.hpp>

void VRB::init( ros::NodeHandle &nh ){

    nh.param( "vrb/form_coef", form_coef_, 1.0 );
    nh.param( "vrb/obs_coef", obs_coef_, 13.0 );
    nh.param( "vrb/swarm_coef", swarm_coef_, 5.0 );
    nh.param( "vrb/sigma_1", sigma_1_, 1.0 );
    nh.param( "vrb/sigma_2", sigma_2_, 1.0 );
    nh.param( "vrb/swarm_clearance", swarm_clearance_, 0.4 );
    nh.param( "vrb/obs_clearance", obs_clearance_, 0.8 );
    nh.param( "vrb/obs_closure", obs_closure_, 1.2 );

    nh.param("manager/drone_id", drone_id_, -1);   
    nh.param("time_interval", t_interval_, 0.001);

}

void VRB::setGridMap(GridMap::Ptr grid_map){
    grid_map_ = grid_map;
}

void VRB::setForm( const std::vector<Eigen::Vector3d> &form_nodes ){
    form_des_ = form_nodes;
    node_num_ = form_des_.size();
}

//For swarm_pos transformation
void VRB::glb2vrb( const Eigen::Vector3d &vrb_origin, const std::vector<Eigen::Vector3d> &global_pos, std::vector<Eigen::Vector3d> &vrb_pos){
    
    vrb_pos.clear();
    for (auto glb_pos: global_pos){
        vrb_pos.push_back( glb_pos - vrb_origin );
    }
}

//For single odom transformation
void VRB::glb2vrb( const Eigen::Vector3d &vrb_origin, const Eigen::Vector3d &global_pos, Eigen::Vector3d &vrb_pos ){
    vrb_pos = global_pos - vrb_origin;
}

//Formation vector field
void VRB::formVF( const Eigen::Vector3d &agent_pos_vrb, Eigen::Vector3d &v_form){
    v_form = form_coef_ * ( form_des_[drone_id_ - 1] - agent_pos_vrb );
    v_form(2) = 0.0;
}


//Swarm avoidance vector field
void VRB::swarmVF( const std::vector<Eigen::Vector3d> &swarm_pos_vrb, Eigen::Vector3d &v_swarm ){
    double dist = 0;
    for( int i = 0; i < node_num_; i++ ){
        dist = ( swarm_pos_vrb[drone_id_-1] - swarm_pos_vrb[i]).norm();
        if( (drone_id_-1)!= i && dist < swarm_clearance_ ){
            v_swarm += swarm_coef_ * exp( - dist * dist / (2 * sigma_2_ * sigma_2_) ) * (swarm_pos_vrb[drone_id_-1] - swarm_pos_vrb[i]) / dist;
        }
    }
    v_swarm(2) = 0.0;
}


void VRB::obsVF( const Eigen::Vector3d &agent_pos_glb, Eigen::Vector3d &v_obs, Eigen::Vector3d &v_form){

    double dist;
    Eigen::Vector3d dist_grad;
    grid_map_->evaluateEDT( agent_pos_glb, dist );

    // std::cout << "agent_pos_glb: " << agent_pos_glb.transpose() << endl;
    // std::cout << " Obs dist: " << dist << std::endl;

    if( dist < obs_clearance_ ){
        grid_map_->evaluateFirstGrad( agent_pos_glb, dist_grad );
        v_obs = obs_coef_ * exp( - dist * dist / ( 2 * sigma_1_ * sigma_1_ ) ) * dist_grad.normalized();

        if( dist < obs_closure_ && dist >= 0.6 * obs_closure_ ){
            // std::cout << "Triggered. " << std::endl;
            v_form = 0.8 * v_form;
            v_obs = 1.2 * v_obs;
        }else if(dist < 0.6 * obs_closure_){
            v_form = 0.4 * v_form;
            v_obs = 1.6 * v_obs;
        }
    }
    v_obs(2) = 0.0;
}

void VRB::adjustVF( const Eigen::Vector3d &v_form, Eigen::Vector3d &v_obs){
    
    double obs_norm = v_obs.norm();
    Eigen::Vector3d obs_unit = v_obs.normalized();

    double steer = acos(v_form.dot(v_obs) / ( v_form.norm() * obs_norm ));
    double obs_angle = atan2( obs_unit(1),obs_unit(0) );
    double pi = 3.1415926;
    
    if( steer > (pi - 0.5) && obs_norm > 0.2 ){
        if(obs_angle <= 0){
            v_obs << obs_norm*(cos(pi/6)*obs_unit(0) - sin(pi/6)*obs_unit(1)) , obs_norm*(sin(pi/6)*obs_unit(0) + cos(pi/6)*obs_unit(1)), 0.0;
        }else{
            v_obs << obs_norm*(cos(-pi/6)*obs_unit(0) - sin(-pi/6)*obs_unit(1)) , obs_norm*(sin(-pi/6)*obs_unit(0) + cos(-pi/6)*obs_unit(1)), 0.0;
        }
    }

    if( obs_angle < (pi/16 - pi)){
        v_obs << obs_norm*(cos(pi/4)*obs_unit(0) - sin(pi/4)*obs_unit(1)) , obs_norm*(sin(pi/4)*obs_unit(0) + cos(pi/4)*obs_unit(1)), 0.0;
    }

    if( obs_angle > (pi - pi/16) ){
        v_obs << obs_norm*(cos(-pi/4)*obs_unit(0) - sin(pi/4)*obs_unit(1)) , obs_norm*(sin(-pi/4)*obs_unit(0) + cos(-pi/4)*obs_unit(1)), 0.0;
    }   
    
}


bool VRB::updateVel( const Eigen::Vector3d &vrb_origin, const Eigen::Vector3d &vrb_vel, const Eigen::Vector3d &vrb_acc, 
                     const std::vector<Eigen::Vector3d> &swarm_pos_glb, const Eigen::Vector3d &odom_pos_glb, 
                     Eigen::Vector3d &v_cmd,  
                     Eigen::Vector3d &formation_vel, Eigen::Vector3d &swarm_vel, Eigen::Vector3d &obs_vel){

    Eigen::Vector3d v_form(0,0,0), v_swarm(0,0,0), v_obs(0,0,0); 
    // std::cout << "drone " << drone_id_ << " vrb_origin: " << vrb_origin.transpose() << std::endl;
    // std::cout << "drone " << drone_id_ << " odom_pos_glb: " << odom_pos_glb.transpose() << std::endl;

    //Get coordinates in VRB frame
    glb2vrb( vrb_origin, swarm_pos_glb, swarm_pos_vrb_ );
    glb2vrb( vrb_origin, odom_pos_glb, odom_pos_vrb_ );
    

    //Generate vector fields
    formVF( odom_pos_vrb_, v_form );
    // std::cout << "start update vel --------------------------- " << std::endl;
    // std::cout << " form term: " << v_form.transpose() << std::endl;
    

    // swarmVF( swarm_pos_vrb_, v_swarm ); 
    // std::cout << " swarm term: " << v_swarm.transpose() << std::endl; 
    // swarm_vel = v_swarm;

    obsVF( odom_pos_glb, v_obs, v_form);
    // std::cout << " obs term: " << v_obs.transpose() << std::endl; 

    // Adjust the direction of v_obs
    adjustVF( v_form, v_obs );

    formation_vel = v_form;
    obs_vel = v_obs;
    
    // v_cmd = v_form + v_swarm  + vrb_vel + vrb_acc * t_interval_;
    // v_cmd = v_form + vrb_vel;
    v_cmd = v_form + v_swarm + v_obs + vrb_vel + vrb_acc * t_interval_;

    static int motherfucker_countman = 0;
    motherfucker_countman++;

    // if(motherfucker_countman == 10){
    //     Eigen::Vector3d obs_unit = v_obs.normalized();

    //     double steer = acos(v_form.dot(v_obs) / ( v_form.norm() * v_obs.norm() ));
    //     double obs_angle = atan2( obs_unit(1),obs_unit(0) );

    //     std::cout << "drone " << drone_id_ << " obs_angle: " << obs_angle << std::endl;
    //     std::cout << "drone " << drone_id_ << " steer: " << steer << std::endl;
    //     motherfucker_countman = 0;
    // }

    // std::cout << "drone " << drone_id_ << " form_value: " << v_form.norm() << " obs_value: "  << v_obs.norm() << std::endl;
    
    //Eliminate the velocity on z-axis
    v_cmd(2) = 0.0; 
    


    return true;
}

// void VRB::formVF( const std::vector<Eigen::Vector3d> &agent_pos_vrb, Eigen::MatrixXd &v_form){
    
//     v_form = Eigen::MatrixXd::Zero(3, node_num_);
//     for( int i = 0; i < node_num_; i++ ){
//         v_form.col(i) = form_coef_ * ( form_des_[i] - agent_pos_vrb[i]);
//     }
// }




// void VRB::swarmVF( const std::vector<Eigen::Vector3d> &agent_pos_vrb, Eigen::MatrixXd &v_swarm){
//     double dist = 0;
//     v_swarm = Eigen::MatrixXd::Zero(3, node_num_);
//     for( int i = 0; i < node_num_; i++ ){
//         for( int j = 0; j < node_num_; j++ ){
//             dist = (agent_pos_vrb[i] - agent_pos_vrb[j]).norm();
//             if( i!=j &&  dist < swarm_clearance_ ){
//                 v_swarm.col(i) += swarm_coef_ * exp( - dist * dist / (2 * sigma_2_ * sigma_2_) ) * (agent_pos_vrb[i] - agent_pos_vrb[j]) / dist;
//             } 
//         } 
//     }
// }

// void VRB::updateVel( const Eigen::Vector3d &vrb_origin, const Eigen::Vector3d &vrb_vel,  const std::vector<Eigen::Vector3d> agent_pos_glb ){

//     //Get coordinates in VRB frame
//     glb2vrb( vrb_origin, agent_pos_glb, agent_pos_vrb_ );

//     //Generate vector fields
//     formVF( agent_pos_vrb_, v_form_ );
//     swarmVF( agent_pos_vrb_, v_swarm_ ); 

//     //Consider the VRB velocity
//     v_des_ = Eigen::MatrixXd::Zero( 3, node_num_ );
//     v_des_.colwise() += vrb_vel;

//     // v_des = v_form + v_obs + v_swarm + v_des;
//     v_des_ = v_form_ + v_swarm_ + v_des_;

//     //Eliminate the velocity on z-axis
//     v_des_.row(2) = Eigen::MatrixXd::Zero( 1, node_num_ );

// }
