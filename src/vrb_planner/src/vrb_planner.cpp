#include <vrb_planner/vrb_planner.h>

void vrbPlanner::init(ros::NodeHandle &nh){
    /* read algorithm parameters */
    nh.param("manager/drone_id", drone_id_, -1);
    nh.param("time_interval", time_interval_, 0.001);

    nh.param("fsm/waypoint0_x", goal_(0), -1.0);
    nh.param("fsm/waypoint0_y", goal_(1), -1.0);
    nh.param("fsm/waypoint0_z", goal_(2), -1.0);
    /* initialize main modules */
    visualization_.reset(new ego_planner::PlanningVisualization(nh));
    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    /* initialize data */
    // have_time_ = false;
    have_odom_      = false;
    have_vrb_odom_  = false;
    occ_flag = false;

    have_swarm_0_ = have_swarm_1_ = have_swarm_2_ = have_swarm_3_ = false;
    have_swarm_4_ = have_swarm_5_ = have_swarm_6_ = false;
    formation_size_ = 7; // 7 for regular hexagon
    swarm_pos_.resize(formation_size_);
    swarm_vel_.resize(formation_size_);
    
    /* ros setting */
    vrb_sub_     = nh.subscribe("/drone_0_visual_slam/odom", 1, &vrbPlanner::vrbOdometryCallback, this);
    swarm_1_sub_ = nh.subscribe("/drone_1_visual_slam/odom", 1, &vrbPlanner::swarm1odometryCallback, this);
    swarm_2_sub_ = nh.subscribe("/drone_2_visual_slam/odom", 1, &vrbPlanner::swarm2odometryCallback, this);
    swarm_3_sub_ = nh.subscribe("/drone_3_visual_slam/odom", 1, &vrbPlanner::swarm3odometryCallback, this);

    swarm_4_sub_ = nh.subscribe("/drone_4_visual_slam/odom", 1, &vrbPlanner::swarm4odometryCallback, this);
    swarm_5_sub_ = nh.subscribe("/drone_5_visual_slam/odom", 1, &vrbPlanner::swarm5odometryCallback, this);
    swarm_6_sub_ = nh.subscribe("/drone_6_visual_slam/odom", 1, &vrbPlanner::swarm6odometryCallback, this);
    swarm_7_sub_ = nh.subscribe("/drone_7_visual_slam/odom", 1, &vrbPlanner::swarm7odometryCallback, this);
    odom_sub_       = nh.subscribe("odom_world", 1, &vrbPlanner::odometryCallback, this);
    cmd_pub_        = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
    cmd_vis_pub_    = nh.advertise<visualization_msgs::Marker> ("/position_cmd_vis", 50);
    
    planner_timer_ = nh.createTimer(ros::Duration(time_interval_), &vrbPlanner::vrbCmdCallback, this);

    /* vrb initialization :
        input: nh,
               grid_map_;    
    */
    vrb_.reset(new VRB);
    vrb_->init(nh);
    vrb_->setGridMap(grid_map_);
    
    // Set desired formation for the swarm
    // Eigen::Vector3d node0(0,0,0);
    // Eigen::Vector3d node1(0,2,0);
    // Eigen::Vector3d node2(2,0,0);
    // std::vector<Eigen::Vector3d> node_des;
    // node_des.push_back(node0);
    // node_des.push_back(node1);
    // node_des.push_back(node2);

    Eigen::Vector3d node1(0,0,0);
    Eigen::Vector3d node2(2.6,-1.5,0);
    Eigen::Vector3d node3(0,-3,0);
    Eigen::Vector3d node4(-2.6,-1.5,0);
    Eigen::Vector3d node5(-2.6,1.5,0);
    Eigen::Vector3d node6(0,3,0);
    Eigen::Vector3d node7(2.6,1.5,0);
    std::vector<Eigen::Vector3d> node_des;
    node_des.push_back(node1);
    node_des.push_back(node2);
    node_des.push_back(node3);
    node_des.push_back(node4);
    node_des.push_back(node5);
    node_des.push_back(node6);
    node_des.push_back(node7);

    vrb_->setForm( node_des );
    vel_cmd_last_ << 0.0 , 0.0 , 0.0;
}
    
void vrbPlanner::vrbCmdCallback(const ros::TimerEvent &e){

    //  if( !have_time_){
    //     time_1_ = ros::Time::now();
    //     have_time_ = true;
    // }

    if (!have_vrb_odom_ || !have_odom_)
        return;
    
    if (!have_swarm_0_ || !have_swarm_1_ || !have_swarm_2_ || !have_swarm_3_ || !have_swarm_4_ || !have_swarm_5_ || !have_swarm_6_)
        return;
    // if (!have_swarm_0_ || !have_swarm_1_ || !have_swarm_2_ )
    //     return;

    if ( (odom_pos_ - goal_).norm() < 0.1 )
        return;

    bool success = true;
    Eigen::Vector3d pos_cmd;
    Eigen::Vector3d vel_cmd;
    Eigen::Vector3d formation_vel, swarm_vel, obs_vel;

    // std::cout << "vrb.acc " << vrb_acc_(0) << " , " << vrb_acc_(1) << " , " << vrb_acc_(2) << " , " << std::endl;
    
    /* vrb planning 
    
        input  : swarm_pos_, swarm_vel_, 
                 odom_pos_, odom_vel_,
                 vrb_pos_, vrb_vel_;
        output : vel_cmd, (or and pos_cmd)
    */

    if(!occ_flag){
         occ_flag = grid_map_->getInflateOccupancy(odom_pos_);
    }
    

    success = vrb_->updateVel(vrb_pos_, vrb_vel_, vrb_acc_, swarm_pos_,odom_pos_,vel_cmd, formation_vel, swarm_vel, obs_vel);
    
    /* publish the pos_cmd */
    
    quadrotor_msgs::PositionCommand cmd;
    ros::Time time_now = ros::Time::now();
    // time_2_ = time_now;

    cmd.header.stamp = time_now;
    cmd.header.frame_id = "world";
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd.trajectory_id = drone_id_;  
    
    if (!occ_flag){

        // double t_duration = (time_2_ - time_1_).toSec();
        cmd.position.x = odom_pos_(0) + time_interval_ * vel_cmd(0);
        cmd.position.y = odom_pos_(1) + time_interval_ * vel_cmd(1);
        cmd.position.z = odom_pos_(2);

        cmd.velocity.x = vel_cmd(0);
        cmd.velocity.y = vel_cmd(1);
        cmd.velocity.z = 0.0;

        // time_1_ = time_2_;

        // vel_cmd_last_ << cmd.velocity.x , cmd.velocity.y , cmd.velocity.z;

        /* use for drawCmd */
        Eigen::Vector3d pos_vis, vel_vis;
        pos_vis(0) = cmd.position.x;
        pos_vis(1) = cmd.position.y;
        pos_vis(2) = cmd.position.z;
        vel_vis(0) = vel_cmd(0);
        vel_vis(1) = vel_cmd(1);
        vel_vis(2) = 0.0;
        // vel cmd
        drawCmd(pos_vis, vel_vis, 0, Eigen::Vector4d(0, 0, 1, 1));
        // formation_vel
        formation_vel(2) = 0.0;
        drawCmd(pos_vis, formation_vel, 1, Eigen::Vector4d(0, 1, 1, 0.5));
        // swarm_vel
        // drawCmd(pos_vis, swarm_vel, 2, Eigen::Vector4d(0, 1, 0, 0.5));
        // obs_vel
        obs_vel(2) = 0.0;
        drawCmd(pos_vis, obs_vel, 3, Eigen::Vector4d(1, 0, 0, 0.5));

    } else {
        cmd.position.x = odom_pos_(0);
        cmd.position.y = odom_pos_(1);
        cmd.position.z = odom_pos_(2);
    }

    cmd_pub_.publish(cmd); 
}

void vrbPlanner::vrbOdometryCallback(const nav_msgs::OdometryConstPtr &msg){
    vrb_pos_(0) = msg->pose.pose.position.x;
    vrb_pos_(1) = msg->pose.pose.position.y;
    vrb_pos_(2) = msg->pose.pose.position.z;
    
    vrb_vel_(0) = msg->twist.twist.linear.x;
    vrb_vel_(1) = msg->twist.twist.linear.y;
    vrb_vel_(2) = msg->twist.twist.linear.z;

    vrb_acc_(0) = msg->twist.twist.angular.x;
    vrb_acc_(1) = msg->twist.twist.angular.y;
    vrb_acc_(2) = msg->twist.twist.angular.z;
     
    have_vrb_odom_ = true;
}

void vrbPlanner::odometryCallback(const nav_msgs::OdometryConstPtr &msg){
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;
    
    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;
    
    have_odom_ = true;
}

void vrbPlanner::swarm1odometryCallback(const nav_msgs::OdometryConstPtr &msg){
    swarm_pos_[0](0) = msg->pose.pose.position.x;
    swarm_pos_[0](1) = msg->pose.pose.position.y;
    swarm_pos_[0](2) = msg->pose.pose.position.z;
    
    swarm_vel_[0](0) = msg->twist.twist.linear.x;
    swarm_vel_[0](1) = msg->twist.twist.linear.y;
    swarm_vel_[0](2) = msg->twist.twist.linear.z;
    
    have_swarm_0_ = true;
}

void vrbPlanner::swarm2odometryCallback(const nav_msgs::OdometryConstPtr &msg){
    swarm_pos_[1](0) = msg->pose.pose.position.x;
    swarm_pos_[1](1) = msg->pose.pose.position.y;
    swarm_pos_[1](2) = msg->pose.pose.position.z;
    
    swarm_vel_[1](0) = msg->twist.twist.linear.x;
    swarm_vel_[1](1) = msg->twist.twist.linear.y;
    swarm_vel_[1](2) = msg->twist.twist.linear.z;
    
    have_swarm_1_ = true;
}

void vrbPlanner::swarm3odometryCallback(const nav_msgs::OdometryConstPtr &msg){
    swarm_pos_[2](0) = msg->pose.pose.position.x;
    swarm_pos_[2](1) = msg->pose.pose.position.y;
    swarm_pos_[2](2) = msg->pose.pose.position.z;
    
    swarm_vel_[2](0) = msg->twist.twist.linear.x;
    swarm_vel_[2](1) = msg->twist.twist.linear.y;
    swarm_vel_[2](2) = msg->twist.twist.linear.z;
    
    have_swarm_2_ = true;
}

void vrbPlanner::swarm4odometryCallback(const nav_msgs::OdometryConstPtr &msg){
    swarm_pos_[3](0) = msg->pose.pose.position.x;
    swarm_pos_[3](1) = msg->pose.pose.position.y;
    swarm_pos_[3](2) = msg->pose.pose.position.z;
    
    swarm_vel_[3](0) = msg->twist.twist.linear.x;
    swarm_vel_[3](1) = msg->twist.twist.linear.y;
    swarm_vel_[3](2) = msg->twist.twist.linear.z;
    
    have_swarm_3_ = true;
}

void vrbPlanner::swarm5odometryCallback(const nav_msgs::OdometryConstPtr &msg){
    swarm_pos_[4](0) = msg->pose.pose.position.x;
    swarm_pos_[4](1) = msg->pose.pose.position.y;
    swarm_pos_[4](2) = msg->pose.pose.position.z;
    
    swarm_vel_[4](0) = msg->twist.twist.linear.x;
    swarm_vel_[4](1) = msg->twist.twist.linear.y;
    swarm_vel_[4](2) = msg->twist.twist.linear.z;
    
    have_swarm_4_ = true;
}

void vrbPlanner::swarm6odometryCallback(const nav_msgs::OdometryConstPtr &msg){
    swarm_pos_[5](0) = msg->pose.pose.position.x;
    swarm_pos_[5](1) = msg->pose.pose.position.y;
    swarm_pos_[5](2) = msg->pose.pose.position.z;
    
    swarm_vel_[5](0) = msg->twist.twist.linear.x;
    swarm_vel_[5](1) = msg->twist.twist.linear.y;
    swarm_vel_[5](2) = msg->twist.twist.linear.z;
    
    have_swarm_5_ = true;
}

void vrbPlanner::swarm7odometryCallback(const nav_msgs::OdometryConstPtr &msg){
    swarm_pos_[6](0) = msg->pose.pose.position.x;
    swarm_pos_[6](1) = msg->pose.pose.position.y;
    swarm_pos_[6](2) = msg->pose.pose.position.z;
    
    swarm_vel_[6](0) = msg->twist.twist.linear.x;
    swarm_vel_[6](1) = msg->twist.twist.linear.y;
    swarm_vel_[6](2) = msg->twist.twist.linear.z;
    
    have_swarm_6_ = true;
}

void vrbPlanner::drawCmd(const Eigen::Vector3d& pos, 
                         const Eigen::Vector3d& vec,
                         int id,
                         const Eigen::Vector4d& color){
    visualization_msgs::Marker mk_state;
    mk_state.header.frame_id = "world";
    mk_state.header.stamp = ros::Time::now();
    mk_state.id = id;
    mk_state.type = visualization_msgs::Marker::ARROW;
    mk_state.action = visualization_msgs::Marker::ADD;

    mk_state.pose.orientation.w = 1.0;
    mk_state.scale.x = 0.1;
    mk_state.scale.y = 0.2;
    mk_state.scale.z = 0.3;

    geometry_msgs::Point pt;
    pt.x = pos(0);
    pt.y = pos(1);
    pt.z = pos(2);
    mk_state.points.push_back(pt);

    double scale = 0.03;

    pt.x = pos(0) + vec(0) * scale;
    pt.y = pos(1) + vec(1) * scale;
    pt.z = pos(2) + vec(2) * scale;
    mk_state.points.push_back(pt);

    mk_state.color.r = color(0);
    mk_state.color.g = color(1);
    mk_state.color.b = color(2);
    mk_state.color.a = color(3);

    cmd_vis_pub_.publish(mk_state);
}
