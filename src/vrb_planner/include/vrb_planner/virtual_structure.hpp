#ifndef _VIRTUAL_STRUCTURE_H_
#define _VIRTUAL_STRUCTURE_H_

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <plan_env/grid_map.h>
// #include <>

// Input:  drone0 as: vrb_origin(pos, vel)
//         pos of all the agent
class VRB{

private:
    std::vector<Eigen::Vector3d> form_des_;
    
    double t_interval_;

    double form_coef_;
    double obs_coef_;
    double swarm_coef_;

    double sigma_1_;
    double sigma_2_;

    double swarm_clearance_;
    double obs_clearance_;
    double obs_closure_;

    int node_num_;
    int drone_id_;
    
    GridMap::Ptr grid_map_;

    //Swarm pos & drone pos in VRB local frame
    std::vector<Eigen::Vector3d> swarm_pos_vrb_;
    Eigen::Vector3d odom_pos_vrb_;

public:
    VRB(){}
    ~VRB(){}

    void init( ros::NodeHandle &nh );
    void setGridMap( GridMap::Ptr grid_map );

    //Get the desired position of agents IN GLOBAL FRAME just for plotting
    // void getDesPos( const Eigen::Vector3d &vrb_origin, std::vector<Eigen::Vector3d> &agent_des_pos );

    void setForm( const std::vector<Eigen::Vector3d> &form_nodes );

    //Transform pos from global frame to VRB frame
    void glb2vrb( const Eigen::Vector3d &vrb_origin, const std::vector<Eigen::Vector3d> &global_pos, std::vector<Eigen::Vector3d> &vrb_pos);
    void glb2vrb( const Eigen::Vector3d &vrb_origin, const Eigen::Vector3d &global_pos, Eigen::Vector3d &vrb_pos );
    
    //Formation vector field
    void formVF( const Eigen::Vector3d &agent_pos_vrb, Eigen::Vector3d &v_form );

    //Inter-agent vector field
    void swarmVF( const std::vector<Eigen::Vector3d> &swarm_pos_vrb, Eigen::Vector3d &v_swarm );

    //Obstacle vector field
    void obsVF( const Eigen::Vector3d &agent_pos_glb, Eigen::Vector3d &v_obs, Eigen::Vector3d &v_form );
    
    //Adjust vector field if no avoidance achieved
    void adjustVF( const Eigen::Vector3d &v_form, Eigen::Vector3d &v_obs );

    //Update the velocity cmd
    bool updateVel( const Eigen::Vector3d &vrb_origin, const Eigen::Vector3d &vrb_vel, const Eigen::Vector3d &vrb_acc,
                    const std::vector<Eigen::Vector3d> &swarm_pos_glb, const Eigen::Vector3d &odom_pos_glb,
                    Eigen::Vector3d &v_cmd, Eigen::Vector3d &formation_vel,  Eigen::Vector3d &swarm_vel,
                    Eigen::Vector3d &obs_vel);

    typedef std::unique_ptr<VRB> Ptr;    
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};


#endif