#ifndef _VRB_PLANNER_H_
#define _VRB_PLANNER_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <optimizer/poly_traj_optimizer.h>
#include <plan_env/grid_map.h>
#include <geometry_msgs/PoseStamped.h>
#include <traj_utils/planning_visualization.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <vrb_planner/virtual_structure.hpp>

class vrbPlanner
{
private:
    /* planning utils */
    ego_planner::PlanningVisualization::Ptr visualization_;
    GridMap::Ptr grid_map_;
    VRB::Ptr     vrb_;

    /* ROS utils */
    ros::Subscriber vrb_sub_, odom_sub_;
    ros::Subscriber swarm_1_sub_, swarm_2_sub_, swarm_3_sub_, swarm_4_sub_, swarm_5_sub_, swarm_6_sub_, swarm_7_sub_;
    ros::Publisher  cmd_pub_, cmd_vis_pub_;
    ros::Timer      planner_timer_;

    /* planner data*/
    bool have_odom_, have_vrb_odom_;
    bool have_swarm_0_, have_swarm_1_, have_swarm_2_, have_swarm_3_, have_swarm_4_, have_swarm_5_, have_swarm_6_;
    int drone_id_;
    Eigen::Vector3d vrb_pos_, vrb_vel_, vrb_acc_;
    Eigen::Vector3d odom_pos_, odom_vel_;
    Eigen::Vector3d goal_;
    int formation_size_;

    // Velocity related
    Eigen::Vector3d vel_cmd_last_;
    double time_interval_;

    //If time initialized
    bool have_time_;
    ros::Time time_1_;
    ros::Time time_2_;

    bool occ_flag;
    

    std::vector<Eigen::Vector3d> swarm_pos_, swarm_vel_;

    /* useful function */
    void vrbOdometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void swarm1odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void swarm2odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void swarm3odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void swarm4odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void swarm5odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void swarm6odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void swarm7odometryCallback(const nav_msgs::OdometryConstPtr &msg);
        
    void vrbCmdCallback(const ros::TimerEvent &e);

    void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, int id, const Eigen::Vector4d& color);



public:
    vrbPlanner(){}
    ~vrbPlanner(){}

    void init(ros::NodeHandle &nh);
    

};


#endif