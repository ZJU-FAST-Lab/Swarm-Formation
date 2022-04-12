#include <vrb_planner/vrb_planner.h>
#include <vrb_planner/virtual_structure.hpp>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    Eigen::Vector3d v0(0,0,0);
    Eigen::Vector3d v1(0,1,0);
    Eigen::Vector3d v2(1,0,0);
    std::vector<Eigen::Vector3d> form_des;
    form_des.push_back(v0);
    form_des.push_back(v1);
    form_des.push_back(v2);

    Eigen::Vector3d v3(0,0,0);
    Eigen::Vector3d v4(0,1,0);
    Eigen::Vector3d v5(1,0,0);
    std::vector<Eigen::Vector3d> agent_cur;
    agent_cur.push_back(v3);
    agent_cur.push_back(v4);
    agent_cur.push_back(v5);

}
