#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <vrb_planner/vrb_planner.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "vrb_planner_node");
  ros::NodeHandle nh("~");

  vrbPlanner vrb_planner;
  vrb_planner.init(nh);

  ros::spin();
  
  return 0;
}
