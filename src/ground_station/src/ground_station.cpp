#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

using namespace std;
vector<ros::Subscriber> odom_list_sub_;
ros::Publisher          formation_vis_pub_;
int drone_num_;

void odometryCallback(const nav_msgs::OdometryConstPtr &msg){

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_station");
    ros::NodeHandle nh("~");

    /*  ground station param  */
    nh.param("ground/drone_num", drone_num_, -1);
    for (int i=0; i<drone_num_; i++){
        ros::Subscriber odom_sub = nh.subscribe("/drone_" + to_string(i) + "_visual_slam/odom", 1, odometryCallback);
        odom_list_sub_.push_back(odom_sub);
    }

    // ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}