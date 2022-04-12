#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <eigen3/Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <stdlib.h>
#include <nav_msgs/Odometry.h>
#include <fstream>

using std::vector;
namespace ego_planner
{
  class PlanningVisualization
  {
  private:
    ros::NodeHandle node;

    ros::Publisher goal_point_pub;
    ros::Publisher global_list_pub;
    ros::Publisher init_list_pub;
    ros::Publisher optimal_list_pub;
    ros::Publisher failed_list_pub;
    ros::Publisher a_star_list_pub;
    ros::Publisher guide_vector_pub;
    ros::Publisher init_list_debug_pub;

    ros::Publisher intermediate_pt0_pub;
    ros::Publisher intermediate_pt1_pub;
    ros::Publisher intermediate_grad0_pub;
    ros::Publisher intermediate_grad1_pub;
    ros::Publisher intermediate_grad_smoo_pub;
    ros::Publisher intermediate_grad_dist_pub;
    ros::Publisher intermediate_grad_feas_pub;
    ros::Publisher intermediate_grad_swarm_pub;

    ros::Publisher swarm_formation_visual_pub;

    enum FORMATION_TYPE
    {
      NONE_FORMATION        = 0,
      REGULAR_HEXAGON       = 1
    };

    int drone_id_;
    int formation_type_;
    int formation_size_, line_size_;
    std::vector<int> line_begin_, line_end_;
    bool start_visual_;
    
    ros::Subscriber drone_0_odom_sub_, drone_1_odom_sub_, drone_2_odom_sub_, drone_3_odom_sub_; 
    ros::Subscriber drone_4_odom_sub_, drone_5_odom_sub_, drone_6_odom_sub_, drone_7_odom_sub_;
    ros::Subscriber drone_8_odom_sub_, drone_9_odom_sub_, drone_10_odom_sub_, drone_11_odom_sub_;

    ros::Timer swarm_graph_visual_timer_;
    ros::Timer benchmark_recorder;

    std::ofstream odom_csv;
    ros::Time t_init;
    ros::Time t_record;

    std::vector<Eigen::Vector3d> swarm_odom;

    void drone_0_odomeCallback(const nav_msgs::OdometryConstPtr &msg);
    void drone_1_odomeCallback(const nav_msgs::OdometryConstPtr &msg);
    void drone_2_odomeCallback(const nav_msgs::OdometryConstPtr &msg);
    void drone_3_odomeCallback(const nav_msgs::OdometryConstPtr &msg);
    void drone_4_odomeCallback(const nav_msgs::OdometryConstPtr &msg);
    void drone_5_odomeCallback(const nav_msgs::OdometryConstPtr &msg);
    void drone_6_odomeCallback(const nav_msgs::OdometryConstPtr &msg);
    void drone_7_odomeCallback(const nav_msgs::OdometryConstPtr &msg);
    void drone_8_odomeCallback(const nav_msgs::OdometryConstPtr &msg);
    void drone_9_odomeCallback(const nav_msgs::OdometryConstPtr &msg);
    void drone_10_odomeCallback(const nav_msgs::OdometryConstPtr &msg);
    void drone_11_odomeCallback(const nav_msgs::OdometryConstPtr &msg);
    
    void swarmGraphVisulCallback(const ros::TimerEvent &e);
    void benchmarkCallback(const ros::TimerEvent &e);

  public:

    PlanningVisualization(/* args */) {}
    ~PlanningVisualization() { 
      if (drone_id_ == 1){ odom_csv.close(); }
     }

    PlanningVisualization(ros::NodeHandle &nh);

    typedef std::shared_ptr<PlanningVisualization> Ptr;

    void initSwarmGraphVisual();

    void displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id,  bool show_sphere = true);
    void generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                  const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void generateArrowDisplayArray(visualization_msgs::MarkerArray &array,
                                   const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
    void displayGlobalPathList(vector<Eigen::Vector3d> global_pts, const double scale, int id);
    void displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id);
    void displayMultiInitPathList(vector<vector<Eigen::Vector3d>> init_trajs, const double scale);
    void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);
    void displayFailedList(Eigen::MatrixXd failed_pts, int id);
    void displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id);
    void displayArrowList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void displayInitPathListDebug(vector<Eigen::Vector3d> init_pts, const double scale, int id);

    void displayIntermediatePt(std::string type, Eigen::MatrixXd &pts, int id, Eigen::Vector4d color);
    void displayIntermediateGrad(std::string type, Eigen::MatrixXd &pts, Eigen::MatrixXd &grad, int id, Eigen::Vector4d color);
    // void displayNewArrow(ros::Publisher& guide_vector_pub, ego_planner::PolyTrajOptimizer::Ptr optimizer);
  };
} // namespace ego_planner
#endif