#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

//standart c++ library
#include <iostream>
#include <limits>
#include <vector>
#include <string>
#include <algorithm>

#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "std_msgs/Float64.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/tf.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>


namespace vec_control {
using namespace std;

class PurePursuit {

private:
/*

ld_gain	->	lookahead distance gain to depend the lookahead distance on speed ld = ld_gain * car_speed
min_ld ->	Minimum lookahead distance
car_wheel_base	->	The distance between the car's front and back wheels
controller_freq	->	The controller Frequency
map_frame	->	map frame name
base_frame -> base link frame name

*/

  double ld_gain_;//lookahead distance gain
  double ld_;     //lookahead distance
  double min_ld_; //min value of lookahead distance
  double car_wheel_base_;
  double alpha_;
  double car_speed_;
  float steering_angle;
  int controller_freq_;
  int point_idx_;
  int last_p_idx_;
  double last_dist_ = std::numeric_limits<double>::infinity();
  bool got_path_ = false;
  bool path_done_ = true;
  bool loop_ = false;
  
  std::string odom_frame_ = "odom";
  std::string map_frame_ = "map";
  std::string base_frame_ = "base_footprint";
  std::vector<geometry_msgs::PoseStamped> path_;
  geometry_msgs::PoseStamped target_point_;
  //ackermann_msgs::AckermannDriveStamped control_msg_;
  geometry_msgs::Twist control_msg_;
  geometry_msgs::PointStamped lookahead_p;

  //ros instruction
  ros::Time last_msg_time_;
  ros::Publisher control_pub_;
  ros::Publisher path_pub_;
  ros::Publisher l_point_pub_;
  ros::Publisher current_speed_pub_;
  ros::Subscriber ackermann_sub_;
  ros::Rate *ros_rate_;
  geometry_msgs::TransformStamped base_location_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener *tfListener_;

  //odometry calculation
  void odom_clk_(const nav_msgs::Odometry::ConstPtr &msg);
  //path calculation
  void path_clk_(const nav_msgs::Path::ConstPtr &msg);
  //control loop for the algorithm itself
  void control_loop_();

public:

  PurePursuit();//constructor
  virtual ~PurePursuit();//deconstructor
};

template <typename T1, typename T2> double distance(T1 pt1, T2 pt2) {
  //euclidian distance calculation.
  return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) +
              pow(pt1.z - pt2.z, 2));
}

} // namespace vec_control

#endif
