#include "pure_pursuit/pure_pursuit.h"
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

/*
At this project the 

subscribes to->
TF from map -> odom -> base_link

/pure_pursuit/path(name of the topic)	nav_msgs/Path(type of node)	Target path to follow
/tricycle_controller/odom(name of the topic)		nav_msgs/Odometry(type of node)	To update the lookahead distance depending on the car speed

Publishes to
/pure_pursuit/control(name of the topic)	ackermann_msgs/AckermannDriveStamped(type of node)	Ackermann message contains the steering angle and speed
/pure_pursuit/lookahead_point(name of the topic)	geometry_msgs/PointStamped(type of node)	
Target Lookahead point

*/

vec_control::PurePursuit::PurePursuit() {
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private("~");
  // Node parameters
  nh_private.param<double>("ld_gain", ld_gain_, 1.0);
  nh_private.param<double>("min_ld", min_ld_, 0.5);
  nh_private.param<double>("car_wheel_base", car_wheel_base_, 0.44);
  nh_private.param<int>("controller_freq", controller_freq_, 10);
  nh_private.param<std::string>("map_frame", map_frame_, "map");
  nh_private.param<std::string>("base_frame", base_frame_, "base_link");
  ld_ = min_ld_;
  
  ros_rate_ = new ros::Rate(controller_freq_);
  // Publishers and subscribers
  control_pub_ = nh_.advertise<geometry_msgs::Twist>("/tricycle_controller/cmd_vel", 1);
  //path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("tricycle_controller/odom",1);

  ros::Subscriber odom_sub_ = nh_.subscribe("/tricycle_controller/odom", 1, &PurePursuit::odom_clk_, this);

  ros::Subscriber path_sub_ = nh_.subscribe("/move_base/DWAPlannerROS/local_plan", 1,  &PurePursuit::path_clk_, this);

  tfListener_ = new tf2_ros::TransformListener(tfBuffer_);
  l_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/pure_pursuit/lookahead_point", 1);

  // main loop
  control_loop_();
}
//to obitain the odometry of the vehicle by subsribe to nav_msgs::Odometry, then we calculate the new lookahead distance
void vec_control::PurePursuit::odom_clk_(const nav_msgs::Odometry::ConstPtr &msg){
  car_speed_ = msg->twist.twist.linear.x;
  ld_ = std::max(ld_gain_ * car_speed_, min_ld_);
}
 
//target path to follow , topic_name = /pure_pursuit/path
//type nav_msgs/Path
void vec_control::PurePursuit::path_clk_(const nav_msgs::Path::ConstPtr &msg) {
  ROS_INFO("New path is received.");
  path_ = msg->poses;
  got_path_ = true;
  path_done_ = false;
  point_idx_ = 0;
  double start_end_dist = distance(path_[0].pose.position, path_.back().pose.position);
  ROS_INFO("Start to End Distance: %f", start_end_dist);
  ROS_INFO("Min lookup distance: %f", min_ld_);

  if (start_end_dist <= min_ld_) {
    loop_ = true;
    ROS_INFO("Is Loop: True");
  }
}
//creating a new path to follow by manually
std::vector<geometry_msgs::PoseStamped> getPath(double x_init, double y_init, double yaw_init){
  std::vector<geometry_msgs::PoseStamped> path;
  double x = x_init;
  double y = y_init;
  double yaw = yaw_init;
  double res = 0.1;

  for(size_t i = 0; i < 50; i++){
    if(i < 20){
      x += res;
      y += res;
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "odom";
      pose.header.stamp = ros::Time::now();
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      path.push_back(pose);
    }
    else if(i <= 20 && i > 40){
      yaw = -M_PI_2;
      x += res;
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "odom";
      pose.pose.position.x = x+0.1;
      pose.pose.position.y = y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      path.push_back(pose);


    }
    else{
      yaw = M_PI_2;
      x += res;
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "odom";
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      path.push_back(pose);
    }
  }
  return path;
}

//The main loop that will run and control the vehicle on the map.
void vec_control::PurePursuit::control_loop_() {
  double y_t = 0, ld_2 = 0, delta = 0;
  double distance_ = 0;

  base_location_ = tfBuffer_.lookupTransform(odom_frame_, base_frame_, ros::Time(0), ros::Duration(3.0));
  path_ = getPath(base_location_.transform.translation.x, base_location_.transform.translation.y, tf::getYaw(base_location_.transform.rotation));
  got_path_ = true;
  point_idx_ = 0;

  while (ros::ok()) {
    if (got_path_) {
      //if we got the path
      // get the current robot location by tf base_link -> map
      // iterate over the path points
      // if the distance between a point(waypoint) and robot >  lookahead then break and take
      // this point transform this point to the robot base_link the y component
      // of this point is y_t delta can be computed as atan2(2 * yt * L_, ld_2)
      try {
        base_location_ = tfBuffer_.lookupTransform(
            odom_frame_, base_frame_, ros::Time(0), ros::Duration(3.0));

        for (; point_idx_ < path_.size(); point_idx_++) {
          //the distance between a point(waypoint) and robot
          distance_ = distance(path_[point_idx_].pose.position,
                               base_location_.transform.translation);
          ROS_INFO("Point ID: %d, Distance %f", point_idx_, distance_);
          if (distance_ >= ld_) {//transform this point to the robot base_link 
            path_[point_idx_].header.stamp =
                ros::Time(0); // Set the timestamp to now for the transform
                                  // to work, because it tries to transform the
                                  // point at the time stamp of the input point
            tfBuffer_.transform(path_[point_idx_], target_point_, base_frame_,
                                ros::Duration(0.1));
            break;
          }
        }

        ld_2 = ld_ * ld_;
        y_t = target_point_.pose.position.y;
        delta = atan2(2 * car_wheel_base_ * y_t, ld_2);
        
        /*control_msg_.drive.steering_angle = delta;
        control_msg_.drive.speed = 2;
        control_msg_.header.stamp = ros::Time::now(); */
        control_msg_.linear.x = 0.1;
        control_msg_.angular.z = 0.1 * sin(delta) / car_wheel_base_;
        control_pub_.publish(control_msg_);
/*
        
        geometry_msgs::PoseArray msg;
        msg.header.frame_id = "odom";
        msg.header.stamp = ros::Time::now();
        for(auto p : path_){
          geometry_msgs::Pose pose;
          pose.position.x = p.path_pub_
          msg.poses.push_back(pose);
        }
        path_pub_.publish(msg); 
       */ 

        last_p_idx_ = point_idx_;
        last_dist_ = distance_;
        
        if (point_idx_ == path_.size() && loop_) {
          point_idx_ = 0;
        } 
        else if (point_idx_ == path_.size()) {
          ROS_INFO("Reached final point");
          

          control_msg_.linear.x = 0.0;
          control_msg_.angular.z = 0.0;
          control_pub_.publish(control_msg_);
          
          got_path_ = false;
          point_idx_ = 0;
        }
        lookahead_p.point = path_[point_idx_].pose.position;
        lookahead_p.header = path_[point_idx_].header;
        l_point_pub_.publish(lookahead_p); // Publish the lookahead point
      } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
      }
    }

    ros::spinOnce();
    ros_rate_->sleep();
  }
}
//deconstructor
vec_control::PurePursuit::~PurePursuit() {
  delete tfListener_;
  delete ros_rate_;
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "pure_pursuit");
 
  vec_control::PurePursuit pp_node;

  return 0;
}