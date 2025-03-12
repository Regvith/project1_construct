#include "my_rb1_ros/Rotate.h"
#include <cmath> // For M_PI
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

ros::Publisher pub;
double current_yaw = 0;
double initial_yaw = 0;
// Convert Quaternion to Yaw
bool odom_msg = false;
double Q_to_yaw(double x, double y, double z, double w) {
  return atan2((2 * (w * z + x * y)), 1 - 2 * (y * y + z * z));
}
//  Odometry Callback Function
void cb_sub(const nav_msgs::Odometry::ConstPtr &msg) {
  double x = msg->pose.pose.orientation.x;
  double y = msg->pose.pose.orientation.y;
  double z = msg->pose.pose.orientation.z;
  double w = msg->pose.pose.orientation.w;
  current_yaw = Q_to_yaw(x, y, z, w);
  odom_msg = true;
}

//  Service Callback Function: Rotates the Robot
bool cb(my_rb1_ros::Rotate::Request &req, my_rb1_ros::Rotate::Response &res) {
  ROS_INFO("Rotate Service Called: Rotating %d degrees", req.degrees);
  if (!odom_msg) {
    ROS_ERROR("Odometry data not received. Cannot proceed with rotation.");
    res.response = "Failed: No odometry data.";
    return false;
  }

  // Convert degrees to radians
  double target_rotation = req.degrees * (M_PI / 180.0);
  initial_yaw = current_yaw;
  double target_yaw = initial_yaw + target_rotation;
  //  Setup Publisher and Rotation Command
  geometry_msgs::Twist move;
  move.linear.x = 0.0;
  move.angular.z = (target_rotation > 0 ? 0.1 : -0.1);
  if (target_yaw > M_PI)
    target_yaw -= 2 * M_PI;
  if (target_yaw < -M_PI)
    target_yaw += 2 * M_PI;
  ros::Rate rate(20); // 10 Hz loop rate

  //  Rotate Until Target Yaw is Reached
  while (ros::ok()) {
    double yaw_error = target_yaw - current_yaw;
    if (yaw_error > M_PI)
      yaw_error -= 2 * M_PI;
    if (yaw_error < -M_PI)
      yaw_error += 2 * M_PI;

    if (fabs(yaw_error) < 0.03) {
      ROS_INFO("Rotations is over");
      break;
    }
    // Stop condition

    pub.publish(move);
    ros::spinOnce();
    rate.sleep();
  }

  // Stop Rotation After Reaching Target
  move.angular.z = 0.0;
  pub.publish(move);

  ROS_INFO("Rotation Success!");
  res.response = "successfully completed";
  return true;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "cmd_node");
  ros::NodeHandle nh;

  //  Initialize Publisher & Subscriber
  pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("/odom", 10, cb_sub);

  //  Start the Service Server
  ros::ServiceServer ser = nh.advertiseService("/rotate_robot", cb);
  ROS_INFO("Rotate Service Server Ready.");

  ros::spin(); // Keep the node running
  return 0;
}
