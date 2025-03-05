/// \file
/// \brief  publishes a body twist at a fixed frequency
///
/// \author Boston Cleek
/// \date 1/24/20
///
/// PUBLISHES:
///   joint_states (sensor_msgs/JointState): angular wheel positions
///
/// SUBSCRIBES:
///   cmd_vel (geometry_msgs/Twist): twist in body frame


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/joint_state.h>

#include <string>
#include <vector>
#include <iostream>

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"



// global variables
static std::string left_wheel_joint, right_wheel_joint;     // joint names
static rigid2d::Twist2D cmd;                                // body cmd_vel
static bool message;                                        // callback flag


/// \brief updates the body twist of the robot
/// \param msg - contains the encoder readings and joint names
void twistCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  cmd.w = msg->angular.z;
  cmd.vx = msg->linear.x;
  cmd.vy = msg->linear.y; // should be 0

  // ROS_INFO("twistCallback");

  message = true;
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_diff_encoders");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

  ros::Subscriber twist_sub = node_handle.subscribe("cmd_vel", 1, twistCallback);
  ros::Publisher joint_pub = node_handle.advertise<sensor_msgs::JointState>("joint_states", 1);

  double wheel_base = 0.0, wheel_radius = 0.0;

  nh.getParam("left_wheel_joint", left_wheel_joint);
  nh.getParam("right_wheel_joint", right_wheel_joint);

  node_handle.getParam("/wheel_base", wheel_base);
  node_handle.getParam("/wheel_radius", wheel_radius);


  ROS_INFO("left_wheel_joint %s", left_wheel_joint.c_str());
  ROS_INFO("right_wheel_joint %s", right_wheel_joint.c_str());

  ROS_INFO("wheel_base %f", wheel_base);
  ROS_INFO("wheel_radius %f", wheel_radius);

  ROS_INFO("Successfully launched fake_diff_encoders node.");

  // check if message has been recieved
  message = false;

  // Assume pose starts at (0, 0, 0)
  rigid2d::Pose pose;
  pose.theta = 0;
  pose.x = 0;
  pose.y = 0;


  // diff drive model
  rigid2d::DiffDrive drive(pose, wheel_base, wheel_radius);


  // timing
  int frequency = 60;
  ros::Rate loop_rate(frequency);

  // timing
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();


  while(node_handle.ok())
  {
    ros::spinOnce();
    current_time = ros::Time::now();

    if (message)
    {
      // scale twist based on frequency
      rigid2d::Twist2D scaled;
      scaled.vx = cmd.vx * 1.0 / frequency;
      scaled.w = cmd.w * 1.0 / frequency;

      // propogate kinematics
      drive.feedforward(scaled);


      // pose = drive.pose();
      // std::cout << "------------------" << std::endl;
      // std::cout << "fake_diff_encoders" << std::endl;
      // std::cout << pose.theta << " " << pose.x << " " << pose.y << std::endl;
      // std::cout << "------------------" << std::endl;


      // wheel encoder readings
      rigid2d::WheelEncoders encoders = drive.getEncoders();

      // WheelVelocities encoders drive.twistToWheels(scaled);


      sensor_msgs::JointState joint_state;
      joint_state.header.stamp = current_time;
      joint_state.name.push_back(left_wheel_joint);
      joint_state.name.push_back(right_wheel_joint);

      joint_state.position.push_back(encoders.left);
      joint_state.position.push_back(encoders.right);

      message = false;

      joint_pub.publish(joint_state);
    }

    last_time = current_time;
    loop_rate.sleep();
  }

  return 0;
}













// end file
