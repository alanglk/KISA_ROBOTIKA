/// \file
/// \brief EKF SLAM
///
/// \author Boston Cleek
/// \date 3/22/20
///
/// PARAMETERS:
///   map_frame_id - map frame
///   odom_frame_id - odometry frame
///   body_frame_id - base link frame
///   left_wheel_joint - name of left wheel joint
///   right_wheel_joint - name of right wheel joint
///   wheel_base - distance between wheels
///   wheel_radius - radius of wheels
///   known_data_association - EKF runs with or without know data association
/// PUBLISHES:
///   slam_path (nav_msgs/Path): trajectory from EKF slam
///   odom_path (nav_msgs/Path): trajectory from odometry
///   gazebo_path (nav_msgs/Path): trajectory from gazebo
///   map (visualization_msgs::MarkerArray): landmarks states from EKF represented as cylinders
///   odom_error (tsim/PoseError): pose error between gazebo and odometry
///   slam_error (tsim/PoseError): pose error between gazebo and EKF slam
/// SUBSCRIBES:
///   joint_states (sensor_msgs/JointState): angular wheel positions
///   landmarks (nuslam::TurtleMap): center and radius of all circles detected
///   /gazebo/model_states (gazebo_msgs/ModelStates): model states from grazebo



#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <vector>
#include <iostream>
#include <exception>


#include <rigid2d/diff_drive.hpp>
#include "landmarks/ekf_filter.hpp"
#include "landmarks/LandmarkMap.h"
#include "tsim/PoseError.h"


using rigid2d::Twist2D;
using rigid2d::Vector2D;
using rigid2d::Pose;
using rigid2d::Transform2D;
using rigid2d::TransformData2D;
using rigid2d::normalize_angle_PI;


static std::string left_wheel_joint, right_wheel_joint;    // joint names
static double left, right;                                 // wheel angular positions for odometry
static double ekf_left, ekf_right;                         // wheel angular positions for SLAM

static bool wheel_odom_flag;                               // odometry update

static std::vector<Vector2D> meas;                         // x/y locations of cylinders relative to robot
static bool map_flag;                                      // map update flag

static bool known_data_association;                        // EKF runs with or without know data association
static geometry_msgs::PoseStamped gazebo_robot_pose;       // pose of robot in gazebo


/// \brief  Retreive gazebo robot pose
/// \param model_data - model states in world
void modelCallBack(const gazebo_msgs::ModelStates::ConstPtr& model_data)
{
  // store names of all items in gazebo
  std::vector<std::string> names = model_data->name;

  // index of robot
  int robot_index = 0;

  // find diff_drive robot
  int ctr = 0;
  for(const auto &item : names)
  {
    // check for robot
    if (item == "diff_drive")
    {
      robot_index = ctr;
    }

    ctr++;
  } // end loop


  // pose of robot
  gazebo_robot_pose.header.stamp = ros::Time::now();
  gazebo_robot_pose.pose.position = model_data->pose[robot_index].position;
  gazebo_robot_pose.pose.orientation = model_data->pose[robot_index].orientation;
}


/// \brief Update the map
/// \param msg -recent map
void mapCallBack(const landmarks::LandmarkMap::ConstPtr &map_msg)
{
  // clear previous measurement
  meas.clear();
  meas.reserve(map_msg->r.size());
  for(unsigned int i = 0; i < map_msg->r.size(); i++)
  {
    Vector2D m;
    m.x = map_msg->cx.at(i);
    m.y = map_msg->cy.at(i);
    meas.push_back(m);
  }

  map_flag = true;
}



/// \brief updates the wheel encoder angles
/// \param msg - contains the encoder readings and joint names
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<std::string> names = msg->name;
  std::vector<std::string>::iterator iter;
  int left_idx, right_idx;

  iter = std::find(names.begin(), names.end(), left_wheel_joint);
  left_idx = std::distance(names.begin(), iter);

  iter = std::find(names.begin(), names.end(), right_wheel_joint);
  right_idx = std::distance(names.begin(), iter);

  if (left_idx > 1)
  {
    throw std::invalid_argument("Left wheel index not found in Odometer.");
  }

  if (right_idx > 1)
  {
    throw std::invalid_argument("Right wheel index not found in Odometer.");
  }


  // ROS_INFO("left index: %d", left_idx);
  // ROS_INFO("right index: %d", right_idx);


  left = msg->position.at(left_idx);
  right = msg->position.at(right_idx);


  wheel_odom_flag = true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "slam");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

  /////////////////////////////////////////////////////////////////////////////

  ros::Subscriber joint_sub = node_handle.subscribe("joint_states", 1, jointStatesCallback);
  ros::Subscriber map_sub = node_handle.subscribe("landmarks", 1, mapCallBack);
  ros::Subscriber scan_sub = nh.subscribe("/gazebo/model_states", 1, modelCallBack);

  ros::Publisher slam_path_pub = node_handle.advertise<nav_msgs::Path>("slam_path", 10);
  ros::Publisher odom_path_pub = node_handle.advertise<nav_msgs::Path>("odom_path", 10);
  ros::Publisher gazebo_path_pub = node_handle.advertise<nav_msgs::Path>("gazebo_path", 10);
  ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::MarkerArray>("slam_map", 10);

  ros::Publisher odom_error_pub = node_handle.advertise<tsim::PoseError>("odom_error", 1);
  ros::Publisher slam_error_pub = node_handle.advertise<tsim::PoseError>("slam_error", 1);

  /////////////////////////////////////////////////////////////////////////////

  std::string map_frame_id, odom_frame_id, marker_frame_id;
  auto wheel_base = 0.0, wheel_radius = 0.0;

  nh.getParam("left_wheel_joint", left_wheel_joint);
  nh.getParam("right_wheel_joint", right_wheel_joint);

  nh.getParam("map_frame_id", map_frame_id);
  nh.getParam("odom_frame_id", odom_frame_id);
  nh.getParam("marker_frame_id", marker_frame_id);

  nh.getParam("known_data_association", known_data_association);


  node_handle.getParam("/wheel_base", wheel_base);
  node_handle.getParam("/wheel_radius", wheel_radius);


  ROS_INFO("map_frame_id %s", map_frame_id.c_str());
  ROS_INFO("odom_frame_id %s", odom_frame_id.c_str());
  ROS_INFO("marker_frame_id %s", marker_frame_id.c_str());

  ROS_INFO("left_wheel_joint %s", left_wheel_joint.c_str());
  ROS_INFO("right_wheel_joint %s", right_wheel_joint.c_str());

  ROS_INFO("wheel_base %f", wheel_base);
  ROS_INFO("wheel_radius %f", wheel_radius);

  ROS_INFO("known_data_association %d", known_data_association);


  ROS_INFO("Successfully launched slam node");

  /////////////////////////////////////////////////////////////////////////////

  tf2_ros::TransformBroadcaster map_odom_broadcaster;
  map_flag = false;
  wheel_odom_flag = false;


  // Assume pose starts at (0,0,0)
  // for odometry
  rigid2d::Pose pose;
  pose.theta = 0;
  pose.x = 0;
  pose.y = 0;


  // diff drive model for odometry
  rigid2d::DiffDrive drive(pose, wheel_base, wheel_radius);
  // diff drive model for SLAM
  rigid2d::DiffDrive ekf_drive(pose, wheel_base, wheel_radius);


  // number of landmarks in model
  int n = 25;
  double md_max = 1e7;//0.30;
  double md_min = 20000.0;//0.05;
  landmarks::EKF ekf(n, md_max, md_min);


  // path from odometry
  nav_msgs::Path odom_path;

  // path from SLAM
  nav_msgs::Path ekf_path;

  // path from gazebo
  nav_msgs::Path gazebo_path;

  // error in pose
  tsim::PoseError odom_error_msg;
  tsim::PoseError slam_error_msg;

  /////////////////////////////////////////////////////////////////////////////

  while(node_handle.ok())
  {
    ros::spinOnce();

    /////////////////////////////////////////////////////////////////////////////


    if (wheel_odom_flag)
    {
      // most recent odom update
      drive.updateOdometry(left, right);
      pose = drive.pose();


      // IMPORTANT: set ekf wheel encoder to current odometry encoders
      ekf_left = left;
      ekf_right = right;

      // update ekf with odometry and sensor measurements
      if (map_flag)
      {
        ekf_drive.updateOdometry(ekf_left, ekf_right);
        rigid2d::WheelVelocities vel = ekf_drive.wheelVelocities();
        rigid2d::Twist2D vb = ekf_drive.wheelsToTwist(vel);

        if (known_data_association)
        {
          ekf.knownCorrespondenceSLAM(meas, vb);
        }

        else
        {
          ekf.SLAM(meas, vb);
        }

        map_flag = false;
      }

      wheel_odom_flag = false;
    }


    /////////////////////////////////////////////////////////////////////////////


    // braodcast transform from map to odom
    // transform from map to robot
    Transform2D Tmr = ekf.getRobotState();

    // transform from odom to robot
    Vector2D vor(pose.x, pose.y);
    Transform2D Tor(vor, pose.theta);

    // transform from robot to odom
    Transform2D Tro = Tor.inv();

    // now we can get transform from map to odom
    Transform2D Tmo = Tmr *  Tro;
    TransformData2D pose_map_odom = Tmo.displacement();


    tf2::Quaternion q_mo;
    q_mo.setRPY(0, 0, pose_map_odom.theta);
    geometry_msgs::Quaternion quat_mo;
    quat_mo = tf2::toMsg(q_mo);


    // broadcast transform between map and odom
    geometry_msgs::TransformStamped tf_mo;
    tf_mo.header.stamp = ros::Time::now();
    tf_mo.header.frame_id = map_frame_id;
    tf_mo.child_frame_id = odom_frame_id;

    tf_mo.transform.translation.x = pose_map_odom.x;
    tf_mo.transform.translation.y = pose_map_odom.y;
    tf_mo.transform.translation.z = 0.0;
    tf_mo.transform.rotation = quat_mo;

    map_odom_broadcaster.sendTransform(tf_mo);

    /////////////////////////////////////////////////////////////////////////////

    // path from SLAM
    geometry_msgs::PoseStamped slam_pose;
    TransformData2D pose_map_robot = Tmr.displacement();

    tf2::Quaternion q_mr;
    q_mr.setRPY(0, 0, pose_map_robot.theta);
    geometry_msgs::Quaternion quat_mr;
    quat_mr = tf2::toMsg(q_mr);

    slam_pose.header.stamp = ros::Time::now();
    slam_pose.pose.position.x = pose_map_robot.x;
    slam_pose.pose.position.y = pose_map_robot.y;
    slam_pose.pose.orientation = quat_mr;

    ekf_path.header.stamp = ros::Time::now();
    ekf_path.header.frame_id = map_frame_id;
    ekf_path.poses.push_back(slam_pose);

    slam_path_pub.publish(ekf_path);

    /////////////////////////////////////////////////////////////////////////////

    // path from odom
    geometry_msgs::PoseStamped odom_pose;

    tf2::Quaternion q_or;
    q_or.setRPY(0, 0, pose.theta);
    geometry_msgs::Quaternion quat_or;
    quat_or = tf2::toMsg(q_or);

    odom_pose.header.stamp = ros::Time::now();
    odom_pose.pose.position.x = pose.x;
    odom_pose.pose.position.y = pose.y;
    odom_pose.pose.orientation = quat_or;

    odom_path.header.stamp = ros::Time::now();
    odom_path.header.frame_id = map_frame_id;
    odom_path.poses.push_back(odom_pose);

    odom_path_pub.publish(odom_path);

    /////////////////////////////////////////////////////////////////////////////

    // path from gazebo
    gazebo_path.header.stamp = ros::Time::now();
    gazebo_path.header.frame_id = map_frame_id;
    gazebo_path.poses.push_back(gazebo_robot_pose);

    gazebo_path_pub.publish(gazebo_path);

    /////////////////////////////////////////////////////////////////////////////


    // marker array of landmark estimates from the ekf filter
    std::vector<Vector2D> map;
    ekf.getMap(map);

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(map.size());

    for(unsigned int i = 0; i < map.size(); i++)
    {
      marker_array.markers[i].header.frame_id = marker_frame_id;
      marker_array.markers[i].header.stamp = ros::Time::now();
      marker_array.markers[i].lifetime = ros::Duration(1.0/ 5.0); // 1/5th sec
      marker_array.markers[i].ns = "marker";
      marker_array.markers[i].id = i;

      marker_array.markers[i].type = visualization_msgs::Marker::CYLINDER;
      marker_array.markers[i].action = visualization_msgs::Marker::ADD;

      marker_array.markers[i].pose.position.x = map.at(i).x;
      marker_array.markers[i].pose.position.y = map.at(i).y;
      marker_array.markers[i].pose.position.z = 0.15;

      marker_array.markers[i].pose.orientation.x = 0.0;
      marker_array.markers[i].pose.orientation.y = 0.0;
      marker_array.markers[i].pose.orientation.z = 0.0;
      marker_array.markers[i].pose.orientation.w = 1.0;

      marker_array.markers[i].scale.x = 2.0 * 0.05;
      marker_array.markers[i].scale.y = 2.0 * 0.05;
      marker_array.markers[i].scale.z = 0.1;

      marker_array.markers[i].color.r = 0.0f;
      marker_array.markers[i].color.g = 0.0f;
      marker_array.markers[i].color.b = 1.0f;
      marker_array.markers[i].color.a = 1.0f;
    }
    marker_pub.publish(marker_array);

    /////////////////////////////////////////////////////////////////////////////
    // ground truth robot heading
    tf2::Quaternion gazebo_robot_quat(gazebo_robot_pose.pose.orientation.x,
                               gazebo_robot_pose.pose.orientation.y,
                               gazebo_robot_pose.pose.orientation.z,
                               gazebo_robot_pose.pose.orientation.w);

    tf2::Matrix3x3 mat(gazebo_robot_quat);
    auto roll = 0.0, pitch = 0.0 , yaw = 0.0;
    mat.getRPY(roll, pitch, yaw);


    // odometry error
    odom_error_msg.x_error = gazebo_robot_pose.pose.position.x - pose.x;
    odom_error_msg.y_error = gazebo_robot_pose.pose.position.y - pose.y;

    odom_error_msg.theta_error = normalize_angle_PI(normalize_angle_PI(yaw) - \
                                                     normalize_angle_PI(pose.theta));

    // slam error
    TransformData2D Trd_mr = Tmr.displacement();

    slam_error_msg.x_error = gazebo_robot_pose.pose.position.x - Trd_mr.x;
    slam_error_msg.y_error = gazebo_robot_pose.pose.position.y - Trd_mr.y;

    slam_error_msg.theta_error = normalize_angle_PI(normalize_angle_PI(yaw) - \
                                                     normalize_angle_PI(Trd_mr.theta));

    odom_error_pub.publish(odom_error_msg);
    slam_error_pub.publish(slam_error_msg);
  }

  return 0;
}












// end file
