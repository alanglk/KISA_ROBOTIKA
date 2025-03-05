/// \file
/// \brief Detects and extracts circular features in laser scan
///
/// PARAMETERS:
///   frame_id - frame the circles are in
/// PUBLISHES:
///   landmarks (landmark_msgs::LandmarkMap): center and radius of all circles detected
/// SUBSCRIBES:
///   scan (sensor_msgs/LaserScan): Lidar scan

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
//#include <sensor_msgs/msg/point_cloud.hpp>
//#include <geometry_msgs/msg/point32.h>

#include <vector>
#include <string>
#include <iostream>
// ADDED by ELI (2024)
// Allows the use of M_PI and M_SQRT2 (pi and pi/2 values)
#define _USE_MATH_DEFINES
#include <cmath>
// END Eli

#include <rigid2d/rigid2d.hpp>
#include "landmark_msgs/msg/landmark_map.hpp"
#include "landmarks/landmarks.hpp"

using landmarks::LaserProperties;
using landmarks::Landmarks;

using rigid2d::Vector2D;
using rigid2d::deg2rad;

using std::placeholders::_1;

using namespace std::chrono_literals;

/// \brief Update the scan
/// \param msg -lidar scan

#define DEBUG 0
class LandmarkNode : public rclcpp::Node
{
  public:
    LandmarkNode() : Node("landmark_detection_node")
      {
        
        this->declare_parameter("frame_id", "laser"); //laser
        frame_id = this->get_parameter("frame_id").as_string();
        RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id.c_str());
        this->declare_parameter("laser_count", 1600);
        laser_count = this->get_parameter("laser_count").as_int();
        this->declare_parameter("beam_angle_min", -M_PI);
        beam_min = this->get_parameter("beam_angle_min").as_double();
        this->declare_parameter("beam_angle_max", M_PI);
        beam_max = this->get_parameter("beam_angle_max").as_double();
        this->declare_parameter("scan_angle_inc", deg2rad(1.0));
        scan_angle_inc = this->get_parameter("scan_angle_inc").as_double();
        this->declare_parameter("range_min", 0.12);
        range_min = this->get_parameter("range_min").as_double();
        this->declare_parameter("range_max", 6.0);
        range_max = this->get_parameter("range_max").as_double();
        scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 1, std::bind(&LandmarkNode::scanCallback, this, _1));
        circle_pub = this->create_publisher<landmark_msgs::msg::LandmarkMap>("landmarks", 1);
        timer_ = this->create_wall_timer(500ms, std::bind(&LandmarkNode::mapPublisher, this));
        
        // lidar properties
        
        beam_delta = scan_angle_inc;
        RCLCPP_INFO(this->get_logger(), "landmark detection node. frame_id: %s beam_min: %g beam_max: %g beam_delta: %g", frame_id.c_str(), beam_min, beam_max, beam_delta);
        RCLCPP_INFO(this->get_logger(), "scan angle increment: %g", scan_angle_inc);
        RCLCPP_INFO(this->get_logger(), "range min: %g range max: %g", range_min, range_max);
    
        // landmark classifier
        epsilon = 0.075;
        //props = LaserProperties():beam_min(0.0), beam_max(0.0), beam_Delta(0.0), range_min(0.0), range_max(0.0){};
        // LaserProperties props(beam_min, beam_max, beam_delta, range_min, range_max);
        // landmarks.setProps(props, epsilon);

      }
    //~LandmarkNode();
    
  private:
    rclcpp::Publisher<landmark_msgs::msg::LandmarkMap>::SharedPtr circle_pub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;  
    rclcpp::TimerBase::SharedPtr timer_;
    std::string frame_id;
    std::vector<float> scan;         // lidar scan
    bool scan_update = false;        // scan update flag
    
        // LaserProperties props(beam_min, beam_max, beam_delta, range_min, range_max);
    // Landmarks landmarks(props, epsilon);
    // lidar properties
    double scan_angle_inc;

    double beam_min;
    double beam_max;
    double beam_delta;
    double range_min;
    double range_max;
    int laser_count;
    // // // landmark classifier
    double epsilon = 0.075;
    //LaserProperties lprops;//(beam_min, beam_max, beam_delta, range_min, range_max);
    
    //Landmarks landmarks;//(lprops, epsilon);
    void scanCallback(sensor_msgs::msg::LaserScan::SharedPtr msg);
    void mapPublisher();

};

void LandmarkNode::scanCallback(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (DEBUG) {
    RCLCPP_INFO(this->get_logger(), "scan received");
  }
  // Just to allocate the vector, values will be overwritten
  scan = msg->ranges;
  // We need to shift the indices in a circular manner so that the readings start from the
  // back side and increment to the right
  for (int i = 0; i < msg->ranges.size(); i++)
  {
   int index = (i + msg->ranges.size() / 2) % msg->ranges.size();
  //  fprintf(stdout, "i: %d index: %d\n", i, index);
   scan[i] = msg->ranges[index];
  }
  
  scan_update = true;
}

void LandmarkNode::mapPublisher()
{
  if (DEBUG) {
    RCLCPP_INFO(this->get_logger(), "laser properties: beam_min=%.3f beam_max=%.3f beam_delta=%.3f range_min=%.3f range_max=%.3f", beam_min, beam_max, beam_delta, range_min, range_max);
  }
  LaserProperties props(beam_min, beam_max, beam_delta, range_min, range_max);
  Landmarks landmarks(props, epsilon);
  if (scan_update)
  {
      if (DEBUG) {
      RCLCPP_INFO(this->get_logger(), "scan updated");
    }
     // find features in new scan
    landmarks.featureDetection(scan);
    fprintf(stdout, "Map publisherNumber of landmarks detected: %d\n", landmarks.lm.size());

    // new map
    landmark_msgs::msg::LandmarkMap map;
    //auto map = landmarks::LandmarkMap();
    map.header.frame_id = frame_id;
    map.header.stamp = rclcpp::Clock().now();


    //std::cout << "Number of circles: " << landmarks.lm.size() << std::endl;

    for(unsigned int i = 0; i < landmarks.lm.size(); i++)
    {
      // Note: the radius refers to the cylinder's radius, not the radius of the triangle from the robot pose
      fprintf(stdout, "Landmark detection node.  landmark[%d]= (%.3f, %.3f)\n", i,landmarks.lm.at(i).x_hat, landmarks.lm.at(i).y_hat);
      map.cx.push_back(landmarks.lm.at(i).x_hat);
      map.cy.push_back(landmarks.lm.at(i).y_hat);
      map.r.push_back(landmarks.lm.at(i).radius);

    }
    circle_pub->publish(map);
    scan_update = false;

  }
}




int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto lmap = std::make_shared<LandmarkNode>();
  rclcpp::spin(lmap);
  rclcpp::shutdown();
  return 0;
}


// end file
