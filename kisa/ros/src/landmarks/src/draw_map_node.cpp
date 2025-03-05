/// \file
/// \brief Draw the landmarks usig a marker array
///
/// \author Boston Cleek
/// \date 2/27/20
///
/// PUBLISHES:
///   map (visualization_msgs::MarkerArray): landmarks represented as cylinders
/// SUBSCRIBES:
///   landmarks (nuslam::TurtleMap): center and radius of all circles detected


#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>
#include <string>
#include <iostream>

#include "landmark_msgs/msg/landmark_map.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

class DrawMapNode : public rclcpp::Node
{
  public:
    DrawMapNode() : Node("draw_map")
    {
      map_update = false;
      
      landmark_sub = this->create_subscription<landmark_msgs::msg::LandmarkMap>("landmarks", 1, std::bind(&DrawMapNode::mapCallback, this, _1));
      map_pub_timer = this->create_wall_timer(200ms, std::bind(&DrawMapNode::publish_markers, this));
      marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("map", 100);
    }
    private:
      bool map_update;               // map update flag
      std::vector<double> cx;        // circles x position
      std::vector<double> cy;        // circles y position
      std::vector<double> r;         // circles radius
      std::string frame_id;          // frame the circles are in


      void publish_markers();
      void mapCallback(landmark_msgs::msg::LandmarkMap::SharedPtr msg);
      rclcpp::Subscription<landmark_msgs::msg::LandmarkMap>::SharedPtr  landmark_sub;
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;

      rclcpp::TimerBase::SharedPtr map_pub_timer;
};

void DrawMapNode::publish_markers()
{
  if (map_update)
  {
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.resize(r.size());

    for(unsigned int i = 0; i < r.size(); i++)
    {
      markers.markers[i].header.frame_id = frame_id;
      markers.markers[i].header.stamp = rclcpp::Clock().now();
      markers.markers[i].lifetime = rclcpp::Duration::from_seconds(1.0/ 5.0); // 1/5th sec
      markers.markers[i].ns = "marker";
      markers.markers[i].id = i;

      markers.markers[i].type = visualization_msgs::msg::Marker::CYLINDER;
      markers.markers[i].action = visualization_msgs::msg::Marker::ADD;

      markers.markers[i].pose.position.x = cx.at(i);
      markers.markers[i].pose.position.y = cy.at(i);
      markers.markers[i].pose.position.z = 0.0;

      markers.markers[i].pose.orientation.x = 0.0;
      markers.markers[i].pose.orientation.y = 0.0;
      markers.markers[i].pose.orientation.z = 0.0;
      markers.markers[i].pose.orientation.w = 1.0;

      markers.markers[i].scale.x = 2.0 * r.at(i);
      markers.markers[i].scale.y = 2.0 * r.at(i);
      markers.markers[i].scale.z = 0.1;

      markers.markers[i].color.r = 0.0f;
      markers.markers[i].color.g = 1.0f;
      markers.markers[i].color.b = 0.0f;
      markers.markers[i].color.a = 1.0f;
    }
    marker_pub->publish(markers);

    map_update = false;
  }
}
void DrawMapNode::mapCallback(landmark_msgs::msg::LandmarkMap::SharedPtr msg)
{
  cx = msg->cx;
  cy = msg->cy;
  r = msg->r;
  frame_id = msg->header.frame_id;
  //  std::cout << "mapcallback. fame_id: "<< frame_id << std::endl;
  map_update = true;
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv); //, "draw_map");
  auto dmapnode = std::make_shared<DrawMapNode>();
  rclcpp::spin(dmapnode);
  rclcpp::shutdown();
  return 0; 
}

// end file
