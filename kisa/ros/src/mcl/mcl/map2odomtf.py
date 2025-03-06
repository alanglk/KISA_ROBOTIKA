import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_inverse
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time

class map2odom(Node):
    def __init__(self):
        super().__init__('map2odom_broadcaster_node')
        self.tf_broadcaster = TransformBroadcaster(self)

        # Assume these are set up elsewhere in your code:
        self.initial_pose_is_known = True #False
        self.global_frame_id = 'map'
        self.odom_frame_id = 'odom'
        # Example latest transform (normally would be provided by the AMCL logic)
        self.latest_tf = TransformStamped()
        self.latest_tf.transform.translation.x = 0.0
        self.latest_tf.transform.translation.y = 0.0
        self.latest_tf.transform.translation.z = 0.0
        self.latest_tf.transform.rotation.w = 1.0  # Identity quaternion
        self.get_logger().info("Starting map to odom broadcaster node")
        # Start a timer to periodically publish the transform (simulate a loop)
        self.timer = self.create_timer(1.0, self.broadcast_transform)

    def broadcast_transform(self):
        if not self.initial_pose_is_known:
            return
        
        # Set the current time as the timestamp
        expiration_time = self.get_clock().now().to_msg()
        
        # Prepare TransformStamped message
        tmp_tf_stamped = TransformStamped()
        tmp_tf_stamped.header.frame_id = self.global_frame_id
        tmp_tf_stamped.header.stamp = expiration_time
        tmp_tf_stamped.child_frame_id = self.odom_frame_id

        # Invert the latest transform and assign it
        inverted_transform = self.invert_transform(self.latest_tf)
        tmp_tf_stamped.transform = inverted_transform.transform

        # Publish the transform
        try:
            self.tf_broadcaster.sendTransform(tmp_tf_stamped)
            self.get_logger().info(f"Broadcasting map to odom transform: {tmp_tf_stamped}")
        except Exception as e:
            self.get_logger().error(f"Failed to broadcast transform: {e}")

    def invert_transform(self, transform):
        """Inverts the given TransformStamped transform."""
        inverted_transform = TransformStamped()
        inverted_transform.header = transform.header  # Keep same header
        inverted_transform.child_frame_id = transform.child_frame_id

        # Invert translation
        inverted_transform.transform.translation.x = -transform.transform.translation.x
        inverted_transform.transform.translation.y = -transform.transform.translation.y
        inverted_transform.transform.translation.z = -transform.transform.translation.z

        # Invert rotation using quaternion_inverse
        q = [transform.transform.rotation.x,
             transform.transform.rotation.y,
             transform.transform.rotation.z,
             transform.transform.rotation.w]
        q_inv = quaternion_inverse(q)
        inverted_transform.transform.rotation.x = q_inv[0]
        inverted_transform.transform.rotation.y = q_inv[1]
        inverted_transform.transform.rotation.z = q_inv[2]
        inverted_transform.transform.rotation.w = q_inv[3]

        return inverted_transform


def main(args=None):
    rclpy.init(args=args)
    node = map2odom()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# ''' Equivalent cpp code
# /*

# AmclNode::initTransforms()
# {
#   RCLCPP_INFO(get_logger(), "initTransforms");

#   // Initialize transform listener and broadcaster
#   tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
#   auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
#     get_node_base_interface(),
#     get_node_timers_interface(),
#     callback_group_);
#   tf_buffer_->setCreateTimerInterface(timer_interface);
#   tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
#   tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());

#   sent_first_transform_ = false;
#   latest_tf_valid_ = false;
#   latest_tf_ = tf2::Transform::getIdentity();
# }
# void
# AmclNode::sendMapToOdomTransform(const tf2::TimePoint & transform_expiration)
# {
#   // AMCL will update transform only when it has knowledge about robot's initial position
#   if (!initial_pose_is_known_) {return;}
#   geometry_msgs::msg::TransformStamped tmp_tf_stamped;
#   tmp_tf_stamped.header.frame_id = global_frame_id_;
#   tmp_tf_stamped.header.stamp = tf2_ros::toMsg(transform_expiration);
#   tmp_tf_stamped.child_frame_id = odom_frame_id_;
#   tf2::impl::Converter<false, true>::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);
#   tf_broadcaster_->sendTransform(tmp_tf_stamped);
# }

# AmclNode::calculateMaptoOdomTransform(
#   const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
#   const std::vector<amcl_hyp_t> & hyps, const int & max_weight_hyp)
# {
#   // subtracting base to odom from map to base and send map to odom instead
#   geometry_msgs::msg::PoseStamped odom_to_map;
#   try {
#     tf2::Quaternion q;
#     q.setRPY(0, 0, hyps[max_weight_hyp].pf_pose_mean.v[2]);
#     tf2::Transform tmp_tf(q, tf2::Vector3(
#         hyps[max_weight_hyp].pf_pose_mean.v[0],
#         hyps[max_weight_hyp].pf_pose_mean.v[1],
#         0.0));

#     geometry_msgs::msg::PoseStamped tmp_tf_stamped;
#     tmp_tf_stamped.header.frame_id = base_frame_id_;
#     tmp_tf_stamped.header.stamp = laser_scan->header.stamp;
#     tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);

#     tf_buffer_->transform(tmp_tf_stamped, odom_to_map, odom_frame_id_);
#   } catch (tf2::TransformException & e) {
#     RCLCPP_DEBUG(get_logger(), "Failed to subtract base to odom transform: (%s)", e.what());
#     return;
#   }

#   tf2::impl::Converter<true, false>::convert(odom_to_map.pose, latest_tf_);
#   latest_tf_valid_ = true;
# }



# */