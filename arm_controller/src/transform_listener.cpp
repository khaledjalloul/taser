#include "arm_controller/transform_listener.hpp"

namespace arm_controller {

TransformListener::TransformListener(const rclcpp::Node::SharedPtr &node)
    : node_(node), buffer_(node->get_clock()), tfListener_(buffer_, node) {}

EigenTransform TransformListener::get_tf(std::string target_frame,
                                         std::string source_frame) {
  try {
    auto tf = buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    return {Vector(tf.transform.translation.x, tf.transform.translation.y,
                   tf.transform.translation.z),
            Quaternion(tf.transform.rotation.w, tf.transform.rotation.x,
                       tf.transform.rotation.y, tf.transform.rotation.z)};
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(node_->get_logger(), "Transform error: %s", ex.what());
    return {Vector(0, 0, 0), Quaternion(1, 0, 0, 0)};
  }
}

} // namespace arm_controller