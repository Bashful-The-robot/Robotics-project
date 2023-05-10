// robp_phidgets
#include <robp_phidgets/encoders_nodelet.h>

// ROS
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(robp::phidgets::EncodersNodelet, nodelet::Nodelet)

namespace robp::phidgets {
void EncodersNodelet::onInit() {
  NODELET_INFO("Initializing Phidgets Encoders nodelet");
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle nh_priv = getPrivateNodeHandle();
  encoders_ = std::make_unique<Encoders>(nh, nh_priv);
}
}  // namespace robp::phidgets
