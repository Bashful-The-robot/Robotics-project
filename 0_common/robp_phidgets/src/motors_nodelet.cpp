// robp_phidgets
#include <robp_phidgets/motors_nodelet.h>

// ROS
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(robp::phidgets::MotorsNodelet, nodelet::Nodelet)

namespace robp::phidgets {
void MotorsNodelet::onInit() {
  NODELET_INFO("Initializing Phidgets Motors nodelet");
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle nh_priv = getPrivateNodeHandle();
  motors_ = std::make_unique<Motors>(nh, nh_priv);
}
}  // namespace robp::phidgets
