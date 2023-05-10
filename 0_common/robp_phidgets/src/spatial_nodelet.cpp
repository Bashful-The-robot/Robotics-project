// robp_phidgets
#include <robp_phidgets/spatial_nodelet.h>

// ROS
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(robp::phidgets::SpatialNodelet, nodelet::Nodelet)

namespace robp::phidgets {
void SpatialNodelet::onInit() {
  NODELET_INFO("Initializing Phidgets Spatial nodelet");
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle nh_priv = getPrivateNodeHandle();
  spatial_ = std::make_unique<Spatial>(nh, nh_priv);
}
}  // namespace robp::phidgets
