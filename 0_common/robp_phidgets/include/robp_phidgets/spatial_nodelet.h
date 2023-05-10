#ifndef ROBP_PHIDGETS_SPATIAL_NODELET_H
#define ROBP_PHIDGETS_SPATIAL_NODELET_H

// robp_phidgets
#include <robp_phidgets/spatial.h>

// ROS
#include <nodelet/nodelet.h>

// STL
#include <memory>

namespace robp::phidgets {
class SpatialNodelet : public nodelet::Nodelet {
 public:
  ~SpatialNodelet() override = default;

 private:
  void onInit() override;

 private:
  std::unique_ptr<Spatial> spatial_;
};
}  // namespace robp::phidgets

#endif  // ROBP_PHIDGETS_SPATIAL_NODELET_H