#ifndef ROBP_PHIDGETS_MOTORS_NODELET_H
#define ROBP_PHIDGETS_MOTORS_NODELET_H

// robp_phidgets
#include <robp_phidgets/motors.h>

// ROS
#include <nodelet/nodelet.h>

// STL
#include <memory>

namespace robp::phidgets {
class MotorsNodelet : public nodelet::Nodelet {
 public:
  ~MotorsNodelet() override = default;

 private:
  void onInit() override;

 private:
  std::unique_ptr<Motors> motors_;
};
}  // namespace robp::phidgets

#endif  // ROBP_PHIDGETS_MOTORS_NODELET_H