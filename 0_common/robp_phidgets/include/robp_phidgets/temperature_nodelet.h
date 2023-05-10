#ifndef ROBP_PHIDGETS_TEMPERATURE_NODELET_H
#define ROBP_PHIDGETS_TEMPERATURE_NODELET_H

// robp_phidgets
#include <robp_phidgets/temperature.h>

// ROS
#include <nodelet/nodelet.h>

// STL
#include <memory>

namespace robp::phidgets {
class TemperatureNodelet : public nodelet::Nodelet {
 public:
  ~TemperatureNodelet() override = default;

 private:
  void onInit() override;

 private:
  std::unique_ptr<Temperature> temperature_;
};
}  // namespace robp::phidgets

#endif  // ROBP_PHIDGETS_TEMPERATURE_NODELET_H