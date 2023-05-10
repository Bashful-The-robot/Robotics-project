#ifndef ROBP_PHIDGETS_ENCODER_NODELET_H
#define ROBP_PHIDGETS_ENCODER_NODELET_H

// robp_phidgets
#include <robp_phidgets/encoders.h>

// ROS
#include <nodelet/nodelet.h>

// STL
#include <memory>

namespace robp::phidgets {
class EncodersNodelet : public nodelet::Nodelet {
 public:
  ~EncodersNodelet() override = default;

 private:
  void onInit() override;

 private:
  std::unique_ptr<Encoders> encoders_;
};
}  // namespace robp::phidgets

#endif  // ROBP_PHIDGETS_ENCODER_NODELET_H