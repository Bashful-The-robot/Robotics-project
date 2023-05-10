#ifndef ROBP_PHIDGETS_ENCODERS_H
#define ROBP_PHIDGETS_ENCODERS_H

// robp_phidgets
#include <robp_phidgets/EncoderConfig.h>
#include <robp_phidgets/encoder.h>

// ROS
#include <dynamic_reconfigure/server.h>

// STL
#include <memory>

namespace robp::phidgets {
class Encoders {
 public:
  Encoders(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

  ~Encoders();

 private:
  void publish();

  void configCallback(robp_phidgets::EncoderConfig& config, uint32_t level);

 private:
  ros::NodeHandle& nh_;

  ros::Publisher pub_;

  std::unique_ptr<Encoder> left_;
  std::unique_ptr<Encoder> right_;

  dynamic_reconfigure::Server<robp_phidgets::EncoderConfig> server_;
  dynamic_reconfigure::Server<robp_phidgets::EncoderConfig>::CallbackType f_;
};
}  // namespace robp::phidgets

#endif  // ROBP_PHIDGETS_ENCODERS_H