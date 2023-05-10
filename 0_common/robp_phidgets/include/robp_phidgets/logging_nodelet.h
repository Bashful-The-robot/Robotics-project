#ifndef ROBP_PHIDGETS_LOGGING_NODELET_H
#define ROBP_PHIDGETS_LOGGING_NODELET_H

// robp_phidgets
#include <robp_phidgets/LoggingConfig.h>

// ROS
#include <dynamic_reconfigure/server.h>
#include <nodelet/nodelet.h>

// STL
#include <memory>

namespace robp::phidgets {
class LoggingNodelet : public nodelet::Nodelet {
 public:
  ~LoggingNodelet() override = default;

 private:
  void onInit() override;

  void callback(robp_phidgets::LoggingConfig& config, uint32_t level);

 private:
  std::unique_ptr<dynamic_reconfigure::Server<robp_phidgets::LoggingConfig>>
      server_;
  dynamic_reconfigure::Server<robp_phidgets::LoggingConfig>::CallbackType f_;
};
}  // namespace robp::phidgets

#endif  // ROBP_PHIDGETS_LOGGING_NODELET_H