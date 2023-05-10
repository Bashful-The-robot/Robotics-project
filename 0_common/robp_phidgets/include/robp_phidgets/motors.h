#ifndef ROBP_PHIDGETS_MOTORS_H
#define ROBP_PHIDGETS_MOTORS_H

// robp_phidgets
#include <robp_phidgets/MotorConfig.h>
#include <robp_phidgets/motor.h>

// robp_msgs
#include <robp_msgs/DutyCycles.h>

// ROS
#include <dynamic_reconfigure/server.h>
#include <std_srvs/Empty.h>

// STL
#include <memory>

namespace robp::phidgets {
class Motors {
 public:
  Motors(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

  ~Motors();

 private:
  void dutyCyclesCallback(robp_msgs::DutyCycles::ConstPtr const& msg);

  void publish();

  bool resetFailsafe(std_srvs::Empty::Request& req,
                     std_srvs::Empty::Response& res);

  void configCallback(robp_phidgets::MotorConfig& config, uint32_t level);

 private:
  ros::Subscriber sub_;

  ros::Publisher pub_;

  ros::ServiceServer reset_failsafe_srv_;

  std::unique_ptr<Motor> left_;
  std::unique_ptr<Motor> right_;

  uint32_t failsafe_time_{};
  bool failsafe_enabled_{false};

  dynamic_reconfigure::Server<robp_phidgets::MotorConfig> server_;
  dynamic_reconfigure::Server<robp_phidgets::MotorConfig>::CallbackType f_;
};
}  // namespace robp::phidgets

#endif  // ROBP_PHIDGETS_MOTORS_H