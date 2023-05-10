#ifndef ROBP_PHIDGETS_TEMPERATURE_H
#define ROBP_PHIDGETS_TEMPERATURE_H

// robp_phidgets
#include <robp_phidgets/TemperatureConfig.h>

// Phidget
extern "C" {
#include <phidget22.h>
}

// ROS
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

// STL
#include <limits>

namespace robp::phidgets {
class Temperature {
 public:
  Temperature(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);

  ~Temperature();

  double dataRate() const;

  void setDataRate(double rate);

  double temperature() const;

  double temperatureChangeTrigger() const;

  void setTemperatureChangeTrigger(double change);

  int port() const;

 private:
  void create();

  void assignEventHandlers();

  void init();

  void publish(double temperature);

  static void CCONV temperatureChangeCallback(PhidgetTemperatureSensorHandle ch,
                                              void *ctx, double temperature);

  static void CCONV attachCallback(PhidgetHandle ch, void *ctx);

  static void CCONV detachCallback(PhidgetHandle ch, void *ctx);

  static void CCONV errorCallback(PhidgetHandle ch, void *ctx,
                                  Phidget_ErrorEventCode code,
                                  char const *description);

  void configCallback(robp_phidgets::TemperatureConfig &config, uint32_t level);

 private:
  ros::Publisher pub_;

  PhidgetTemperatureSensorHandle temperature_;

  dynamic_reconfigure::Server<robp_phidgets::TemperatureConfig> server_;
  dynamic_reconfigure::Server<robp_phidgets::TemperatureConfig>::CallbackType f_;

  int port_;
  std::string frame_id_;

  double data_rate_{-1};
  double temperature_change_trigger_{std::numeric_limits<double>::quiet_NaN()};
};
}  // namespace robp::phidgets

#endif  // ROBP_PHIDGETS_TEMPERATURE_H