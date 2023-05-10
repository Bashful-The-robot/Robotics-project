#ifndef ROBP_PHIDGETS_SPATIAL_H
#define ROBP_PHIDGETS_SPATIAL_H

// robp_phidgets
#include <robp_phidgets/SpatialConfig.h>

// Phidget
extern "C" {
#include <phidget22.h>
}

// ROS
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>

// STL
#include <limits>
#include <string>

namespace robp::phidgets {
class Spatial {
 public:
  Spatial(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);

  ~Spatial();

  void calibrate();

  void setAHRSParameters(double angular_velocity_threshold,
                         double angular_velocity_delta_threshold,
                         double acceleration_threshold, double mag_time,
                         double accel_time, double bias_time);

  Phidget_SpatialAlgorithm algorithm() const;

  void setAlgorithm(Phidget_SpatialAlgorithm algorithm);

  double dataRate() const;

  void setDataRate(double rate);

  bool heatingEnabled() const;

  void setHeatingEnabled(bool enabled);

  void setMagnetometerCorrectionParameters(double magnetic_field,
                                           double offset0, double offset1,
                                           double offset2, double gain0,
                                           double gain1, double gain2,
                                           double T0, double T1, double T2,
                                           double T3, double T4, double T5);

  tf2::Quaternion quaternion() const;

  void resetMagnetometerCorrectionParameters();

  void saveMagnetometerCorrectionParameters();

  void zeroAlgorithm();

  void zeroGyro();

  int port() const;

 private:
  void create();

  void assignEventHandlers();

  void init();

  void publish();

  static void CCONV algorithmCallback(PhidgetSpatialHandle ch, void *ctx,
                                      double const quaternion[4],
                                      double timestamp);

  static void CCONV spatialCallback(PhidgetSpatialHandle ch, void *ctx,
                                    double const acceleration[3],
                                    double const angular_rate[3],
                                    double const magnetic_field[3],
                                    double timestamp);

  static void CCONV attachCallback(PhidgetHandle ch, void *ctx);

  static void CCONV detachCallback(PhidgetHandle ch, void *ctx);

  static void CCONV errorCallback(PhidgetHandle ch, void *ctx,
                                  Phidget_ErrorEventCode code,
                                  char const *description);

  bool calibrateCallback(std_srvs::Empty::Request &req,
                         std_srvs::Empty::Response &res);

  void configCallback(robp_phidgets::SpatialConfig &config, uint32_t level);

 private:
  ros::Publisher imu_pub_;
  ros::Publisher mag_pub_;

  ros::ServiceServer calibrate_srv_;

  dynamic_reconfigure::Server<robp_phidgets::SpatialConfig> server_;
  dynamic_reconfigure::Server<robp_phidgets::SpatialConfig>::CallbackType f_;

  PhidgetSpatialHandle spatial_{};

  int port_;

  std::string frame_id_{"imu_link"};

  // Time
  ros::Time ros_time_zero_;
  bool synchronize_timestamps_{true};
  uint64_t data_time_zero_ns_{0};
  uint64_t last_data_timestamp_ns_{0};
  uint64_t last_ros_stamp_ns_{0};
  int64_t time_resync_interval_ns_{0};
  int64_t data_interval_ns_{0};
  bool can_publish_{false};
  ros::Time last_cb_time_;
  int64_t cb_delta_epsilon_ns_{1000 * 1000};

  tf2::Quaternion orientation_;
  double angular_velocity_variance_;
  tf2::Vector3 angular_velocity_;
  double linear_acceleration_variance_;
  tf2::Vector3 linear_acceleration_;

  double magnetic_field_variance_;
  tf2::Vector3 magnetic_field_;

  bool initialized_{false};

  double data_rate_{-1};

  bool heating_enabled_{false};

  // AHRS parameters
  double angular_velocity_threshold_{};
  double angular_velocity_delta_threshold_{};
  double acceleration_threshold_{};
  double mag_time_{};
  double accel_time_{};
  double bias_time_{};

  // Algorithm
  Phidget_SpatialAlgorithm algorithm_;

  // Magnetometer correction parameters
  double cp_magnetic_field_{};
  double cp_offset0_{};
  double cp_offset1_{};
  double cp_offset2_{};
  double cp_gain0_{};
  double cp_gain1_{};
  double cp_gain2_{};
  double cp_T0_{};
  double cp_T1_{};
  double cp_T2_{};
  double cp_T3_{};
  double cp_T4_{};
  double cp_T5_{};
};

void CCONV accelerationChange(PhidgetAccelerometerHandle ch, void *ctx,
                              double const acceleration[3], double timestamp);
}  // namespace robp::phidgets

#endif  // ROBP_PHIDGETS_SPATIAL_H