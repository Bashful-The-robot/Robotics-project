// robp_phidgets
#include <robp_phidgets/error.h>
#include <robp_phidgets/spatial.h>
#include <robp_phidgets/util.h>

// ROS
#include <angles/angles.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// STL

namespace robp::phidgets {
Spatial::Spatial(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
    : server_(nh_priv) {
  imu_pub_ = nh_priv.advertise<sensor_msgs::Imu>("data_raw", 1);
  mag_pub_ = nh_priv.advertise<sensor_msgs::MagneticField>("mag", 1);

  if (!nh_priv.getParam("port", port_)) {
    ROS_FATAL("Must specify 'port'");
    exit(1);
  }

  create();
  assignEventHandlers();
  openWaitForAttachment(reinterpret_cast<PhidgetHandle>(spatial_), port_,
                        nh_priv.param("timeout", PHIDGET_TIMEOUT_DEFAULT));

  calibrate_srv_ =
      nh_priv.advertiseService("calibrate", &Spatial::calibrateCallback, this);

  calibrate();

  // Dynamic reconfigure
  f_ = boost::bind(&Spatial::configCallback, this, _1, _2);
  server_.setCallback(f_);
}

Spatial::~Spatial() {
  closeAndDelete(reinterpret_cast<PhidgetHandle *>(&spatial_));
}

void Spatial::create() {
  PhidgetReturnCode ret = PhidgetSpatial_create(&spatial_);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to create temperature sensor on port " + std::to_string(port_),
        ret);
  }
}

void Spatial::assignEventHandlers() {
  PhidgetReturnCode ret;

  ret = PhidgetSpatial_setOnAlgorithmDataHandler(spatial_, algorithmCallback,
                                                 this);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError("Failed to set on algorithm data handler on port " +
                           std::to_string(port_),
                       ret);
  }

  ret = PhidgetSpatial_setOnSpatialDataHandler(spatial_, spatialCallback, this);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError("Failed to set on spatial data handler on port " +
                           std::to_string(port_),
                       ret);
  }

  ret = Phidget_setOnAttachHandler(reinterpret_cast<PhidgetHandle>(spatial_),
                                   attachCallback, this);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to set attach handler on port " + std::to_string(port_), ret);
  }

  ret = Phidget_setOnDetachHandler(reinterpret_cast<PhidgetHandle>(spatial_),
                                   detachCallback, this);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to set detach handler on port " + std::to_string(port_), ret);
  }

  ret = Phidget_setOnErrorHandler(reinterpret_cast<PhidgetHandle>(spatial_),
                                  errorCallback, this);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to set error handler on port " + std::to_string(port_), ret);
  }
}

void Spatial::calibrate() {
  ROS_INFO(
      "Calibrating IMU, this takes around 2 seconds to finish. Make sure that "
      "the device "
      "is not moved during this time.");
  zeroGyro();
  ros::Duration(2.0).sleep();
  ROS_INFO("Calibrating IMU done.");
}

void Spatial::setAHRSParameters(double angular_velocity_threshold,
                                double angular_velocity_delta_threshold,
                                double acceleration_threshold, double mag_time,
                                double accel_time, double bias_time) {
  PhidgetReturnCode ret = PhidgetSpatial_setAHRSParameters(
      spatial_, angular_velocity_threshold, angular_velocity_delta_threshold,
      acceleration_threshold, mag_time, accel_time, bias_time);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError("Failed to set AHRS parameters for spatial on port " +
                           std::to_string(port_),
                       ret);
  }
  angular_velocity_threshold_ = angular_velocity_threshold;
  angular_velocity_delta_threshold_ = angular_velocity_delta_threshold;
  acceleration_threshold_ = acceleration_threshold;
  mag_time_ = mag_time;
  accel_time_ = accel_time;
  bias_time_ = bias_time;
}

Phidget_SpatialAlgorithm Spatial::algorithm() const {
  Phidget_SpatialAlgorithm alg;
  PhidgetReturnCode ret = PhidgetSpatial_getAlgorithm(spatial_, &alg);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to get algorithm for spatial on port " + std::to_string(port_),
        ret);
  }
  return alg;
}

void Spatial::setAlgorithm(Phidget_SpatialAlgorithm algorithm) {
  PhidgetReturnCode ret = PhidgetSpatial_setAlgorithm(spatial_, algorithm);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to set algorithm for spatial on port " + std::to_string(port_),
        ret);
  }
  algorithm_ = algorithm;
}

double Spatial::dataRate() const {
  double rate;
  PhidgetReturnCode ret = PhidgetSpatial_getDataRate(spatial_, &rate);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to get data rate for spatial on port " + std::to_string(port_),
        ret);
  }
  return rate;
}

void Spatial::setDataRate(double rate) {
  PhidgetReturnCode ret = PhidgetSpatial_setDataRate(spatial_, rate);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to set data rate for spatial on port " + std::to_string(port_),
        ret);
  }
  data_rate_ = rate;
  data_interval_ns_ = 1000.0 * 1000.0 * 1.0 / rate;
}

bool Spatial::heatingEnabled() const {
  int enabled;
  PhidgetReturnCode ret = PhidgetSpatial_getHeatingEnabled(spatial_, &enabled);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError("Failed to get heating enabled for spatial on port " +
                           std::to_string(port_),
                       ret);
  }
  return enabled;
}

void Spatial::setHeatingEnabled(bool enabled) {
  PhidgetReturnCode ret = PhidgetSpatial_setDataRate(spatial_, enabled);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError("Failed to set heating enabled for spatial on port " +
                           std::to_string(port_),
                       ret);
  }
  heating_enabled_ = enabled;
}

void Spatial::setMagnetometerCorrectionParameters(
    double magnetic_field, double offset0, double offset1, double offset2,
    double gain0, double gain1, double gain2, double T0, double T1, double T2,
    double T3, double T4, double T5) {
  PhidgetReturnCode ret = PhidgetSpatial_setMagnetometerCorrectionParameters(
      spatial_, magnetic_field, offset0, offset1, offset2, gain0, gain1, gain2,
      T0, T1, T2, T3, T4, T5);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to set magnetometer correction parameters for spatial on "
        "port " +
            std::to_string(port_),
        ret);
  }
  cp_magnetic_field_ = magnetic_field;
  cp_offset0_ = offset0;
  cp_offset1_ = offset1;
  cp_offset2_ = offset2;
  cp_gain0_ = gain0;
  cp_gain1_ = gain1;
  cp_gain2_ = gain2;
  cp_T0_ = T0;
  cp_T1_ = T1;
  cp_T2_ = T2;
  cp_T3_ = T3;
  cp_T4_ = T4;
  cp_T5_ = T5;
}

tf2::Quaternion Spatial::quaternion() const {
  PhidgetSpatial_SpatialQuaternion quat;
  PhidgetReturnCode ret = PhidgetSpatial_getQuaternion(spatial_, &quat);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to get quaternion for spatial on port " + std::to_string(port_),
        ret);
  }
  return {quat.x, quat.y, quat.z, quat.w};
}

void Spatial::resetMagnetometerCorrectionParameters() {
  PhidgetReturnCode ret =
      PhidgetSpatial_resetMagnetometerCorrectionParameters(spatial_);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to reset magnetometer correction parameters for spatial on "
        "port " +
            std::to_string(port_),
        ret);
  }
}

void Spatial::saveMagnetometerCorrectionParameters() {
  PhidgetReturnCode ret =
      PhidgetSpatial_saveMagnetometerCorrectionParameters(spatial_);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to save magnetometer correction parameters for spatial on "
        "port " +
            std::to_string(port_),
        ret);
  }
}

void Spatial::zeroAlgorithm() {
  PhidgetReturnCode ret = PhidgetSpatial_zeroAlgorithm(spatial_);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to zero algorithm for spatial on port " + std::to_string(port_),
        ret);
  }
}

void Spatial::zeroGyro() {
  PhidgetReturnCode ret = PhidgetSpatial_zeroGyro(spatial_);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to zero gyro for spatial on port " + std::to_string(port_),
        ret);
  }
}

int Spatial::port() const { return port_; }

void Spatial::init() {
  orientation_ = tf2::Quaternion(0, 0, 0, 1);
  angular_velocity_ = tf2::Vector3();
  linear_acceleration_ = tf2::Vector3();
  magnetic_field_ = tf2::Vector3();

  if (!initialized_) {
    return;
  }

  if (0 <= data_rate_) {
    setDataRate(data_rate_);
  }
  if (heatingEnabled() != heating_enabled_) {
    setHeatingEnabled(heating_enabled_);
  }
  setAHRSParameters(angular_velocity_threshold_,
                    angular_velocity_delta_threshold_, acceleration_threshold_,
                    mag_time_, accel_time_, bias_time_);
  setAlgorithm(algorithm_);
  setMagnetometerCorrectionParameters(
      cp_magnetic_field_, cp_offset0_, cp_offset1_, cp_offset2_, cp_gain0_,
      cp_gain1_, cp_gain2_, cp_T0_, cp_T1_, cp_T2_, cp_T3_, cp_T4_, cp_T5_);
}

void Spatial::publish() {
  uint64_t imu_diff_in_ns = last_data_timestamp_ns_ - data_time_zero_ns_;
  uint64_t time_in_ns = ros_time_zero_.toNSec() + imu_diff_in_ns;

  if (time_in_ns < last_ros_stamp_ns_) {
    ROS_WARN("Time went backwards (%lu < %lu)! Not publishing message.",
             time_in_ns, last_ros_stamp_ns_);
    return;
  }
  last_ros_stamp_ns_ = time_in_ns;
  ros::Time ros_time = ros::Time().fromNSec(time_in_ns);

  sensor_msgs::Imu::Ptr imu_msg(new sensor_msgs::Imu);
  imu_msg->header.frame_id = frame_id_;
  imu_msg->header.stamp = ros_time;

  sensor_msgs::MagneticField::Ptr mag_msg(new sensor_msgs::MagneticField);
  mag_msg->header = imu_msg->header;

  // build covariance matrices
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (i == j) {
        int idx = j * 3 + i;
        imu_msg->angular_velocity_covariance[idx] = angular_velocity_variance_;
        imu_msg->linear_acceleration_covariance[idx] =
            linear_acceleration_variance_;
        mag_msg->magnetic_field_covariance[idx] = magnetic_field_variance_;
      }
    }
  }

  imu_msg->orientation = tf2::toMsg(orientation_);
  // TODO: imu_msg->orientation_covariance = ...;
  imu_msg->angular_velocity = tf2::toMsg(angular_velocity_);
  imu_msg->linear_acceleration = tf2::toMsg(linear_acceleration_);

  mag_msg->magnetic_field = tf2::toMsg(magnetic_field_);

  imu_pub_.publish(imu_msg);
  mag_pub_.publish(mag_msg);
}

void Spatial::algorithmCallback(PhidgetSpatialHandle ch, void *ctx,
                                double const q[4], double timestamp) {
  Spatial *s = static_cast<Spatial *>(ctx);
  s->orientation_ = tf2::Quaternion(q[0], q[1], q[2], q[3]);
}

void Spatial::spatialCallback(PhidgetSpatialHandle ch, void *ctx,
                              double const acc[3], double const ar[3],
                              double const mag[3], double timestamp) {
  Spatial *s = static_cast<Spatial *>(ctx);

  ros::Time now = ros::Time::now();

  if (0 == s->last_cb_time_.sec && 0 == s->last_cb_time_.nsec) {
    s->last_cb_time_ = now;
    return;
  }

  ros::Duration time_since_last_cb = now - s->last_cb_time_;
  uint64_t this_ts_ns = static_cast<uint64_t>(timestamp * 1000.0 * 1000.0);

  if (s->synchronize_timestamps_) {
    // if (time_since_last_cb.toNSec() >= (s->data_interval_ns_ -
    // s->cb_delta_epsilon_ns_) &&
    //     time_since_last_cb.toNSec() <= (s->data_interval_ns_ +
    //     s->cb_delta_epsilon_ns_)) {
    s->ros_time_zero_ = now;
    s->data_time_zero_ns_ = this_ts_ns;
    s->synchronize_timestamps_ = false;
    s->can_publish_ = true;
    // } else {
    // 	ROS_DEBUG(
    // 	    "Data not within acceptable window for synchronization: "
    // 	    "expected between %ld and %ld, saw %ld",
    // 	    s->data_interval_ns_ - s->cb_delta_epsilon_ns_,
    // 	    s->data_interval_ns_ + s->cb_delta_epsilon_ns_,
    // time_since_last_cb.toNSec());
    // }
  }

  if (s->can_publish_) {
    constexpr double g = 9.80665;

    s->angular_velocity_ =
        tf2::Vector3(angles::from_degrees(ar[0]), angles::from_degrees(ar[1]),
                     angles::from_degrees(ar[2]));
    s->linear_acceleration_ = tf2::Vector3(acc[0], acc[1], acc[2]) * -g;
    if (PUNK_DBL != mag[0]) {
      s->magnetic_field_ = tf2::Vector3(mag[0], mag[1], mag[2]) * 1e-4;
    } else {
      constexpr double nan = std::numeric_limits<double>::quiet_NaN();
      s->magnetic_field_ = tf2::Vector3(nan, nan, nan);
    }

    s->last_data_timestamp_ns_ = this_ts_ns;

    s->publish();
  }

  ros::Duration diff = now - s->ros_time_zero_;
  if (0 < s->time_resync_interval_ns_ &&
      diff.toNSec() >= s->time_resync_interval_ns_) {
    s->synchronize_timestamps_ = true;
  }

  s->last_cb_time_ = now;
}

void Spatial::attachCallback(PhidgetHandle ch, void *ctx) {
  printf("Attach spatial on port %d\n", static_cast<Spatial *>(ctx)->port());
  static_cast<Spatial *>(ctx)->init();
  static_cast<Spatial *>(ctx)->synchronize_timestamps_ = true;
  static_cast<Spatial *>(ctx)->can_publish_ = false;
  static_cast<Spatial *>(ctx)->last_cb_time_ = ros::Time(0);
}

void Spatial::detachCallback(PhidgetHandle ch, void *ctx) {
  printf("Detach spatial on port %d\n", static_cast<Spatial *>(ctx)->port());
}

void Spatial::errorCallback(PhidgetHandle ch, void *ctx,
                            Phidget_ErrorEventCode code,
                            char const *description) {
  fprintf(stderr, "\x1B[31mError spatial on port %d: %s\033[0m\n",
          static_cast<Spatial *>(ctx)->port(), description);
  fprintf(stderr, "----------\n");
  PhidgetLog_log(PHIDGET_LOG_ERROR, "Error %d: %s", code, description);
}

bool Spatial::calibrateCallback(std_srvs::Empty::Request &,
                                std_srvs::Empty::Response &) {
  calibrate();
  return true;
}

void Spatial::configCallback(robp_phidgets::SpatialConfig &config, uint32_t level) {
  frame_id_ = config.frame_id;

  setDataRate(config.data_rate);
  setAlgorithm(0 == config.algorithm
                   ? SPATIAL_ALGORITHM_NONE
                   : (1 == config.algorithm ? SPATIAL_ALGORITHM_AHRS
                                            : SPATIAL_ALGORITHM_IMU));
  setAHRSParameters(config.ahrs_angular_velocity_threshold,
                    config.ahrs_angular_velocity_delta_threshold,
                    config.ahrs_acceleration_threshold, config.ahrs_mag_time,
                    config.ahrs_accel_time, config.ahrs_bias_time);
  if (heatingEnabled() != config.heating_enabled) {
    setHeatingEnabled(config.heating_enabled);
  }
  setMagnetometerCorrectionParameters(
      config.cc_mag_field, config.cc_offset0, config.cc_offset1,
      config.cc_offset2, config.cc_gain0, config.cc_gain1, config.cc_gain2,
      config.cc_t0, config.cc_t1, config.cc_t2, config.cc_t3, config.cc_t4,
      config.cc_t5);

  initialized_ = true;
}
}  // namespace robp::phidgets