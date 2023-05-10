// robp_phidgets
#include <robp_phidgets/error.h>
#include <robp_phidgets/temperature.h>
#include <robp_phidgets/util.h>

// ROS
#include <sensor_msgs/Temperature.h>

namespace robp::phidgets {
Temperature::Temperature(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
    : server_(nh_priv) {
  pub_ = nh_priv.advertise<sensor_msgs::Temperature>("temperature", 1);

  if (!nh_priv.getParam("port", port_)) {
    ROS_FATAL("Must specify 'port'");
    exit(1);
  }

  create();
  assignEventHandlers();
  openWaitForAttachment(reinterpret_cast<PhidgetHandle>(temperature_), port_,
                        nh_priv.param("timeout", PHIDGET_TIMEOUT_DEFAULT));

  // Dynamic reconfigure
  f_ = boost::bind(&Temperature::configCallback, this, _1, _2);
  server_.setCallback(f_);
}

Temperature::~Temperature() {
  closeAndDelete(reinterpret_cast<PhidgetHandle *>(&temperature_));
}

void Temperature::create() {
  PhidgetReturnCode ret = PhidgetTemperatureSensor_create(&temperature_);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to create temperature sensor on port " + std::to_string(port_),
        ret);
  }
}

void Temperature::assignEventHandlers() {
  PhidgetReturnCode ret;

  ret = PhidgetTemperatureSensor_setOnTemperatureChangeHandler(
      temperature_, temperatureChangeCallback, this);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError("Failed to set on temperature change handler on port " +
                           std::to_string(port_),
                       ret);
  }

  ret = Phidget_setOnAttachHandler(
      reinterpret_cast<PhidgetHandle>(temperature_), attachCallback, this);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to set attach handler on port " + std::to_string(port_), ret);
  }

  ret = Phidget_setOnDetachHandler(
      reinterpret_cast<PhidgetHandle>(temperature_), detachCallback, this);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to set detach handler on port " + std::to_string(port_), ret);
  }

  ret = Phidget_setOnErrorHandler(reinterpret_cast<PhidgetHandle>(temperature_),
                                  errorCallback, this);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to set error handler on port " + std::to_string(port_), ret);
  }
}

double Temperature::dataRate() const {
  double rate;
  PhidgetReturnCode ret =
      PhidgetTemperatureSensor_getDataRate(temperature_, &rate);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to get data rate for temperature sensor on port " +
            std::to_string(port_),
        ret);
  }
  return rate;
}

void Temperature::setDataRate(double rate) {
  PhidgetReturnCode ret =
      PhidgetTemperatureSensor_setDataRate(temperature_, rate);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to set data rate for temperature sensor on port " +
            std::to_string(port_),
        ret);
  }
  data_rate_ = rate;
}

double Temperature::temperature() const {
  double temp;
  PhidgetReturnCode ret =
      PhidgetTemperatureSensor_getTemperature(temperature_, &temp);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to get temperature for temperature sensor on port " +
            std::to_string(port_),
        ret);
  }
  return temp;
}

double Temperature::temperatureChangeTrigger() const {
  double change;
  PhidgetReturnCode ret = PhidgetTemperatureSensor_getTemperatureChangeTrigger(
      temperature_, &change);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to get temperature change trigger for temperature sensor on "
        "port " +
            std::to_string(port_),
        ret);
  }
  return change;
}

void Temperature::setTemperatureChangeTrigger(double change) {
  PhidgetReturnCode ret = PhidgetTemperatureSensor_setTemperatureChangeTrigger(
      temperature_, change);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to set temperature change trigger for temperature sensor on "
        "port " +
            std::to_string(port_),
        ret);
  }
  temperature_change_trigger_ = change;
}

int Temperature::port() const { return port_; }

void Temperature::init() {
  if (0 <= data_rate_) {
    setDataRate(data_rate_);
  }
  if (0 < temperature_change_trigger_) {
    setTemperatureChangeTrigger(temperature_change_trigger_);
  }
}

void Temperature::publish(double temperature) {
  sensor_msgs::Temperature::Ptr msg(new sensor_msgs::Temperature);
  msg->header.frame_id = frame_id_;
  msg->header.stamp = ros::Time::now();
  msg->temperature = temperature;
  msg->variance = 0.0;
  pub_.publish(msg);
}

void Temperature::temperatureChangeCallback(PhidgetTemperatureSensorHandle ch,
                                            void *ctx, double temperature) {
  static_cast<Temperature *>(ctx)->publish(temperature);
}

void Temperature::attachCallback(PhidgetHandle ch, void *ctx) {
  printf("Attach temperature sensor on port %d\n",
         static_cast<Temperature *>(ctx)->port());
  static_cast<Temperature *>(ctx)->init();
}

void Temperature::detachCallback(PhidgetHandle ch, void *ctx) {
  printf("Detach temperature sensor on port %d\n",
         static_cast<Temperature *>(ctx)->port());
}

void Temperature::errorCallback(PhidgetHandle ch, void *ctx,
                                Phidget_ErrorEventCode code,
                                char const *description) {
  fprintf(stderr, "\x1B[31mError temperature sensor on port %d: %s\033[0m\n",
          static_cast<Temperature *>(ctx)->port(), description);
  fprintf(stderr, "----------\n");
  PhidgetLog_log(PHIDGET_LOG_ERROR, "Error %d: %s", code, description);
}

void Temperature::configCallback(robp_phidgets::TemperatureConfig &config, uint32_t level) {
  frame_id_ = config.frame_id;
  setDataRate(config.data_rate);
  setTemperatureChangeTrigger(config.temperature_change_trigger);
}
}  // namespace robp::phidgets
