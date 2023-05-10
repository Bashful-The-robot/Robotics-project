// robp_phidgets
#include <robp_phidgets/error.h>
#include <robp_phidgets/motor.h>
#include <robp_phidgets/util.h>

// STL
#include <string>

namespace robp::phidgets {
Motor::Motor(int port, uint32_t attach_timeout_ms,
             std::function<void()> callback)
    : callback_(callback), port_(port) {
  create();
  assignEventHandlers();
  openWaitForAttachment(reinterpret_cast<PhidgetHandle>(motor_), port,
                        attach_timeout_ms);
}

Motor::~Motor() { closeAndDelete(reinterpret_cast<PhidgetHandle *>(&motor_)); }

void Motor::create() {
  PhidgetReturnCode ret = PhidgetDCMotor_create(&motor_);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to create motor on port " + std::to_string(port_), ret);
  }
}

void Motor::assignEventHandlers() {
  PhidgetReturnCode ret;

  ret = PhidgetDCMotor_setOnVelocityUpdateHandler(motor_,
                                                  velocityUpdateCallback, this);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError("Failed to set on velocity update handler on port " +
                           std::to_string(port_),
                       ret);
  }

  ret = Phidget_setOnAttachHandler(reinterpret_cast<PhidgetHandle>(motor_),
                                   attachCallback, this);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to set attach handler on port " + std::to_string(port_), ret);
  }

  ret = Phidget_setOnDetachHandler(reinterpret_cast<PhidgetHandle>(motor_),
                                   detachCallback, this);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to set detach handler on port " + std::to_string(port_), ret);
  }

  ret = Phidget_setOnErrorHandler(reinterpret_cast<PhidgetHandle>(motor_),
                                  errorCallback, this);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to set error handler on port " + std::to_string(port_), ret);
  }
}

double Motor::acceleration() const {
  double acc;
  PhidgetReturnCode ret = PhidgetDCMotor_getAcceleration(motor_, &acc);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to get acceleration for motor on port " + std::to_string(port_),
        ret);
  }
  return acc;
}

void Motor::setAcceleration(double acceleration) {
  PhidgetReturnCode ret = PhidgetDCMotor_setAcceleration(motor_, acceleration);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to set acceleration for motor on port " + std::to_string(port_),
        ret);
  }
  acceleration_ = acceleration;
}

double Motor::targetBrakingStrength() const {
  double braking;
  PhidgetReturnCode ret =
      PhidgetDCMotor_getTargetBrakingStrength(motor_, &braking);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to get target braking strength for motor on port " +
            std::to_string(port_),
        ret);
  }
  return braking;
}

void Motor::setTargetBrakingStrength(double braking) {
  PhidgetReturnCode ret =
      PhidgetDCMotor_setTargetBrakingStrength(motor_, braking);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to set target braking strength for motor on port " +
            std::to_string(port_),
        ret);
  }
  target_braking_strength_ = braking;
}

double Motor::brakingStrength() const {
  double braking;
  PhidgetReturnCode ret = PhidgetDCMotor_getBrakingStrength(motor_, &braking);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError("Failed to get braking strength for motor on port " +
                           std::to_string(port_),
                       ret);
  }
  return braking;
}

double Motor::currentLimit() const {
  double limit;
  PhidgetReturnCode ret = PhidgetDCMotor_getCurrentLimit(motor_, &limit);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError("Failed to get current limit for motor on port " +
                           std::to_string(port_),
                       ret);
  }
  return limit;
}

void Motor::setCurrentLimit(double limit) {
  PhidgetReturnCode ret = PhidgetDCMotor_setCurrentLimit(motor_, limit);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError("Failed to set current limit for motor on port " +
                           std::to_string(port_),
                       ret);
  }
  current_limit_ = limit;
}

double Motor::dataRate() const {
  double rate;
  PhidgetReturnCode ret = PhidgetDCMotor_getDataRate(motor_, &rate);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to get data rate for motor on port " + std::to_string(port_),
        ret);
  }
  return rate;
}

void Motor::setDataRate(double rate) {
  PhidgetReturnCode ret = PhidgetDCMotor_setDataRate(motor_, rate);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to set data rate for motor on port " + std::to_string(port_),
        ret);
  }
  data_rate_ = rate;
}

void Motor::setFailsafe(uint32_t time_ms) {
  PhidgetReturnCode ret = PhidgetDCMotor_enableFailsafe(motor_, time_ms);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to enable failsafe for motor on port " + std::to_string(port_),
        ret);
  }
  failsafe_time_ = time_ms;
}

void Motor::resetFailsafe() {
  PhidgetReturnCode ret = PhidgetDCMotor_resetFailsafe(motor_);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to reset failsafe for motor on port " + std::to_string(port_),
        ret);
  }
}

double Motor::targetVelocity() const {
  double vel;
  PhidgetReturnCode ret = PhidgetDCMotor_getTargetVelocity(motor_, &vel);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError("Failed to get target velocity for motor on port " +
                           std::to_string(port_),
                       ret);
  }
  return vel;
}

void Motor::setTargetVelocity(double velocity) {
  PhidgetReturnCode ret = PhidgetDCMotor_setTargetVelocity(motor_, velocity);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError("Failed to set target velocity for motor on port " +
                           std::to_string(port_),
                       ret);
  }
}

double Motor::velocity() const {
  double vel;
  PhidgetReturnCode ret = PhidgetDCMotor_getVelocity(motor_, &vel);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to get velocity for motor on port " + std::to_string(port_),
        ret);
  }
  return vel;
}

int Motor::port() const { return port_; }

void Motor::init() {
  if (0 <= acceleration_) {
    setAcceleration(acceleration_);
  }
  if (0 <= target_braking_strength_) {
    setTargetBrakingStrength(target_braking_strength_);
  }
  if (0 <= current_limit_) {
    setCurrentLimit(current_limit_);
  }
  if (0 <= data_rate_) {
    setDataRate(data_rate_);
  }
  if (0 < failsafe_time_) {
    setFailsafe(failsafe_time_);
  }
}

double Motor::velocityUpdate() const {
  std::lock_guard lk(m_);
  update_ = false;
  return last_velocity_;
}

bool Motor::hasUpdate() const { return update_; }

void Motor::velocityUpdateCallback(PhidgetDCMotorHandle ch, void *ctx,
                                   double velocity) {
  {
    std::lock_guard lk(static_cast<Motor *>(ctx)->m_);
    static_cast<Motor *>(ctx)->last_velocity_ = velocity;
    static_cast<Motor *>(ctx)->update_ = true;
  }
  static_cast<Motor *>(ctx)->callback_();
}

void Motor::attachCallback(PhidgetHandle ch, void *ctx) {
  printf("Attach motor on port %d\n", static_cast<Motor *>(ctx)->port());
  static_cast<Motor *>(ctx)->init();
}

void Motor::detachCallback(PhidgetHandle ch, void *ctx) {
  printf("Detach motor on port %d\n", static_cast<Motor *>(ctx)->port());
}

void Motor::errorCallback(PhidgetHandle ch, void *ctx,
                          Phidget_ErrorEventCode code,
                          char const *description) {
  fprintf(stderr, "\x1B[31mError motor on port %d: %s\033[0m\n",
          static_cast<Motor *>(ctx)->port(), description);
  fprintf(stderr, "----------\n");
  PhidgetLog_log(PHIDGET_LOG_ERROR, "Error %d: %s", code, description);
}
}  // namespace robp::phidgets