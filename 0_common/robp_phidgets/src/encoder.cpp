// robp_phidgets
#include <robp_phidgets/encoder.h>
#include <robp_phidgets/error.h>
#include <robp_phidgets/util.h>

// ROS
#include <ros/console.h>

namespace robp::phidgets {
Encoder::Encoder(int port, uint32_t attach_timeout_ms,
                 std::function<void()> callback)
    : callback_(callback), port_(port) {
  create();
  assignEventHandlers();
  openWaitForAttachment(reinterpret_cast<PhidgetHandle>(encoder_), port,
                        attach_timeout_ms);
}

Encoder::~Encoder() {
  closeAndDelete(reinterpret_cast<PhidgetHandle *>(&encoder_));
}

void Encoder::create() {
  PhidgetReturnCode ret = PhidgetEncoder_create(&encoder_);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to create encoder on port " + std::to_string(port_), ret);
  }
}

void Encoder::assignEventHandlers() {
  PhidgetReturnCode ret;

  ret = PhidgetEncoder_setOnPositionChangeHandler(encoder_,
                                                  positionChangeCallback, this);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError("Failed to set on position change handler on port " +
                           std::to_string(port_),
                       ret);
  }

  ret = Phidget_setOnAttachHandler(reinterpret_cast<PhidgetHandle>(encoder_),
                                   attachCallback, this);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to set attach handler on port " + std::to_string(port_), ret);
  }

  ret = Phidget_setOnDetachHandler(reinterpret_cast<PhidgetHandle>(encoder_),
                                   detachCallback, this);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to set detach handler on port " + std::to_string(port_), ret);
  }

  ret = Phidget_setOnErrorHandler(reinterpret_cast<PhidgetHandle>(encoder_),
                                  errorCallback, this);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to set error handler on port " + std::to_string(port_), ret);
  }
}

double Encoder::dataRate() const {
  double rate;
  PhidgetReturnCode ret = PhidgetEncoder_getDataRate(encoder_, &rate);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to get data rate for encoder on port " + std::to_string(port_),
        ret);
  }
  return rate;
}

void Encoder::setDataRate(double rate) {
  PhidgetReturnCode ret = PhidgetEncoder_setDataRate(encoder_, rate);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to set data rate for encoder on port " + std::to_string(port_),
        ret);
  }
  data_rate_ = rate;
}

int64_t Encoder::indexPosition() const {
  int64_t idx;
  PhidgetReturnCode ret = PhidgetEncoder_getIndexPosition(encoder_, &idx);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError("Failed to get index position for encoder on port " +
                           std::to_string(port_),
                       ret);
  }
  return idx;
}

int64_t Encoder::position() const {
  int64_t pos;
  PhidgetReturnCode ret = PhidgetEncoder_getPosition(encoder_, &pos);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to get position for encoder on port " + std::to_string(port_),
        ret);
  }
  return pos;
}

void Encoder::setPosition(int64_t position) {
  PhidgetReturnCode ret = PhidgetEncoder_setPosition(encoder_, position);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to set position for encoder on port " + std::to_string(port_),
        ret);
  }
}

uint32_t Encoder::positionChangeTrigger() const {
  uint32_t change;
  PhidgetReturnCode ret =
      PhidgetEncoder_getPositionChangeTrigger(encoder_, &change);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to get position change trigger for encoder on port " +
            std::to_string(port_),
        ret);
  }
  return change;
}

void Encoder::setPositionChangeTrigger(uint32_t trigger) {
  PhidgetReturnCode ret =
      PhidgetEncoder_setPositionChangeTrigger(encoder_, trigger);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to set position change trigger for encoder on port " +
            std::to_string(port_),
        ret);
  }
  position_change_trigger_ = trigger;
}

int Encoder::port() const { return port_; }

std::pair<int, double> Encoder::change() const {
  std::lock_guard lk(m_);
  auto p = position_change_;
  auto t = time_change_;
  position_change_ = 0;
  time_change_ = 0;
  return {p, t};
}

bool Encoder::hasChange() const { return 0 != time_change_; }

void Encoder::init() {
  if (0 <= data_rate_) {
    setDataRate(data_rate_);
  }
  if (0 <= position_change_trigger_) {
    setPositionChangeTrigger(position_change_trigger_);
  }
}

void Encoder::positionChangeCallback(PhidgetEncoderHandle ch, void *ctx,
                                     int position_change, double time_change,
                                     int index_triggered) {
  {
    std::lock_guard lk(static_cast<Encoder *>(ctx)->m_);
    static_cast<Encoder *>(ctx)->position_change_ += position_change;
    static_cast<Encoder *>(ctx)->time_change_ += time_change;
  }
  static_cast<Encoder *>(ctx)->callback_();
}

void Encoder::attachCallback(PhidgetHandle ch, void *ctx) {
  printf("Attach encoder on port %d\n", static_cast<Encoder *>(ctx)->port());
  static_cast<Encoder *>(ctx)->init();
}

void Encoder::detachCallback(PhidgetHandle ch, void *ctx) {
  printf("Detach encoder on port %d\n", static_cast<Encoder *>(ctx)->port());
}

void Encoder::errorCallback(PhidgetHandle ch, void *ctx,
                            Phidget_ErrorEventCode code,
                            char const *description) {
  fprintf(stderr, "\x1B[31mError encoder on port %d: %s\033[0m\n",
          static_cast<Encoder *>(ctx)->port(), description);
  fprintf(stderr, "----------\n");
  PhidgetLog_log(PHIDGET_LOG_ERROR, "Error %d: %s", code, description);
}
}  // namespace robp::phidgets