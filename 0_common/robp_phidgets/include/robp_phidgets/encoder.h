#ifndef ROBP_PHIDGETS_ENCODER_H
#define ROBP_PHIDGETS_ENCODER_H

// Phidget
extern "C" {
#include <phidget22.h>
}

// STL
#include <functional>
#include <mutex>
#include <utility>

namespace robp::phidgets {
class Encoder {
 public:
  Encoder(int port, uint32_t attach_timeout_ms, std::function<void()> callback);

  ~Encoder();

  double dataRate() const;

  void setDataRate(double rate);

  int64_t indexPosition() const;

  int64_t position() const;

  void setPosition(int64_t position);

  uint32_t positionChangeTrigger() const;

  void setPositionChangeTrigger(uint32_t trigger);

  int port() const;

  std::pair<int, double> change() const;

  bool hasChange() const;

 private:
  void create();

  void assignEventHandlers();

  void init();

  static void CCONV positionChangeCallback(PhidgetEncoderHandle ch, void *ctx,
                                           int position_change,
                                           double time_change,
                                           int index_triggered);

  static void CCONV attachCallback(PhidgetHandle ch, void *ctx);

  static void CCONV detachCallback(PhidgetHandle ch, void *ctx);

  static void CCONV errorCallback(PhidgetHandle ch, void *ctx,
                                  Phidget_ErrorEventCode code,
                                  char const *description);

 private:
  PhidgetEncoderHandle encoder_;

  std::function<void()> callback_;

  int port_;

  double data_rate_{-1};
  int position_change_trigger_{-1};

  mutable std::mutex m_;
  mutable int position_change_{};
  mutable double time_change_{};
};
}  // namespace robp::phidgets

#endif  // ROBP_PHIDGETS_ENCODER_H