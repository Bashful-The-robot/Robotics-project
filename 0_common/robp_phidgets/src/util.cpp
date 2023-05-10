// robp_hidgets
#include <robp_phidgets/error.h>
#include <robp_phidgets/util.h>

namespace robp::phidgets {
void openWaitForAttachment(PhidgetHandle handle, int hub_port,
                           uint32_t timeout_ms) {
  PhidgetReturnCode ret;

  ret = Phidget_setHubPort(handle, hub_port);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to set device hub port " + std::to_string(hub_port), ret);
  }

  ret = Phidget_openWaitForAttachment(handle, timeout_ms);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError(
        "Failed to open device on hub port " + std::to_string(hub_port), ret);
  }
}

void closeAndDelete(PhidgetHandle* handle) {
  PhidgetReturnCode ret;

  ret = Phidget_close(*handle);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError("Failed to close device", ret);
  }

  ret = Phidget_delete(handle);
  if (EPHIDGET_OK != ret) {
    throw PhidgetError("Failed to delete device", ret);
  }
}
}  // namespace robp::phidgets