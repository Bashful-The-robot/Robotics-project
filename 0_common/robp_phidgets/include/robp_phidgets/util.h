#ifndef ROBP_PHIDGETS_ERROR_HANDLING_H
#define ROBP_PHIDGETS_ERROR_HANDLING_H

// Phidget
extern "C" {
#include <phidget22.h>
}

namespace robp::phidgets {

void openWaitForAttachment(PhidgetHandle handle, int hub_port,
                           uint32_t timeout_ms);

void closeAndDelete(PhidgetHandle *handle);
}  // namespace robp::phidgets

#endif  // ROBP_PHIDGETS_ERROR_HANDLING_H