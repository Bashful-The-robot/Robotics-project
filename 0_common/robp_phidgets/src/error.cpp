// robp_phidgets
#include <robp_phidgets/error.h>

namespace robp::phidgets {
PhidgetError::PhidgetError(std::string const& msg, PhidgetReturnCode code)
    : std::exception() {
  const char* error_ptr;
  PhidgetReturnCode ret = Phidget_getErrorDescription(code, &error_ptr);
  if (ret == EPHIDGET_OK) {
    msg_ = msg + ": " + std::string(error_ptr);
  } else {
    msg_ = msg + ": Unknown error";
  }
}

char const* PhidgetError::what() const noexcept { return msg_.c_str(); }
}  // namespace robp::phidgets