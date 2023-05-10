#ifndef ROBP_PHIDGETS_ERROR_H
#define ROBP_PHIDGETS_ERROR_H

// Phidget
extern "C" {
#include <phidget22.h>
}

// STL
#include <exception>
#include <string>

namespace robp::phidgets {
class PhidgetError : public std::exception {
 public:
  explicit PhidgetError(std::string const& msg, PhidgetReturnCode code);

  char const* what() const noexcept;

 private:
  std::string msg_;
};
}  // namespace robp::phidgets

#endif  // ROBP_PHIDGETS_ERROR_H