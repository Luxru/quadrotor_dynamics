#include "flightlib/common/command.hpp"


namespace flightlib {

Command::Command(const Scalar t, const CommandType type)
  : t(t), type(type) {}

bool Command::valid() const {
  bool valid = false;
  switch (type) {
    case SINGLE_ROTOR_THRUSTS:
      valid = thrusts.allFinite();
      break;
    case RATES_THRUST:
      valid = omega.allFinite();
      break;
    case TORQUES_THRUST:
      valid = torques.allFinite();
      break;
    default:
      break;
  }
  return std::isfinite(t) && valid;
}

}  // namespace flightlib