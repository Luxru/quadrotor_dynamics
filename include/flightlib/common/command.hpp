
#pragma once

#include <cmath>
#include <cstdint>
#include <sys/types.h>

#include "flightlib/common/types.hpp"

namespace flightlib {

enum CommandType : uint8_t {
  SINGLE_ROTOR_THRUSTS = 0,
  RATES_THRUST = 1,
  TORQUES_THRUST = 2,
};

struct Command {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  Command() = default;
  Command(const Scalar t,const CommandType type);

  bool valid() const;

  CommandType type{SINGLE_ROTOR_THRUSTS};
  /// time in [s]
  Scalar t{NAN};

  /// Collective mass-normalized thrust in [m/s^2]
  Scalar collective_thrust{NAN};

  /// Bodyrates in [rad/s]
  Vector<3> omega{NAN, NAN, NAN};

  // Torques in [Nm]
  Vector<3> torques{NAN, NAN, NAN};  

  /// Single rotor thrusts in [N]
  Vector<4> thrusts{NAN, NAN, NAN, NAN};
};

}  // namespace flightlib