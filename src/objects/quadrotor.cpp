#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include <iostream>

namespace flightlib {

Quadrotor::Quadrotor(const std::string &cfg_path)
  : world_box_((Matrix<3, 2>() << -100, 100, -100, 100, -100, 100).finished()),
    size_(1.0, 1.0, 1.0),
    collision_(false) {
  //
  YAML::Node cfg = YAML::LoadFile(cfg_path);

  // create quadrotor dynamics and update the parameters
  dynamics_.updateParams(cfg);
  init();
}

Quadrotor::Quadrotor(const QuadrotorDynamics &dynamics)
  : world_box_((Matrix<3, 2>() << -100, 100, -100, 100, -100, 100).finished()),
    dynamics_(dynamics),
    size_(1.0, 1.0, 1.0),
    collision_(false) {
  init();
}

Quadrotor::~Quadrotor() {}

bool Quadrotor::run(const Command &cmd, const Scalar ctl_dt) {
  //check if the command is valid
  assert(cmd.valid());
  cmd_ = cmd;
  switch (cmd_.type) {
    case CommandType::SINGLE_ROTOR_THRUSTS:
      cmd_.thrusts = dynamics_.clampThrust(cmd_.thrusts);
      break;
    case CommandType::RATES_THRUST:
      cmd_.collective_thrust = dynamics_.clampThrust(cmd_.collective_thrust);
      cmd_.omega = dynamics_.clampBodyrates(cmd_.omega);
      break;
    case CommandType::TORQUES_THRUST:
    // TODO: check if the torques are valid
      cmd_.collective_thrust = dynamics_.clampThrust(cmd_.collective_thrust);
      cmd_.torques = dynamics_.clampTorques(cmd_.torques);
      break;
    default:
      std::cout << "[Quadrotor] Unknown command type!" << std::endl;
      return false;
  }
  return run(ctl_dt);
}

bool Quadrotor::run(const Scalar ctl_dt){
  //check if the state is valid
  if (!state_.valid()) return false;
  if (!cmd_.valid()) return false;

  // save the old state
  QuadState old_state = state_;
  QuadState next_state = state_;

  // time, max integration time step is max_dt
  const Scalar max_dt = integrator_ptr_->dtMax();
  Scalar remain_ctl_dt = ctl_dt;

  // simulation loop
  while (remain_ctl_dt > 0.0) {
    const Scalar sim_dt = std::min(remain_ctl_dt, max_dt);

    const Vector<4> motor_thrusts_des = runFlightCtl(sim_dt, state_, cmd_);

    runMotors(sim_dt, motor_thrusts_des);
    // motor_thrusts_ = cmd_.thrusts;

    const Vector<4> force_torques = B_allocation_ * motor_thrusts_;

    // Compute linear acceleration and body torque
    const Vector<3> force(0.0, 0.0, force_torques[0]);
    state_.a = state_.q() * force * 1.0 / dynamics_.getMass() + gz_;

    // compute body torque
    state_.tau = force_torques.segment<3>(1);

    // dynamics integration
    integrator_ptr_->step(state_.x, sim_dt, next_state.x);

    // update state and sim time
    state_.qx /= state_.qx.norm();

    //
    state_.x = next_state.x;
    remain_ctl_dt -= sim_dt;
  }
  state_.t += ctl_dt;
  //
  constrainInWorldBox(old_state);
  return true;
}

void Quadrotor::init(void) {
  // reset
  updateDynamics(dynamics_);
  reset();
}

bool Quadrotor::reset(void) {
  state_.setZero();
  motor_omega_.setZero();
  motor_thrusts_.setZero();
  return true;
}

bool Quadrotor::reset(const QuadState &state) {
  if (!state.valid()) return false;
  state_ = state;
  motor_omega_.setZero();
  motor_thrusts_.setZero();
  return true;
}

Vector<4> Quadrotor::runRatesThrust(const Scalar sim_dt, const Vector<3> &omega, const Command &command){
  const Scalar force = dynamics_.getMass() * command.collective_thrust;

  const Vector<3> omega_err = command.omega - omega;

  const Vector<3> body_torque_des =
  dynamics_.getJ() * Kinv_ang_vel_tau_ * omega_err +
  state_.w.cross(dynamics_.getJ() * state_.w);

  const Vector<4> thrust_and_torque(force, body_torque_des.x(),
                                  body_torque_des.y(), body_torque_des.z());

  const Vector<4> motor_thrusts_des = B_allocation_inv_ * thrust_and_torque;

  return dynamics_.clampThrust(motor_thrusts_des);
}

Vector<4> Quadrotor::runTorquesThrust(const Scalar sim_dt, const Command &command){
  const Scalar force = dynamics_.getMass() * command.collective_thrust;
  const Vector<3> body_torque_des = command.torques;
  const Vector<4> thrust_and_torque(force, body_torque_des.x(),
                                  body_torque_des.y(), body_torque_des.z());
  const Vector<4> motor_thrusts_des = B_allocation_inv_ * thrust_and_torque;
  return dynamics_.clampThrust(motor_thrusts_des);
}

Vector<4> Quadrotor::runFlightCtl(const Scalar sim_dt, const QuadState &state_,
                                  const Command &command) {
  //check cmd type and return thrusts
  switch (cmd_.type) {
    case CommandType::SINGLE_ROTOR_THRUSTS:
      return command.thrusts;
    case CommandType::RATES_THRUST:
      return runRatesThrust(sim_dt, state_.w, command);
    case CommandType::TORQUES_THRUST:
      return runTorquesThrust(sim_dt, command);
  }
}

void Quadrotor::runMotors(const Scalar sim_dt,
                          const Vector<4> &motor_thruts_des) {
  const Vector<4> motor_omega_des =
    dynamics_.motorThrustToOmega(motor_thruts_des);
  const Vector<4> motor_omega_clamped =
    dynamics_.clampMotorOmega(motor_omega_des);

  // simulate motors as a first-order system
  const Scalar c = std::exp(-sim_dt * dynamics_.getMotorTauInv());
  motor_omega_ = c * motor_omega_ + (1.0 - c) * motor_omega_clamped;

  motor_thrusts_ = dynamics_.motorOmegaToThrust(motor_omega_);
  motor_thrusts_ = dynamics_.clampThrust(motor_thrusts_);
}

bool Quadrotor::setState(const QuadState &state) {
  if (!state.valid()) return false;
  state_ = state;
  return true;
}

bool Quadrotor::setWorldBox(const Ref<Matrix<3, 2>> box) {
  if (box(0, 0) >= box(0, 1) || box(1, 0) >= box(1, 1) ||
      box(2, 0) >= box(2, 1)) {
    return false;
  }
  world_box_ = box;
  return true;
}

bool Quadrotor::constrainInWorldBox(const QuadState &old_state) {
  if (!old_state.valid()) return false;

  // violate world box constraint in the x-axis
  if (state_.x(QS::POSX) < world_box_(0, 0) ||
      state_.x(QS::POSX) > world_box_(0, 1)) {
    state_.x(QS::POSX) = old_state.x(QS::POSX);
    state_.x(QS::VELX) = 0.0;
  }

  // violate world box constraint in the y-axis
  if (state_.x(QS::POSY) < world_box_(1, 0) ||
      state_.x(QS::POSY) > world_box_(1, 1)) {
    state_.x(QS::POSY) = old_state.x(QS::POSY);
    state_.x(QS::VELY) = 0.0;
  }

  // violate world box constraint in the x-axis
  if (state_.x(QS::POSZ) <= world_box_(2, 0) ||
      state_.x(QS::POSZ) > world_box_(2, 1)) {
    //
    state_.x(QS::POSZ) = world_box_(2, 0);

    // reset velocity to zero
    state_.x(QS::VELX) = 0.0;
    state_.x(QS::VELY) = 0.0;

    // reset acceleration to zero
    state_.a << 0.0, 0.0, 0.0;
    // reset angular velocity to zero
    state_.w << 0.0, 0.0, 0.0;
  }
  return true;
}

bool Quadrotor::getState(QuadState *const state) const {
  if (!state_.valid()) return false;

  *state = state_;
  return true;
}

bool Quadrotor::getMotorThrusts(Ref<Vector<4>> motor_thrusts) const {
  motor_thrusts = motor_thrusts_;
  return true;
}

bool Quadrotor::getMotorOmega(Ref<Vector<4>> motor_omega) const {
  motor_omega = motor_omega_;
  return true;
}

bool Quadrotor::getDynamics(QuadrotorDynamics *const dynamics) const {
  if (!dynamics_.valid()) return false;
  *dynamics = dynamics_;
  return true;
}

const QuadrotorDynamics &Quadrotor::getDynamics() { return dynamics_; }

bool Quadrotor::updateDynamics(const QuadrotorDynamics &dynamics) {
  if (!dynamics.valid()) {
    std::cout << "[Quadrotor] dynamics is not valid!" << std::endl;
    return false;
  }
  //max dt is 2.5e-3
  dynamics_ = dynamics;
  integrator_ptr_ =
    std::make_unique<IntegratorRK4>(dynamics_.getDynamicsFunction(), 2.5e-3);

  B_allocation_ = dynamics_.getAllocationMatrix();
  B_allocation_inv_ = B_allocation_.inverse();
  return true;
}


Vector<3> Quadrotor::getSize(void) const { return size_; }

Vector<3> Quadrotor::getPosition(void) const { return state_.p; }


bool Quadrotor::getCollision() const { return collision_; }

std::ostream &operator<<(std::ostream &os, const Quadrotor &quad) {
  os.precision(3);
  QuadState state = quad.state_;
  os << "Quadrotor :\n"
     << "pose =             [" << state.p.x()<< ", " << state.p.y() << ", " << state.p.z() << "]\n"
     << "velocity =         [" << state.v.x() << ", " << state.v.y() << ", " << state.v.z() << "]\n"
     << "bodyrates =        [" << state.w.x() << ", " << state.w.y() << ", " << state.w.z() << "]\n"
     << "acceleration =     [" << state.a.x() << ", " << state.a.y() << ", " << state.a.z() << "]\n"
     << "attitude =         [" << state.q().w() << ", " << state.q().x() << ", " << state.q().y() << ", " << state.q().z() << "]\n"
     << "torque =           [" << state.tau.x() << ", " << state.tau.y() << ", " << state.tau.z() << "]\n"
     << "bw =               [" << state.bw.x() << ", " << state.bw.y() << ", " << state.bw.z() << "]\n"
     << "ba =               [" << state.ba.x() << ", " << state.ba.y() << ", " << state.ba.z() << "]\n"
     << "time =             [" << state.t << "]\n"
     << std::endl;
  os.precision();
  return os;
}
}  // namespace flightlib
