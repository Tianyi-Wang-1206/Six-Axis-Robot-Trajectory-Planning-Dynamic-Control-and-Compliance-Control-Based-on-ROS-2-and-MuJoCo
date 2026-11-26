#ifndef LITE6_HARDWARE__LUGRE_FRICTION_HPP_
#define LITE6_HARDWARE__LUGRE_FRICTION_HPP_

#include <array>
#include <cmath>

namespace lite6_hardware
{

struct LuGreParameters
{
  double fc;      // N m
  double fs;      // N m
  double vs;      // rad/s
  double sigma0;  // N m/rad
  double sigma1;  // N m s/rad
  double sigma2;  // N m s/rad
};

struct JointFrictionParameters
{
  const char * name;
  LuGreParameters parameters;
};

constexpr std::array<JointFrictionParameters, 6> kLite6LuGreParameters{{
  {"joint1", {1.5, 1.725, 0.02, 3000.0, 0.075, 1.0}},
  {"joint2", {1.5, 1.725, 0.02, 3000.0, 0.075, 1.0}},
  {"joint3", {1.0, 1.15, 0.02, 2000.0, 0.05, 0.5}},
  {"joint4", {0.5, 0.575, 0.02, 1000.0, 0.025, 0.2}},
  {"joint5", {0.2, 0.23, 0.02, 400.0, 0.01, 0.1}},
  {"joint6", {0.1, 0.115, 0.02, 200.0, 0.005, 0.05}},
}};

class LuGreFriction
{
public:
  explicit LuGreFriction(const LuGreParameters & parameters)
  : parameters_(parameters) {}

  // Return tau_f at (z_n, v_n), then advance z by one physics timestep.
  // tau_f is the torque the drive must overcome; inject its NEGATIVE into MuJoCo.
  double step(double velocity, double dt)
  {
    const auto & p = parameters_;
    const double ratio = velocity / p.vs;
    const double g = p.fc + (p.fs - p.fc) * std::exp(-ratio * ratio);
    const double rate = p.sigma0 * std::abs(velocity) / g;
    const double z_dot = std::fma(-rate, z_, velocity);
    const double torque = p.sigma0 * z_ + p.sigma1 * z_dot + p.sigma2 * velocity;

    if (velocity != 0.0) {
      // Exact constant-velocity update; expm1 preserves precision near zero.
      // z_next = z + (z_ss - z) * (1 - exp(-rate * dt)).
      const double z_ss = std::copysign(g / p.sigma0, velocity);
      z_ = std::fma(z_ss - z_, -std::expm1(-rate * dt), z_);
    }
    // At v == 0 the given ODE has z_dot == 0: retain bristle memory.
    return torque;
  }

  double deflection() const {return z_;}

private:
  LuGreParameters parameters_;
  double z_ = 0.0;  // rad; initially unstrained bristles
};

}  // namespace lite6_hardware

#endif  // LITE6_HARDWARE__LUGRE_FRICTION_HPP_
