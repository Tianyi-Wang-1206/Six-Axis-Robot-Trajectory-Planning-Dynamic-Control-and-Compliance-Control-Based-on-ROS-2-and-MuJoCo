#pragma once
#include <cmath>
#include "lite6_interfaces/msg/compliance_settings.hpp"
namespace lite6_interfaces {
inline bool valid_compliance_settings(const msg::ComplianceSettings& s) {
  const auto bounded=[](double v,double lo,double hi) {
    return std::isfinite(v)&&v>=lo&&v<=hi;
  };
  return bounded(s.joint_stiffness,0.0,100.0)&&bounded(s.joint_damping,0.1,20.0)&&
    bounded(s.translation_stiffness,0.0,1000.0)&&bounded(s.translation_damping,1.0,100.0)&&
    bounded(s.rotation_stiffness,0.0,50.0)&&bounded(s.rotation_damping,0.1,20.0)&&
    bounded(s.sensitivity,1.0,5.0);
}
}
