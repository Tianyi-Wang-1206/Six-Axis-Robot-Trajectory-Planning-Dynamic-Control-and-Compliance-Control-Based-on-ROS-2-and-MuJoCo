#pragma once
#include <cmath>
#include "lite6_interfaces/msg/compliance_settings.hpp"
#include "lite6_interfaces/msg/impedance_trajectory_point.hpp"
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

inline bool valid_impedance_trajectory_gains(const msg::ImpedanceTrajectoryPoint& p) {
  const auto bounded=[](double v,double lo,double hi) {
    return std::isfinite(v)&&v>=lo&&v<=hi;
  };
  for(int i=0;i<3;++i) {
    if(!bounded(p.translation_stiffness[i],0.0,1000.0)||
       !bounded(p.translation_damping[i],1.0,100.0)||
       !bounded(p.rotation_stiffness[i],0.0,50.0)||
       !bounded(p.rotation_damping[i],0.1,20.0)) return false;
  }
  return bounded(p.nullspace_damping,0.0,20.0);
}
}
