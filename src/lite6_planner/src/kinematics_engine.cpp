#include "lite6_planner/kinematics_engine.hpp"

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_packages_with_prefixes.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/collision/collision.hpp>
#include <coal/shape/geometric_shapes.h>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>

namespace lite6_planner {
namespace {
constexpr double kTwoPi = 6.2831853071795864769;
constexpr double kFKPositionTol = 2e-4;  // metres
constexpr double kFKRotationTol = 2e-3;  // radians
constexpr double kJacobianLength = 0.25; // metres; normalize linear rows
bool all_finite(const std::array<double,6>& q) {
  for (double x:q) if (!std::isfinite(x)) return false;
  return true;
}
double rotation_error(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
  const Eigen::AngleAxisd aa(a.transpose()*b);
  return std::abs(aa.angle());
}
} // namespace

KinematicsEngine::KinematicsEngine() = default;

std::string IKDiagnostics::summary() const {
  if(!input_error.empty()) return input_error;
  std::ostringstream out;
  out<<"No IK branch satisfies all required checks";
  std::map<std::string,size_t> counts;
  for(const auto& r:rejected) ++counts[r.reason];
  for(const auto& entry:counts) out<<"; "<<entry.first<<"="<<entry.second;
  // Each branch records its first failing check, not an exhaustive diagnosis.
  for(const auto& r:rejected)
    out<<"\nbranch "<<r.branch<<": "<<r.reason<<": "<<r.detail;
  return out.str();
}

bool KinematicsEngine::load_limits(const std::string& file) {
  try {
    const auto root = YAML::LoadFile(file);
    for (size_t i=0;i<6;++i) {
      const auto n = root["joint_limits"]["joint" + std::to_string(i+1)];
      auto& lim=joint_limits_[i];
      lim.min_pos=n["min_pos"].as<double>(); lim.max_pos=n["max_pos"].as<double>();
      lim.max_vel=n["max_vel"].as<double>(); lim.max_acc=n["max_acc"].as<double>();
      // Required: do not silently fall back to 10 * acceleration.
      lim.max_jerk=n["max_jerk"].as<double>();
      if (!(std::isfinite(lim.min_pos)&&std::isfinite(lim.max_pos)&&
            std::isfinite(lim.max_vel)&&std::isfinite(lim.max_acc)&&
            std::isfinite(lim.max_jerk)&&lim.max_pos>lim.min_pos&&
            lim.max_vel>0&&lim.max_acc>0&&lim.max_jerk>0)) return false;
    }
    const auto c=root["cartesian_limits"];
    cart_limits_={c["max_trans_vel"].as<double>(),c["max_trans_acc"].as<double>(),
                  c["max_rot_vel"].as<double>(),c["max_rot_acc"].as<double>()};
    if (!(cart_limits_.max_trans_vel>0&&cart_limits_.max_trans_acc>0&&
          cart_limits_.max_rot_vel>0&&cart_limits_.max_rot_acc>0)) return false;
    const double s=root["singularity_threshold"].as<double>();
    const double j=root["quadrant_jump_threshold"].as<double>();
    if (!std::isfinite(s)||s<=0||!std::isfinite(j)||j<=0) return false;
    singularity_threshold_=s;
    quadrant_jump_threshold_=j;
    return true;
  } catch (const std::exception& e) {
    std::cerr<<"[KinematicsEngine] limits config: "<<e.what()<<'\n';
    return false;
  }
}

bool KinematicsEngine::load_ik_config(const std::string& file) {
  try {
    auto root=YAML::LoadFile(file);
    ground_size_=root["ground_plane"]["size"].as<std::vector<double>>();
    ground_pos_=root["ground_plane"]["position"].as<std::vector<double>>();
    auto weights=root["ik_weights"].as<std::vector<double>>();
    if (ground_size_.size()!=3||ground_pos_.size()!=3||weights.size()!=6) return false;
    for (size_t i=0;i<3;++i) {
      if (!(std::isfinite(ground_pos_[i])&&std::isfinite(ground_size_[i])&&ground_size_[i]>0)) return false;
    }
    for(size_t i=0;i<6;++i) {
      if (!(std::isfinite(weights[i])&&weights[i]>0)) return false;
      ik_weights_[i]=weights[i];
    }
    return true;
  } catch (const std::exception& e) {
    std::cerr<<"[KinematicsEngine] IK config: "<<e.what()<<'\n';
    return false;
  }
}

bool KinematicsEngine::init_robot_model(const std::string& urdf, const std::string& srdf) {
  try {
    pinocchio::urdf::buildModel(urdf,pin_model_);
    const auto flange = pin_model_.getFrameId("link6");
    const auto tcp = pin_model_.getFrameId("link_eef");
    if (flange >= pin_model_.nframes || tcp >= pin_model_.nframes ||
        pin_model_.frames[flange].parentJoint != pin_model_.frames[tcp].parentJoint) {
      throw std::runtime_error("Expected a fixed flange-to-TCP transform");
    }
    flange_to_tcp_ = pin_model_.frames[flange].placement.actInv(
      pin_model_.frames[tcp].placement).toHomogeneousMatrix();
    tcp_frame_id_ = tcp;
    std::vector<std::string> dirs;
    for(const auto& pkg:ament_index_cpp::get_packages_with_prefixes()) dirs.push_back(pkg.second+"/share");
    pinocchio::urdf::buildGeom(pin_model_,urdf,pinocchio::COLLISION,geom_model_,dirs);
    geom_model_.addAllCollisionPairs();
    pinocchio::srdf::removeCollisionPairs(pin_model_,geom_model_,srdf);
    auto ground=std::make_shared<coal::Box>(ground_size_[0],ground_size_[1],ground_size_[2]);
    auto base=pin_model_.getFrameId("link_base");
    const auto joint=pin_model_.frames[base].parentJoint;
    pinocchio::GeometryObject object("ground_plane",joint,base,
      pinocchio::SE3(Eigen::Matrix3d::Identity(),Eigen::Vector3d(ground_pos_[0],ground_pos_[1],ground_pos_[2])),ground);
    geom_model_.addGeometryObject(object);
    const size_t last=geom_model_.geometryObjects.size()-1;
    for(size_t i=0;i<last;++i) geom_model_.addCollisionPair(pinocchio::CollisionPair(i,last));
    if (pin_model_.nq!=6||pin_model_.nv!=6) throw std::runtime_error("Expected six 1-DoF joints");
    return true;
  } catch(const std::exception& e) {
    std::cerr<<"[KinematicsEngine] model init: "<<e.what()<<'\n';
    return false;
  }
}

bool KinematicsEngine::joint_valid(const std::array<double,6>& q,double eps) const {
  if (!all_finite(q)) return false;
  for(size_t i=0;i<6;++i)
    if(q[i]<joint_limits_[i].min_pos-eps||q[i]>joint_limits_[i].max_pos+eps) return false;
  return true;
}

Eigen::Matrix4d KinematicsEngine::forward_pose(const std::array<double,6>& q) const {
  pinocchio::Data data(pin_model_);
  return forward_pose(q,data);
}

Eigen::Matrix4d KinematicsEngine::forward_pose(
    const std::array<double,6>& q,pinocchio::Data& data) const {
  const Eigen::Map<const Eigen::Matrix<double,6,1>> qe(q.data());
  pinocchio::forwardKinematics(pin_model_,data,qe);
  const auto& frame=pinocchio::updateFramePlacement(pin_model_,data,tcp_frame_id_);
  Eigen::Matrix4d T=Eigen::Matrix4d::Identity();
  T.topLeftCorner<3,3>()=frame.rotation();
  T.topRightCorner<3,1>()=frame.translation();
  return T;
}

bool KinematicsEngine::check_collision(const std::array<double,6>& q,std::string* detail) const {
  auto workspace=collision_workspace();
  return check_collision(q,workspace,detail);
}
bool KinematicsEngine::check_collision(const std::array<double,6>& q,
    CollisionWorkspace& workspace,std::string* detail) const {
  if(detail) detail->clear();
  if (!all_finite(q)) {
    if(detail) *detail="Non-finite joint configuration";
    return true;
  }
  auto& data=workspace.data;
  auto& geometry_data=workspace.geometry;
  const Eigen::Map<const Eigen::Matrix<double,6,1>> qe(q.data());
  pinocchio::forwardKinematics(pin_model_,data,qe);
  pinocchio::updateGeometryPlacements(pin_model_,data,geom_model_,geometry_data);
  const bool collided=pinocchio::computeCollisions(geom_model_,geometry_data,true);
  if(collided&&detail) {
    *detail="Collision detected";
    for(size_t i=0;i<geom_model_.collisionPairs.size();++i) {
      if(geometry_data.collisionResults[i].isCollision()) {
        const auto& pair=geom_model_.collisionPairs[i];
        *detail="Collision between "+geom_model_.geometryObjects[pair.first].name+
            " and "+geom_model_.geometryObjects[pair.second].name;
        break;
      }
    }
  }
  return collided;
}

double KinematicsEngine::check_singularity(const std::array<double,6>& q) const {
  if(!all_finite(q)) return 0;
  pinocchio::Data data(pin_model_);
  return check_singularity(q,data);
}

double KinematicsEngine::check_singularity(
    const std::array<double,6>& q,pinocchio::Data& data) const {
  if(!all_finite(q)) return 0;
  const Eigen::Map<const Eigen::Matrix<double,6,1>> qe(q.data());
  pinocchio::Data::Matrix6x J(6,6);
  J.setZero();
  pinocchio::computeFrameJacobian(pin_model_,data,qe,
    tcp_frame_id_,pinocchio::LOCAL_WORLD_ALIGNED,J);
  // Pinocchio spatial order: [linear velocity; angular velocity].
  J.topRows(3)/=kJacobianLength;
  Eigen::JacobiSVD<Eigen::Matrix<double,6,6>> svd(J,Eigen::ComputeFullU|Eigen::ComputeFullV);
  return svd.singularValues()[5];
}

void KinematicsEngine::raw_inverse_kinematics(const Eigen::Matrix4d& T, double cg0[8][6]) const
{
    double R11 = T(0,0), R12 = T(0,1), R13 = T(0,2), Xt = T(0,3);
    double R21 = T(1,0), R22 = T(1,1), R23 = T(1,2), Yt = T(1,3);
    double R31 = T(2,0), R32 = T(2,1), R33 = T(2,2), Zt = T(2,3);

    double t1, t2, t3, t4, t5, t6, t7, t8, t9, t10;
    double t11, t12, t13, t14, t15, t16, t17, t18, t19, t20;
    double t21, t22, t23, t24, t25, t26, t27, t28, t29, t30;
    double t31, t32, t33, t34, t35, t36, t37, t38, t39, t40, t41;

    // Inverse kinematics calculations (Maple generated)
    t1 = 0.1e1 / 0.16e2;
    t2 = R23 * t1 - Yt;
    t3 = R13 * t1 - Xt;
    t4 = atan2(t2, t3);
    t5 = -0.32e2;
    t6 = 0.256e3;
    t5 = R13 * R13 + R23 * R23 + t5 * (R13 * Xt + R23 * Yt) + t6 * (Xt * Xt + Yt * Yt);
    t5 = pow(t5, -0.1e1 / 0.2e1);
    t7 = t5 * (-pow(t2, 0.2e1) - pow(t3, 0.2e1));
    t8 = -0.487e3 / 0.2000e4;
    t1 = -R33 * t1 + Zt + t8;
    t8 = pow(t7, 0.2e1);
    t9 = pow(t1, 0.2e1);
    t10 = 0.2500e4 / 0.1001e4 * t9;
    t11 = 0.640000e6 / 0.1001e4 * t8 + t10 - 0.14842517e8 / 0.308000000e9;
    t8 = -pow(t11, 0.2e1) + t6 * t8 + t9;

    if (t8 < -1e-5) {
        t8 = std::numeric_limits<double>::quiet_NaN(); 
    } else if (t8 < 0.0) {
        t8 = 0.0;
    }

    t8 = std::sqrt(t8); 
    t12 = atan2(t11, -t8);
    t13 = 0.16e2;
    t14 = atan2(t1, t13 * t7);
    t15 = t12 - t14;
    t16 = sin(t15);
    t17 = cos(t15);
    t18 = 0.87e2 / 0.16000e5 * R33;
    t19 = -0.87e2 / 0.1000e4 * Zt;
    t20 = -0.22761e5 / 0.6250e4 * t7;
    t21 = 0.22783761e8 / 0.500000000e9 * t16 + 0.87087e5 / 0.5000000e7 * t17 + 0.42369e5 / 0.2000000e7 + t18 + t19 + t20;
    t22 = 0.22761e5 / 0.1600000e7 * R33;
    t7 = 0.174e3 / 0.125e3 * t7;
    t23 = -0.22761e5 / 0.100000e6 * Zt;
    t16 = 0.22783761e8 / 0.500000000e9 * t17 - 0.87087e5 / 0.5000000e7 * t16 + 0.11084607e8 / 0.200000000e9 + t22 + t7 + t23;
    t12 = t12 - t14 - atan2(t21, t16);
    t17 = pow(t16, 0.2e1) + pow(t21, 0.2e1);
    t17 = pow(t17, -0.1e1 / 0.2e1);
    t24 = R13 * t3 + R23 * t2;
    t21 = t21 * t17;
    t25 = t13 * t16 * t17 * t24 * t5 - t21 * R33;
    t26 = t13 * t5 * (R13 * t2 - R23 * t3);
    t21 = t21 * t5;
    t16 = t16 * t17;
    t17 = t13 * t21 * t24 + t16 * R33;
    t27 = -pow(t17, 0.2e1) + 0.1e1;

    if (t27 < -1e-5) {
        t27 = std::numeric_limits<double>::quiet_NaN(); 
    } else if (t27 < 0.0) {
        t27 = 0.0;
    }

    t27 = std::sqrt(t27);
    t17 = -t17;
    t28 = R12 * t3 + R22 * t2;
    t29 = -t13 * t21 * t28 - t16 * R32;
    t30 = R11 * t3 + R21 * t2;
    t16 = t13 * t21 * t30 + t16 * R31;
    t8 = atan2(t11, t8);
    t11 = t8 - t14;
    t21 = sin(t11);
    t31 = cos(t11);
    t20 = 0.22783761e8 / 0.500000000e9 * t21 + 0.87087e5 / 0.5000000e7 * t31 + 0.42369e5 / 0.2000000e7 + t18 + t19 + t20;
    t7 = 0.22783761e8 / 0.500000000e9 * t31 - 0.87087e5 / 0.5000000e7 * t21 + 0.11084607e8 / 0.200000000e9 + t22 + t7 + t23;
    t8 = t8 - atan2(t20, t7) - t14;
    t14 = pow(t20, 0.2e1) + pow(t7, 0.2e1);
    t14 = pow(t14, -0.1e1 / 0.2e1);
    t20 = t20 * t14;
    t21 = t13 * t14 * t24 * t5 * t7 - t20 * R33;
    t20 = t20 * t5;
    t7 = t7 * t14;
    t14 = t13 * t20 * t24 + t7 * R33;
    t24 = -pow(t14, 0.2e1) + 0.1e1;

    if (t24 < -1e-5) {
        t24 = std::numeric_limits<double>::quiet_NaN(); 
    } else if (t24 < 0.0) {
        t24 = 0.0;
    }
    
    t24 = std::sqrt(t24);
    t14 = -t14;
    t28 = -t13 * t20 * t28 - t7 * R32;
    t7 = t13 * t20 * t30 + t7 * R31;
    t20 = atan2(-t2, -t3);
    t30 = t5 * (pow(t2, 0.2e1) + pow(t3, 0.2e1));
    t31 = pow(t30, 0.2e1);
    t10 = 0.640000e6 / 0.1001e4 * t31 + t10 - 0.14842517e8 / 0.308000000e9;
    t6 = -pow(t10, 0.2e1) + t31 * t6 + t9;

    if (t6 < -1e-5) {
        t6 = std::numeric_limits<double>::quiet_NaN(); 
    } else if (t6 < 0.0) {
        t6 = 0.0;
    }

    t6 = std::sqrt(t6);
    t9 = atan2(t10, -t6);
    t1 = atan2(t1, t13 * t30);
    t31 = t9 - t1;
    t32 = sin(t31);
    t33 = cos(t31);
    t34 = -0.22761e5 / 0.6250e4 * t30;
    t35 = 0.22783761e8 / 0.500000000e9 * t32 + 0.87087e5 / 0.5000000e7 * t33 + 0.42369e5 / 0.2000000e7 + t18 + t19 + t34;
    t30 = 0.174e3 / 0.125e3 * t30;
    t32 = 0.22783761e8 / 0.500000000e9 * t33 - 0.87087e5 / 0.5000000e7 * t32 + 0.11084607e8 / 0.200000000e9 + t22 + t30 + t23;
    t9 = atan2(t35, t32) - t9 + t1;
    t33 = pow(t32, 0.2e1) + pow(t35, 0.2e1);
    t33 = pow(t33, -0.1e1 / 0.2e1);
    t36 = -R13 * t3 - R23 * t2;
    t35 = t35 * t33;
    t37 = t13 * t32 * t33 * t36 * t5 - t35 * R33;
    t38 = t13 * t5 * (-R13 * t2 + R23 * t3);
    t35 = t35 * t5;
    t32 = t32 * t33;
    t33 = t13 * t35 * t36 + t32 * R33;
    t39 = -pow(t33, 0.2e1) + 0.1e1;

    if (t39 < -1e-5) {
        t39 = std::numeric_limits<double>::quiet_NaN(); 
    } else if (t39 < 0.0) {
        t39 = 0.0;
    }

    t39 = std::sqrt(t39);
    t33 = -t33;
    t40 = -R12 * t3 - R22 * t2;
    t41 = -t13 * t35 * t40 - t32 * R32;
    t2 = -R11 * t3 - R21 * t2;
    t3 = t13 * t2 * t35 + t32 * R31;
    t6 = atan2(t10, t6);
    t10 = t6 - t1;
    t32 = sin(t10);
    t35 = cos(t10);
    t18 = 0.22783761e8 / 0.500000000e9 * t32 + 0.87087e5 / 0.5000000e7 * t35 + 0.42369e5 / 0.2000000e7 + t18 + t19 + t34;
    t19 = 0.22783761e8 / 0.500000000e9 * t35 - 0.87087e5 / 0.5000000e7 * t32 + 0.11084607e8 / 0.200000000e9 + t22 + t30 + t23;
    t1 = atan2(t18, t19) - t6 + t1;
    t6 = pow(t18, 0.2e1) + pow(t19, 0.2e1);
    t6 = pow(t6, -0.1e1 / 0.2e1);
    t18 = t18 * t6;
    t22 = t13 * t19 * t36 * t5 * t6 - t18 * R33;
    t5 = t18 * t5;
    t6 = t19 * t6;
    t18 = t13 * t36 * t5 + t6 * R33;
    t19 = -pow(t18, 0.2e1) + 0.1e1;

    if (t19 < -1e-5) {
        t19 = std::numeric_limits<double>::quiet_NaN(); 
    } else if (t19 < 0.0) {
        t19 = 0.0;
    }
    
    t19 = std::sqrt(t19);
    t18 = -t18;
    t23 = -t13 * t40 * t5 - t6 * R32;
    t2 = t13 * t2 * t5 + t6 * R31;

    cg0[0][0] = t4;  cg0[0][1] = t15; cg0[0][2] = t12; cg0[0][3] = atan2(t26, t25);   cg0[0][4] = atan2(-t27, t17); cg0[0][5] = atan2(t29, t16);
    cg0[1][0] = t4;  cg0[1][1] = t15; cg0[1][2] = t12; cg0[1][3] = atan2(-t26, -t25); cg0[1][4] = atan2(t27, t17);  cg0[1][5] = atan2(-t29, -t16);
    cg0[2][0] = t4;  cg0[2][1] = t11; cg0[2][2] = t8;  cg0[2][3] = atan2(t26, t21);   cg0[2][4] = atan2(-t24, t14); cg0[2][5] = atan2(t28, t7);
    cg0[3][0] = t4;  cg0[3][1] = t11; cg0[3][2] = t8;  cg0[3][3] = atan2(-t26, -t21); cg0[3][4] = atan2(t24, t14);  cg0[3][5] = atan2(-t28, -t7);
    cg0[4][0] = t20; cg0[4][1] = t31; cg0[4][2] = -t9; cg0[4][3] = atan2(t38, t37);   cg0[4][4] = atan2(-t39, t33); cg0[4][5] = atan2(t41, t3);
    cg0[5][0] = t20; cg0[5][1] = t31; cg0[5][2] = -t9; cg0[5][3] = atan2(-t38, -t37); cg0[5][4] = atan2(t39, t33);  cg0[5][5] = atan2(-t41, -t3);
    cg0[6][0] = t20; cg0[6][1] = t10; cg0[6][2] = -t1; cg0[6][3] = atan2(t38, t22);   cg0[6][4] = atan2(-t19, t18); cg0[6][5] = atan2(t23, t2);
    cg0[7][0] = t20; cg0[7][1] = t10; cg0[7][2] = -t1; cg0[7][3] = atan2(-t38, -t22); cg0[7][4] = atan2(t19, t18);  cg0[7][5] = atan2(-t23, -t2);
}

IKResult KinematicsEngine::solve_optimal_ik(const Eigen::Matrix4d& T,
    const std::array<double,6>& q_ref,
    std::vector<std::array<double,6>>& sorted,bool continuous,IKDiagnostics* diagnostics) const {
  auto workspace=collision_workspace();
  return solve_optimal_ik(T,q_ref,sorted,continuous,workspace,diagnostics);
}

IKResult KinematicsEngine::solve_optimal_ik(const Eigen::Matrix4d& T,
    const std::array<double,6>& q_ref,
    std::vector<std::array<double,6>>& sorted,bool continuous,
    CollisionWorkspace& workspace,IKDiagnostics* diagnostics) const {
  sorted.clear();
  if(diagnostics) *diagnostics=IKDiagnostics{};
  if(!T.allFinite()||!all_finite(q_ref)) {
    if(diagnostics) diagnostics->input_error="Non-finite IK target or reference joints";
    return IKResult::ERR_NO_MATH_SOLUTION;
  }
  if ((T.topLeftCorner<3,3>().transpose()*T.topLeftCorner<3,3>()-
       Eigen::Matrix3d::Identity()).norm()>1e-5 ||
       std::abs(T.topLeftCorner<3,3>().determinant()-1.0)>1e-5) {
    if(diagnostics) diagnostics->input_error="IK target rotation is not a valid rotation matrix";
    return IKResult::ERR_NO_MATH_SOLUTION;
  }

  double raw[8][6];
  // The analytical solver targets the original flange; all public poses use TCP.
  raw_inverse_kinematics(T * flange_to_tcp_.inverse(),raw);
  struct Candidate {std::array<double,6> q; double cost;};
  std::vector<Candidate> candidates;
  bool blocked=false, jumped=false, singular=false;
  size_t branch_index=0;
  for(auto& branch:raw) {
    ++branch_index;
    auto reject=[&](const std::string& reason,const std::string& detail) {
      if(diagnostics) diagnostics->rejected.push_back({branch_index,reason,detail});
    };
    auto joint_detail=[&](int i,double value) {
      std::ostringstream out;
      out<<std::setprecision(10)<<"joint"<<i+1<<" value="<<value
         <<" rad, range=["<<joint_limits_[i].min_pos<<", "<<joint_limits_[i].max_pos<<"] rad";
      return out.str();
    };
    auto reject_invalid=[&](const std::array<double,6>& values) {
      for(int i=0;i<6;++i) {
        if(!std::isfinite(values[i])||values[i]<joint_limits_[i].min_pos-tolerance::kJointPosition||
            values[i]>joint_limits_[i].max_pos+tolerance::kJointPosition) {
          reject("joint limit / invalid joint",joint_detail(i,values[i]));break;
        }
      }
    };
    std::array<double,6> q{};
    bool valid=true, branch_jump=false;
    double cost=0;
    std::string jump_detail;
    for(int i=0;i<6;++i) {
      const double base=branch[i];
      if(!std::isfinite(base)) {
        reject("no finite analytic solution","joint"+std::to_string(i+1)+" is non-finite");
        valid=false;break;
      }
      const auto& lim=joint_limits_[i];
      const int k_lo=static_cast<int>(std::ceil((lim.min_pos-base-1e-10)/kTwoPi));
      const int k_hi=static_cast<int>(std::floor((lim.max_pos-base+1e-10)/kTwoPi));
      if(k_lo>k_hi) {
        reject("joint limit",joint_detail(i,base)+"; no equivalent angle within limits");
        valid=false;blocked=true;break;
      }
      const int k=std::clamp(static_cast<int>(std::round((q_ref[i]-base)/kTwoPi)),k_lo,k_hi);
      q[i]=base+k*kTwoPi;
      const double dq=q[i]-q_ref[i];
      cost+=ik_weights_[i]*dq*dq;
      if(continuous&&std::abs(dq)>quadrant_jump_threshold_) {
        branch_jump=true;
        if(jump_detail.empty()) {
          std::ostringstream out;
          out<<std::setprecision(10)<<"joint"<<i+1<<" delta="<<dq
             <<" rad, threshold="<<quadrant_jump_threshold_<<" rad";
          jump_detail=out.str();
        }
      }
    }
    if(!valid) continue;
    if(branch_jump) {reject("branch jump",jump_detail);jumped=true;continue;}
    if(!joint_valid(q)) {reject_invalid(q);blocked=true;continue;}
    // Seeded closed-loop pose correction (CLIK): keep the Maple branch,
    // but correct finite symbolic/model mismatch using a normalized SVD-DLS step.
    // Only a small local correction is permitted; this is NOT a new global IK.
    for(int iteration=0;iteration<8;++iteration) {
      const Eigen::Matrix4d fk=forward_pose(q,workspace.data);
      if((fk.topRightCorner<3,1>()-T.topRightCorner<3,1>()).norm()<kFKPositionTol*0.25&&
         rotation_error(fk.topLeftCorner<3,3>(),T.topLeftCorner<3,3>())<kFKRotationTol*0.25) break;
      const pinocchio::SE3 current(fk.topLeftCorner<3,3>(),fk.topRightCorner<3,1>());
      const pinocchio::SE3 desired(T.topLeftCorner<3,3>(),T.topRightCorner<3,1>());
      const auto pose_error=pinocchio::log6(current.actInv(desired)).toVector();
      if(!pose_error.allFinite()) break;
      const Eigen::Map<const Eigen::Matrix<double,6,1>> qe(q.data());
      pinocchio::Data::Matrix6x jac(6,6);jac.setZero();
      pinocchio::computeFrameJacobian(pin_model_,workspace.data,qe,
          tcp_frame_id_,pinocchio::LOCAL,jac);
      jac.topRows(3)/=kJacobianLength;
      Eigen::Matrix<double,6,1> error=pose_error;
      error.head<3>()/=kJacobianLength;
      Eigen::JacobiSVD<Eigen::Matrix<double,6,6>> svd(jac,Eigen::ComputeFullU|Eigen::ComputeFullV);
      const double sigma=svd.singularValues()[5];
      const double lambda=0.001+0.06*std::pow(std::max(0.0,0.05-sigma)/0.05,2);
      Eigen::Matrix<double,6,1> scaled=svd.matrixU().transpose()*error;
      for(int j=0;j<6;++j) {
        const double a=svd.singularValues()[j];
        scaled[j]*=a/(a*a+lambda*lambda);
      }
      Eigen::Matrix<double,6,1> correction=svd.matrixV()*scaled;
      const double norm=correction.lpNorm<Eigen::Infinity>();
      if(!std::isfinite(norm)) break;
      if(norm>0.12) correction*=0.12/norm;
      for(int j=0;j<6;++j) q[j]+=correction[j];
      if(!joint_valid(q)) break;
    }
    if(!joint_valid(q)) {reject_invalid(q);blocked=true;continue;}
    const Eigen::Matrix4d fk=forward_pose(q,workspace.data);
    const double pos_error=(fk.topRightCorner<3,1>()-T.topRightCorner<3,1>()).norm();
    const double rot_error=rotation_error(fk.topLeftCorner<3,3>(),T.topLeftCorner<3,3>());
    if(pos_error>kFKPositionTol||rot_error>kFKRotationTol) {
      std::ostringstream out;
      out<<std::setprecision(10)<<"position residual="<<pos_error<<" m (tolerance="
         <<kFKPositionTol<<" m), rotation residual="<<rot_error
         <<" rad (tolerance="<<kFKRotationTol<<" rad) after local correction";
      reject("FK verification failed",out.str());
      continue; // Do not publish an unverified inverse-kinematics branch.
    }
    if(continuous) {
      const double sigma=check_singularity(q,workspace.data),threshold=singularity_threshold_;
      if(sigma<threshold) {
        std::ostringstream out;
        out<<std::setprecision(10)<<"normalized Jacobian sigma_min="<<sigma<<", threshold="<<threshold;
        reject("near singularity",out.str());singular=true;continue;
      }
    }
    std::string collision;
    if(check_collision(q,workspace,diagnostics?&collision:nullptr)) {
      reject("collision",collision);blocked=true;continue;
    }
    cost=0.0;
    for(int j=0;j<6;++j) cost+=ik_weights_[j]*std::pow(q[j]-q_ref[j],2);
    bool duplicate=false;
    for(const auto& c:candidates) {
      double d=0;
      for(int j=0;j<6;++j) d=std::max(d,std::abs(c.q[j]-q[j]));
      if(d<1e-5) {duplicate=true;break;}
    }
    if(!duplicate) candidates.push_back({q,cost});
  }
  std::sort(candidates.begin(),candidates.end(),[](const Candidate& a,const Candidate& b){return a.cost<b.cost;});
  for(const auto& c:candidates) sorted.push_back(c.q);
  if(!sorted.empty()) return IKResult::SUCCESS;
  if(singular) return IKResult::ERR_SINGULARITY;
  if(jumped) return IKResult::ERR_QUADRANT_JUMP;
  if(blocked) return IKResult::ERR_LIMIT_OR_COLLISION;
  return IKResult::ERR_NO_MATH_SOLUTION;
}

} // namespace lite6_planner
