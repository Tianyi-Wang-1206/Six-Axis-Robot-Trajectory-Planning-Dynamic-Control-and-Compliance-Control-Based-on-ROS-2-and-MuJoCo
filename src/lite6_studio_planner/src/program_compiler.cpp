#include "lite6_studio_planner/program_compiler.hpp"
#include "lite6_planner/motion_tolerances.hpp"
#include "lite6_interfaces/compliance_validation.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <rclcpp/duration.hpp>
#include <Eigen/SVD>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace lite6_studio_planner
{
namespace
{

using D = lite6_interfaces::msg::CompileDiagnostic;
using IKResult = lite6_planner::IKResult;

constexpr double kCharLength = 0.25;  // m/rad; matches KinematicsEngine & curve_geometry

const std::vector<std::string> kJointNames = {
  "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

builtin_interfaces::msg::Duration make_ros_duration(double t_sec)
{
  const int64_t total_ns = static_cast<int64_t>(std::llround(t_sec * 1e9));
  builtin_interfaces::msg::Duration d;
  d.sec = static_cast<int32_t>(total_ns / 1000000000LL);
  d.nanosec = static_cast<uint32_t>(total_ns % 1000000000LL);
  return d;
}

double rot_diff(const Eigen::Matrix3d & a, const Eigen::Matrix3d & b)
{
  return std::abs(Eigen::AngleAxisd(a.transpose() * b).angle());
}

void add_diag_with_severity(
  std::vector<D> & diags,
  uint8_t severity,
  uint8_t stage,
  const SourceLocation & loc,
  const std::string & code,
  const std::string & msg)
{
  D d;
  d.severity = severity;
  d.stage = stage;
  d.line_number = loc.line;
  d.column_number = loc.column;
  d.statement_index = loc.statement_index;
  d.command_text = loc.text;
  d.code = code;
  d.message = msg;
  diags.push_back(d);
}

void add_diag(
  std::vector<D> & diags,
  uint8_t stage,
  const SourceLocation & loc,
  const std::string & code,
  const std::string & msg)
{
  add_diag_with_severity(diags, D::SEVERITY_ERROR, stage, loc, code, msg);
}

void add_warning(
  std::vector<D> & diags,
  uint8_t stage,
  const SourceLocation & loc,
  const std::string & code,
  const std::string & msg)
{
  add_diag_with_severity(diags, D::SEVERITY_WARNING, stage, loc, code, msg);
}

std::string first_error_message(const std::vector<D> & diags, const std::string & fallback)
{
  for (const auto & d : diags) {
    if (d.severity == D::SEVERITY_ERROR) {
      return d.message;
    }
  }
  return diags.empty() ? fallback : diags.front().message;
}

std::string ik_error_code(IKResult res)
{
  switch (res) {
    case IKResult::ERR_SINGULARITY: return "ERR_SINGULARITY";
    case IKResult::ERR_QUADRANT_JUMP: return "ERR_QUADRANT_JUMP";
    case IKResult::ERR_LIMIT_OR_COLLISION: return "ERR_IK_LIMIT_OR_COLLISION";
    default: return "ERR_NO_IK_SOLUTION";
  }
}

void populate_execution_block(
  const SampledTrajectory1kHz & sampled,
  bool is_impedance,
  const ImpedanceParams & imp_params,
  ExecutionBlock & blk)
{
  blk.type = is_impedance ?
    ExecutionBlockType::IMPEDANCE_TRAJECTORY :
    ExecutionBlockType::RIGID_TRAJECTORY;
  blk.loc = sampled.samples.front().loc;
  blk.start_joints = sampled.samples.front().q;
  blk.end_joints = sampled.samples.back().q;
  blk.duration_sec = sampled.duration_sec;
  blk.sample_locations.clear();
  blk.sample_locations.reserve(sampled.samples.size());
  blk.sample_path_u.clear();
  blk.sample_path_u.reserve(sampled.samples.size());
  for (const auto & smp : sampled.samples) {
    blk.sample_path_u.push_back(smp.path_u);
  }

  if (is_impedance) {
    blk.impedance_params = imp_params;
    blk.impedance_goal.joint_names = kJointNames;
    blk.impedance_goal.transition_time = imp_params.transition_time;
    blk.impedance_goal.end_behavior = imp_params.end_behavior;
    blk.impedance_goal.points.clear();
    blk.impedance_goal.points.reserve(sampled.samples.size());
    for (const auto & s : sampled.samples) {
      lite6_interfaces::msg::ImpedanceTrajectoryPoint pt;
      pt.time_from_start = make_ros_duration(s.time_sec);
      pt.positions = s.q;
      pt.velocities = s.dq;
      pt.accelerations = s.ddq;
      pt.translation_stiffness = imp_params.kt;
      pt.translation_damping = imp_params.dt;
      pt.rotation_stiffness = imp_params.kr;
      pt.rotation_damping = imp_params.dr;
      pt.nullspace_damping = imp_params.null_damping;
      pt.wrench_ff = imp_params.wrench_ff;
      blk.impedance_goal.points.push_back(pt);
      blk.sample_locations.push_back(s.loc);
    }
  } else {
    blk.rigid_trajectory.joint_names = kJointNames;
    blk.rigid_trajectory.points.clear();
    blk.rigid_trajectory.points.reserve(sampled.samples.size());
    for (const auto & s : sampled.samples) {
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.time_from_start = make_ros_duration(s.time_sec);
      pt.positions.assign(s.q.begin(), s.q.end());
      pt.velocities.assign(s.dq.begin(), s.dq.end());
      pt.accelerations.assign(s.ddq.begin(), s.ddq.end());
      blk.rigid_trajectory.points.push_back(pt);
      blk.sample_locations.push_back(s.loc);
    }
  }
}

using Mat6 = Eigen::Matrix<double, 6, 6>;

Vec6 to_vec6(const std::array<double, 6> & a)
{
  Vec6 v;
  for (int i = 0; i < 6; ++i) {
    v[i] = a[static_cast<size_t>(i)];
  }
  return v;
}

std::array<double, 6> to_array(const Vec6 & v)
{
  std::array<double, 6> a{};
  for (int i = 0; i < 6; ++i) {
    a[static_cast<size_t>(i)] = v[i];
  }
  return a;
}

// Twist derivatives [p; w] of a Cartesian sample with respect to the path parameter.
void cartesian_derivatives(const CartesianSample & cs, Vec6 & v1, Vec6 & v2)
{
  v1.head<3>() = cs.p_s;
  v1.tail<3>() = cs.w_s;
  v2.head<3>() = cs.p_ss;
  v2.tail<3>() = cs.w_ss;
}

// Differential kinematics on the LOCAL_WORLD_ALIGNED TCP Jacobian:
//   J q1 = V1,   J q2 = V2 - dJ(q, q1) q1.
bool differential_kinematics(
  const pinocchio::Model & model, pinocchio::Data & data, pinocchio::FrameIndex tcp,
  const Vec6 & q, const Vec6 & v1, const Vec6 & v2, Vec6 & q1, Vec6 & q2)
{
  pinocchio::Data::Matrix6x J(6, 6), dJ(6, 6);
  J.setZero();
  dJ.setZero();
  pinocchio::computeFrameJacobian(model, data, q, tcp, pinocchio::LOCAL_WORLD_ALIGNED, J);
  const Mat6 J6 = J;
  const Eigen::FullPivLU<Mat6> lu(J6);
  if (!lu.isInvertible()) {
    return false;
  }
  q1 = lu.solve(v1);
  pinocchio::computeJointJacobiansTimeVariation(model, data, q, q1);
  pinocchio::updateFramePlacement(model, data, tcp);
  pinocchio::getFrameJacobianTimeVariation(model, data, tcp, pinocchio::LOCAL_WORLD_ALIGNED, dJ);
  q2 = lu.solve(v2 - dJ * q1);
  return q1.allFinite() && q2.allFinite();
}

// Newton refinement of an IK solution onto the exact target pose (keeps the branch).
bool newton_ik(
  const pinocchio::Model & model, pinocchio::Data & data, pinocchio::FrameIndex tcp,
  const Eigen::Matrix4d & target, const Vec6 & q_guess, Vec6 & q_out)
{
  const Eigen::Vector3d p_t = target.topRightCorner<3, 1>();
  const Eigen::Matrix3d r_t = target.topLeftCorner<3, 3>();
  Vec6 q = q_guess;
  pinocchio::Data::Matrix6x J(6, 6);
  for (int it = 0; it < 8; ++it) {
    pinocchio::forwardKinematics(model, data, q);
    const auto & frame = pinocchio::updateFramePlacement(model, data, tcp);
    Vec6 e;
    e.head<3>() = p_t - frame.translation();
    e.tail<3>() = pinocchio::log3(Eigen::Matrix3d(r_t * frame.rotation().transpose()));
    if (e.norm() < 1e-13) {
      break;
    }
    J.setZero();
    pinocchio::computeFrameJacobian(model, data, q, tcp, pinocchio::LOCAL_WORLD_ALIGNED, J);
    q += Mat6(J).fullPivLu().solve(e);
    if (!q.allFinite()) {
      return false;
    }
  }
  pinocchio::forwardKinematics(model, data, q);
  const auto & frame = pinocchio::updateFramePlacement(model, data, tcp);
  const double p_err = (p_t - frame.translation()).norm();
  const double r_err = pinocchio::log3(Eigen::Matrix3d(r_t * frame.rotation().transpose())).norm();
  q_out = q;
  return p_err < 1e-9 && r_err < 1e-8 && (q - q_guess).cwiseAbs().maxCoeff() < 0.05;
}

// Samples a phase-synchronized S-curve; path_u maps the normalized progress s in [0, 1].
SampledTrajectory1kHz scurve_samples(
  const MultiAxisSCurve & scurve, const SourceLocation & loc,
  double path_u_start, double path_u_end)
{
  SampledTrajectory1kHz sampled;
  const int total_ticks = std::max(1, static_cast<int>(std::llround(scurve.duration() * 1000.0)));
  sampled.duration_sec = static_cast<double>(total_ticks) * 0.001;
  sampled.samples.reserve(static_cast<size_t>(total_ticks + 1));
  for (int tick = 0; tick <= total_ticks; ++tick) {
    const double t = static_cast<double>(tick) * 0.001;
    SampledTrajectory1kHz::Sample smp;
    smp.time_sec = t;
    const double prog = (tick == total_ticks) ? 1.0 : scurve.progress(t);
    smp.path_u = path_u_start + prog * (path_u_end - path_u_start);
    std::array<double, 6> jrk{};
    scurve.eval(t, smp.q, smp.dq, smp.ddq, jrk);
    if (tick == 0 || tick == total_ticks) {
      smp.dq.fill(0.0);
      smp.ddq.fill(0.0);
    }
    smp.loc = loc;
    sampled.samples.push_back(smp);
  }
  return sampled;
}

// Appends b after a; both must meet at rest at the same configuration.
void append_at_rest(SampledTrajectory1kHz & a, const SampledTrajectory1kHz & b)
{
  if (a.samples.empty()) {
    a = b;
    return;
  }
  const double t0 = a.samples.back().time_sec;
  for (size_t k = 1; k < b.samples.size(); ++k) {
    auto smp = b.samples[k];
    smp.time_sec += t0;
    a.samples.push_back(smp);
  }
  a.duration_sec = a.samples.back().time_sec;
}

// Map a desired Cartesian spatial twist V=[vx,vy,vz,wx,wy,wz] and optional spatial acceleration
// at joint configuration q into joint velocities and accelerations using length-normalized
// SVD Jacobian inversion with explicit singularity rejection.
bool cartesian_twist_to_joint(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const pinocchio::FrameIndex frame_id,
  const std::array<double, 6> & q,
  const std::array<double, 6> & spatial_vel,
  const std::array<double, 6> * spatial_acc,
  double singularity_threshold,
  std::array<double, 6> & out_dq,
  std::array<double, 6> * out_ddq,
  double * out_sigma_min = nullptr)
{
  const Eigen::Map<const Eigen::Matrix<double, 6, 1>> qe(q.data());
  if (frame_id >= static_cast<pinocchio::FrameIndex>(model.nframes)) {
    return false;
  }

  pinocchio::Data::Matrix6x J(6, 6);
  J.setZero();
  pinocchio::computeFrameJacobian(
    model, data, qe, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);

  // Normalize linear rows by kCharLength so translational [m] and rotational [rad]
  // singular values share a dimensionless scale consistent with KinematicsEngine.
  Eigen::Matrix<double, 6, 6> J_norm = J;
  J_norm.topRows(3) /= kCharLength;

  const Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(
    J_norm, Eigen::ComputeFullU | Eigen::ComputeFullV);
  double sigma_min = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 6; ++i) {
    sigma_min = std::min(sigma_min, svd.singularValues()[i]);
  }
  if (out_sigma_min != nullptr) {
    *out_sigma_min = sigma_min;
  }
  if (!std::isfinite(sigma_min) || sigma_min < singularity_threshold) {
    return false;
  }

  auto solve_norm = [&](const Eigen::Matrix<double, 6, 1> & rhs_raw) {
    Eigen::Matrix<double, 6, 1> rhs = rhs_raw;
    rhs.head<3>() /= kCharLength;
    Eigen::Matrix<double, 6, 1> proj = svd.matrixU().transpose() * rhs;
    for (int i = 0; i < 6; ++i) {
      proj[i] /= svd.singularValues()[i];
    }
    return Eigen::Matrix<double, 6, 1>(svd.matrixV() * proj);
  };

  Eigen::Matrix<double, 6, 1> V;
  for (int i = 0; i < 6; ++i) {
    V[i] = spatial_vel[static_cast<size_t>(i)];
  }
  const Eigen::Matrix<double, 6, 1> dq = solve_norm(V);
  if (!dq.allFinite()) {
    return false;
  }
  for (size_t i = 0; i < 6; ++i) {
    out_dq[i] = dq[static_cast<int>(i)];
  }

  if (spatial_acc != nullptr && out_ddq != nullptr) {
    pinocchio::computeJointJacobiansTimeVariation(model, data, qe, dq);
    pinocchio::updateFramePlacement(model, data, frame_id);
    pinocchio::Data::Matrix6x dJ(6, 6);
    dJ.setZero();
    pinocchio::getFrameJacobianTimeVariation(
      model, data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, dJ);
    Eigen::Matrix<double, 6, 1> A;
    for (int i = 0; i < 6; ++i) {
      A[i] = (*spatial_acc)[static_cast<size_t>(i)];
    }
    const Eigen::Matrix<double, 6, 1> ddq = solve_norm(A - dJ * dq);
    if (!ddq.allFinite()) {
      return false;
    }
    for (size_t i = 0; i < 6; ++i) {
      (*out_ddq)[i] = ddq[static_cast<int>(i)];
    }
  }
  return true;
}

bool segment_collision_free(
  const lite6_planner::KinematicsEngine & engine,
  lite6_planner::KinematicsEngine::CollisionWorkspace & workspace,
  const std::array<double, 6> & q0,
  const std::array<double, 6> & q1)
{
  double max_step = 0.0;
  for (size_t i = 0; i < 6; ++i) {
    max_step = std::max(max_step, std::abs(q1[i] - q0[i]));
  }
  const int subdivs = std::max(1, static_cast<int>(std::ceil(max_step / 0.04)));
  for (int sub = 1; sub <= subdivs; ++sub) {
    const double alpha = static_cast<double>(sub) / static_cast<double>(subdivs);
    std::array<double, 6> q_interp{};
    for (size_t i = 0; i < 6; ++i) {
      q_interp[i] = (1.0 - alpha) * q0[i] + alpha * q1[i];
    }
    if (engine.check_collision(q_interp, workspace)) {
      return false;
    }
  }
  return true;
}

}  // namespace

ProgramCompiler::ProgramCompiler(const lite6_planner::KinematicsEngine & engine)
: engine_(engine)
{
}

std::array<lite6_planner::JointLimit, 6> ProgramCompiler::motion_limits(bool is_impedance) const
{
  auto limits = engine_.get_joint_limits();
  if (is_impedance) {
    for (auto & lim : limits) {
      lim.max_vel = std::min(lim.max_vel, kImpedanceJointVelocityCap);
    }
  }
  return limits;
}

bool ProgramCompiler::verify_sampled_trajectory_1khz(
  const SampledTrajectory1kHz & sampled,
  bool check_singularity_along_path,
  std::vector<Diagnostic> & diags,
  bool is_impedance) const
{
  if (sampled.samples.size() < 2) {
    SourceLocation loc;
    if (!sampled.samples.empty()) {
      loc = sampled.samples.front().loc;
    }
    add_diag(
      diags, D::STAGE_PLANNING, loc, "ERR_TOO_FEW_SAMPLES",
      "Generated trajectory has fewer than 2 samples");
    return false;
  }

  const auto limits = motion_limits(is_impedance);
  const double sing_thresh = engine_.get_singularity_threshold();
  auto workspace = engine_.collision_workspace();

  // Stage 4: Full 1 kHz Kinematic Limits Verification
  for (size_t k = 0; k < sampled.samples.size(); ++k) {
    const auto & s = sampled.samples[k];
    for (size_t i = 0; i < 6; ++i) {
      if (!std::isfinite(s.q[i]) || !std::isfinite(s.dq[i]) || !std::isfinite(s.ddq[i])) {
        add_diag(
          diags, D::STAGE_KINEMATIC_LIMIT, s.loc, "ERR_NONFINITE_SAMPLE",
          "Non-finite state on joint" + std::to_string(i + 1) +
            " at t=" + std::to_string(s.time_sec) + " s");
        return false;
      }
      const auto & lim = limits[i];
      if (s.q[i] < lim.min_pos - lite6_planner::tolerance::kTrajectoryPosition ||
          s.q[i] > lim.max_pos + lite6_planner::tolerance::kTrajectoryPosition) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(5)
            << "Joint position limit violated at t=" << s.time_sec << " s: joint" << (i + 1)
            << " position=" << s.q[i] << " rad outside [" << lim.min_pos << ", "
            << lim.max_pos << "] rad";
        add_diag(diags, D::STAGE_KINEMATIC_LIMIT, s.loc, "ERR_JOINT_POS_LIMIT", oss.str());
        return false;
      }
      if (std::abs(s.dq[i]) > lim.max_vel + lite6_planner::tolerance::kTrajectoryVelocity) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(5)
            << "Joint velocity limit violated at t=" << s.time_sec << " s: joint" << (i + 1)
            << " |dq|=" << std::abs(s.dq[i]) << " rad/s exceeds max_vel=" << lim.max_vel
            << " rad/s" << (is_impedance ? " (impedance block joint speed cap)" : "");
        add_diag(diags, D::STAGE_KINEMATIC_LIMIT, s.loc, "ERR_JOINT_VEL_LIMIT", oss.str());
        return false;
      }
      if (std::abs(s.ddq[i]) > lim.max_acc + lite6_planner::tolerance::kTrajectoryAcceleration) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(5)
            << "Joint acceleration limit violated at t=" << s.time_sec << " s: joint" << (i + 1)
            << " |ddq|=" << std::abs(s.ddq[i]) << " rad/s^2 exceeds max_acc=" << lim.max_acc
            << " rad/s^2";
        add_diag(diags, D::STAGE_KINEMATIC_LIMIT, s.loc, "ERR_JOINT_ACC_LIMIT", oss.str());
        return false;
      }
      if (k > 0) {
        const double dt = s.time_sec - sampled.samples[k - 1].time_sec;
        if (dt <= 1e-9) {
          add_diag(
            diags, D::STAGE_KINEMATIC_LIMIT, s.loc, "ERR_NONMONOTONE_TIME",
            "Non-monotone sample timestamp at index " + std::to_string(k));
          return false;
        }
        const double jrk = (s.ddq[i] - sampled.samples[k - 1].ddq[i]) / dt;
        if (std::abs(jrk) > lim.max_jerk * 1.001 + 1e-3) {
          std::ostringstream oss;
          oss << std::fixed << std::setprecision(3)
              << "Joint jerk limit violated at t=" << s.time_sec << " s: joint" << (i + 1)
              << " |jerk|=" << std::abs(jrk) << " rad/s^3 exceeds max_jerk=" << lim.max_jerk
              << " rad/s^3";
          add_diag(diags, D::STAGE_KINEMATIC_LIMIT, s.loc, "ERR_JOINT_JERK_LIMIT", oss.str());
          return false;
        }
      }
    }

    if (check_singularity_along_path && (k % 10 == 0 || k + 1 == sampled.samples.size())) {
      const double sigma = engine_.check_singularity(s.q, workspace.data);
      if (sigma < sing_thresh) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6)
            << "Kinematic singularity encountered at t=" << s.time_sec
            << " s: normalized Jacobian sigma_min=" << sigma
            << " < singularity_threshold=" << sing_thresh;
        add_diag(diags, D::STAGE_KINEMATIC_LIMIT, s.loc, "ERR_SINGULARITY", oss.str());
        return false;
      }
    }
  }

  // Stage 5: Full 1 kHz Coal Self & Environment Collision Verification (with 0.04 rad subdivision)
  std::string col_detail;
  if (engine_.check_collision(sampled.samples.front().q, workspace, &col_detail)) {
    add_diag(
      diags, D::STAGE_COLLISION, sampled.samples.front().loc, "ERR_COLLISION",
      "Collision at trajectory start (t=0.000 s): " + col_detail);
    return false;
  }

  for (size_t k = 1; k < sampled.samples.size(); ++k) {
    const auto & prev = sampled.samples[k - 1];
    const auto & curr = sampled.samples[k];
    double max_step = 0.0;
    for (size_t i = 0; i < 6; ++i) {
      max_step = std::max(max_step, std::abs(curr.q[i] - prev.q[i]));
    }
    const int subdivs = std::max(1, static_cast<int>(std::ceil(max_step / 0.04)));
    for (int sub = 1; sub <= subdivs; ++sub) {
      const double alpha = static_cast<double>(sub) / static_cast<double>(subdivs);
      std::array<double, 6> q_interp{};
      for (size_t i = 0; i < 6; ++i) {
        q_interp[i] = (1.0 - alpha) * prev.q[i] + alpha * curr.q[i];
      }
      if (engine_.check_collision(q_interp, workspace, &col_detail)) {
        const double t_approx = prev.time_sec + alpha * (curr.time_sec - prev.time_sec);
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3)
            << col_detail << " at t=" << t_approx << " s (sample " << k << "/"
            << (sampled.samples.size() - 1) << ")";
        add_diag(diags, D::STAGE_COLLISION, curr.loc, "ERR_COLLISION", oss.str());
        return false;
      }
    }
  }

  return true;
}

bool ProgramCompiler::validate_and_clamp_start_joints(
  const std::array<double, 6> & q_in,
  const SourceLocation & loc,
  std::array<double, 6> & q_out,
  std::vector<Diagnostic> & diags) const
{
  const auto & limits = engine_.get_joint_limits();
  q_out = q_in;
  for (size_t i = 0; i < 6; ++i) {
    if (!std::isfinite(q_out[i]) ||
        q_out[i] < limits[i].min_pos - lite6_planner::tolerance::kObservedPosition ||
        q_out[i] > limits[i].max_pos + lite6_planner::tolerance::kObservedPosition) {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(5)
          << "Initial robot joint" << (i + 1) << " position=" << q_out[i]
          << " rad is outside mechanical limits [" << limits[i].min_pos << ", "
          << limits[i].max_pos << "] rad";
      add_diag(diags, D::STAGE_KINEMATIC_LIMIT, loc, "ERR_START_JOINT_LIMIT", oss.str());
      return false;
    }
    q_out[i] = std::clamp(q_out[i], limits[i].min_pos, limits[i].max_pos);
  }
  std::string start_col_detail;
  if (engine_.check_collision(q_out, &start_col_detail)) {
    add_diag(
      diags, D::STAGE_COLLISION, loc, "ERR_START_COLLISION",
      "Start robot configuration is in collision: " + start_col_detail);
    return false;
  }
  return true;
}

bool ProgramCompiler::compile_cartesian_group(
  const std::vector<MotionStatement> & group,
  const std::array<double, 6> & start_q,
  bool is_impedance,
  const ImpedanceParams & imp_params,
  ExecutionBlock & out_block,
  std::vector<Diagnostic> & out_diagnostics) const
{
  auto workspace = engine_.collision_workspace();
  const Eigen::Matrix4d start_pose = engine_.forward_pose(start_q, workspace.data);
  CompositeCartesianPath cart_path;
  SourceLocation err_loc = group.front().loc;
  std::string err_code, err_msg;
  if (!cart_path.build(start_pose, group, err_loc, err_code, err_msg)) {
    add_diag(out_diagnostics, D::STAGE_PLANNING, err_loc, err_code, err_msg);
    return false;
  }

  // Warn when a 3-point MoveC passes a custom intermediate 'via' orientation that differs
  // from both endpoints and the SLERP trajectory, since only 'via' position defines the arc.
  Eigen::Matrix4d curr_pose = start_pose;
  for (const auto & st : group) {
    if (st.type == MotionCommandType::MOVE_L) {
      curr_pose = st.pose_target;
    } else if (st.type == MotionCommandType::MOVE_C_THREE_POINT) {
      if (st.via_has_orientation) {
        const Eigen::Matrix3d R0 = curr_pose.topLeftCorner<3, 3>();
        const Eigen::Matrix3d R1 = st.pose_target.topLeftCorner<3, 3>();
        const Eigen::Matrix3d Rv = st.pose_via.topLeftCorner<3, 3>();
        const Eigen::Vector3d p0 = curr_pose.topRightCorner<3, 1>();
        const Eigen::Vector3d pv = st.pose_via.topRightCorner<3, 1>();
        const Eigen::Vector3d p1 = st.pose_target.topRightCorner<3, 1>();
        const double d_a = (pv - p0).norm();
        const double d_b = (p1 - pv).norm();
        const double frac = (d_a + d_b > 1e-9) ? (d_a / (d_a + d_b)) : 0.5;
        const Eigen::AngleAxisd aa(R0.transpose() * R1);
        const Eigen::Matrix3d R_slerp =
          R0 * Eigen::AngleAxisd(frac * aa.angle(), aa.axis()).toRotationMatrix();
        if (rot_diff(Rv, R0) > 0.05 && rot_diff(Rv, R1) > 0.05 && rot_diff(Rv, R_slerp) > 0.05) {
          add_warning(
            out_diagnostics, D::STAGE_SEMANTIC, st.loc, "WARN_MOVEC_VIA_ORI_IGNORED",
            "MoveC 3-point 'via' orientation is ignored (only its XYZ position defines the arc; "
            "orientation interpolates smoothly from start to target). Pass via=[x, y, z] to omit orientation.");
        }
      }
      curr_pose = st.pose_target;
    } else if (st.type == MotionCommandType::MOVE_C_CENTER_ANGLE) {
      const Eigen::Vector3d p0 = curr_pose.topRightCorner<3, 1>();
      const Eigen::Vector3d ez = st.circle_axis.normalized();
      const Eigen::Vector3d rel = p0 - st.circle_center;
      const Eigen::Vector3d eff_center = st.circle_center + rel.dot(ez) * ez;
      const Eigen::Matrix3d R_rot =
        Eigen::AngleAxisd(st.circle_angle_rad, ez).toRotationMatrix();
      Eigen::Matrix4d next_pose = Eigen::Matrix4d::Identity();
      next_pose.topRightCorner<3, 1>() = eff_center + R_rot * (p0 - eff_center);
      next_pose.topLeftCorner<3, 3>() = st.has_explicit_target_ori ?
        st.circle_target_rot : (R_rot * curr_pose.topLeftCorner<3, 3>());
      curr_pose = next_pose;
    }
  }

  SampledTrajectory1kHz sampled;
  if (!plan_cartesian_path(
        cart_path, start_q, group.front().options, group.front().loc, is_impedance, sampled,
        out_diagnostics))
  {
    return false;
  }

  out_block = ExecutionBlock();
  out_block.source_kind = BlockSourceKind::CARTESIAN_GROUP;
  out_block.source_motions = group;
  out_block.cartesian_path = std::make_shared<CompositeCartesianPath>(std::move(cart_path));
  out_block.path_start_joints = start_q;
  populate_execution_block(sampled, is_impedance, imp_params, out_block);
  return true;
}

bool ProgramCompiler::build_cartesian_joint_path(
  const CartesianPath & cart_path,
  const std::array<double, 6> & start_q,
  C3JointSpline & out_path,
  std::vector<Diagnostic> & out_diagnostics) const
{
  const auto & model = engine_.get_pin_model();
  const auto tcp = engine_.tcp_frame_id();
  auto workspace = engine_.collision_workspace();
  auto & data = workspace.data;
  const double S = cart_path.total_length();
  const double sing_thresh = engine_.get_singularity_threshold();

  // Knots always include the piece boundaries, where higher path derivatives may jump.
  std::vector<double> breaks{0.0};
  for (double b : cart_path.piece_boundaries()) {
    breaks.push_back(b);
  }
  breaks.push_back(1.0);

  // Septic Hermite interpolation with exact derivatives converges as O(h^8): start from
  // 5 mm knots and split only the intervals whose residual check fails, caching all
  // previously solved knots across refinement passes.
  constexpr double kKnotStep = 0.005;          // m of pseudo arc length
  constexpr double kResidualPos = 5e-6;        // m
  constexpr double kResidualRot = 5e-5;        // rad
  constexpr size_t kMaxKnots = 8000;

  struct Knot
  {
    double u = 0.0;
    Vec6 q = Vec6::Zero();
    Vec6 m1 = Vec6::Zero();
    Vec6 m2 = Vec6::Zero();
    Vec6 m3 = Vec6::Zero();
    bool solved = false;
    bool m3_valid = false;
  };

  std::vector<Knot> knots;
  for (size_t b = 0; b + 1 < breaks.size(); ++b) {
    const double span = breaks[b + 1] - breaks[b];
    const int n_sub = std::max(2, static_cast<int>(std::ceil(span * S / kKnotStep)));
    for (int k = 0; k < n_sub; ++k) {
      Knot kn;
      kn.u = breaks[b] + span * k / n_sub;
      knots.push_back(kn);
    }
  }
  {
    Knot end_kn;
    end_kn.u = 1.0;
    knots.push_back(end_kn);
  }

  double worst_p = 0.0, worst_r = 0.0, worst_u = 0.0;
  for (int refine = 0; refine < 6; ++refine) {
    if (knots.size() > kMaxKnots) {
      add_diag(
        out_diagnostics, D::STAGE_PLANNING, cart_path.eval_normalized(0.0).loc,
        "ERR_PATH_TOO_LONG", "Cartesian path requires too many IK knots");
      return false;
    }
    const size_t n = knots.size();
    for (size_t k = 0; k < n; ++k) {
      if (knots[k].solved) {
        continue;
      }
      const CartesianSample cs = cart_path.eval_normalized(knots[k].u);
      Vec6 v1, v2;
      cartesian_derivatives(cs, v1, v2);
      if (k == 0) {
        if (!newton_ik(model, data, tcp, cs.pose, to_vec6(start_q), knots[k].q)) {
          add_diag(
            out_diagnostics, D::STAGE_PLANNING, cs.loc, "ERR_NO_IK_SOLUTION",
            "Path start pose is not reachable from the start configuration");
          return false;
        }
      } else {
        Vec6 predicted;
        if (refine > 0) {
          Vec6 dummy1, dummy2, dummy3;
          out_path.eval(knots[k].u, predicted, dummy1, dummy2, dummy3);
        } else {
          const double du = knots[k].u - knots[k - 1].u;
          predicted = knots[k - 1].q + du * (knots[k - 1].m1 + 0.5 * du * knots[k - 1].m2);
        }
        // Try fast Newton refinement from the continuous predictor first on refinement passes;
        // verify limits, collision, and branch continuity before accepting.
        bool got_ik = false;
        if (refine > 0 && newton_ik(model, data, tcp, cs.pose, predicted, knots[k].q)) {
          const auto q_arr = to_array(knots[k].q);
          if (engine_.joint_valid(q_arr) && !engine_.check_collision(q_arr, workspace)) {
            got_ik = true;
          }
        }
        if (!got_ik) {
          std::vector<std::array<double, 6>> sols;
          lite6_planner::IKDiagnostics ik_diag;
          const auto status = engine_.solve_optimal_ik(
            cs.pose, to_array(predicted), sols, /*continuous=*/true, workspace, &ik_diag);
          if (status != IKResult::SUCCESS) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(4)
                << "Cartesian IK failed at path progress " << (100.0 * knots[k].u) << "% (xyz=["
                << cs.pose(0, 3) << ", " << cs.pose(1, 3) << ", " << cs.pose(2, 3)
                << "] m): " << ik_diag.summary();
            add_diag(
              out_diagnostics, D::STAGE_PLANNING, cs.loc, ik_error_code(status), oss.str());
            return false;
          }
          if (!newton_ik(model, data, tcp, cs.pose, to_vec6(sols.front()), knots[k].q)) {
            knots[k].q = to_vec6(sols.front());
          }
        }
        if ((knots[k].q - predicted).cwiseAbs().maxCoeff() >
            0.05 + 0.5 * (knots[k].q - knots[k - 1].q).cwiseAbs().maxCoeff())
        {
          add_diag(
            out_diagnostics, D::STAGE_PLANNING, cs.loc, "ERR_QUADRANT_JUMP",
            "IK branch discontinuity along Cartesian path at progress " +
              std::to_string(100.0 * knots[k].u) + "%");
          return false;
        }
      }
      if (engine_.check_singularity(to_array(knots[k].q), data) < sing_thresh ||
          !differential_kinematics(model, data, tcp, knots[k].q, v1, v2, knots[k].m1, knots[k].m2))
      {
        add_diag(
          out_diagnostics, D::STAGE_KINEMATIC_LIMIT, cs.loc, "ERR_SINGULARITY",
          "Kinematic singularity on Cartesian path at progress " +
            std::to_string(100.0 * knots[k].u) + "%");
        return false;
      }
      knots[k].solved = true;
      knots[k].m3_valid = false;
      if (k > 0) {
        knots[k - 1].m3_valid = false;
      }
      if (k + 1 < n) {
        knots[k + 1].m3_valid = false;
      }
    }

    // d3q/du3 by clean central difference of the exact analytic d2q/du2 (only at new knots
    // and their immediate neighbors whose spacing shrank).
    auto q2_at = [&](size_t k, double uu, Vec6 & out) -> bool {
      const double du = uu - knots[k].u;
      const Vec6 guess = knots[k].q + du * (knots[k].m1 + 0.5 * du * knots[k].m2);
      const CartesianSample cs = cart_path.eval_normalized(uu);
      Vec6 qq, v1, v2, q1;
      cartesian_derivatives(cs, v1, v2);
      return newton_ik(model, data, tcp, cs.pose, guess, qq) &&
             differential_kinematics(model, data, tcp, qq, v1, v2, q1, out);
    };
    for (size_t k = 0; k < n; ++k) {
      if (knots[k].m3_valid) {
        continue;
      }
      const double gap_l = k > 0 ? knots[k].u - knots[k - 1].u : 1.0;
      const double gap_r = k + 1 < n ? knots[k + 1].u - knots[k].u : 1.0;
      const double h = std::max(
        1e-8, std::min({std::max(1e-4 / S, 1e-5), 0.25 * gap_l, 0.25 * gap_r}));
      Vec6 a, b;
      bool ok = false;
      if (k == 0) {
        ok = q2_at(k, knots[k].u + h, a) && q2_at(k, knots[k].u + 2.0 * h, b);
        knots[k].m3 = (-3.0 * knots[k].m2 + 4.0 * a - b) / (2.0 * h);
      } else if (k + 1 == n) {
        ok = q2_at(k, knots[k].u - h, a) && q2_at(k, knots[k].u - 2.0 * h, b);
        knots[k].m3 = (3.0 * knots[k].m2 - 4.0 * a + b) / (2.0 * h);
      } else {
        ok = q2_at(k, knots[k].u + h, a) && q2_at(k, knots[k].u - h, b);
        knots[k].m3 = (a - b) / (2.0 * h);
      }
      if (!ok || !knots[k].m3.allFinite()) {
        add_diag(
          out_diagnostics, D::STAGE_PLANNING, cart_path.eval_normalized(knots[k].u).loc,
          "ERR_SINGULARITY", "Differential kinematics failed near path progress " +
            std::to_string(100.0 * knots[k].u) + "%");
        return false;
      }
      knots[k].m3_valid = true;
    }

    std::vector<double> u_vec(n);
    std::vector<Vec6> q_vec(n), m1_vec(n), m2_vec(n), m3_vec(n);
    for (size_t k = 0; k < n; ++k) {
      u_vec[k] = knots[k].u;
      q_vec[k] = knots[k].q;
      m1_vec[k] = knots[k].m1;
      m2_vec[k] = knots[k].m2;
      m3_vec[k] = knots[k].m3;
    }
    out_path.build_hermite(q_vec, m1_vec, m2_vec, m3_vec, u_vec);

    // Residual of the joint path against the exact Cartesian path between knots.
    std::vector<Knot> refined;
    refined.reserve(n + n / 4);
    refined.push_back(knots.front());
    bool accurate = true;
    worst_p = worst_r = 0.0;
    for (size_t k = 0; k + 1 < n; ++k) {
      bool interval_ok = true;
      for (double frac : {0.25, 0.5, 0.75}) {
        const double um = knots[k].u + frac * (knots[k + 1].u - knots[k].u);
        Vec6 qm, q1, q2, q3;
        out_path.eval(um, qm, q1, q2, q3);
        const Eigen::Matrix4d fk = engine_.forward_pose(to_array(qm), data);
        const CartesianSample ref = cart_path.eval_normalized(um);
        const double p_err = (fk.topRightCorner<3, 1>() - ref.pose.topRightCorner<3, 1>()).norm();
        const double r_err = rot_diff(fk.topLeftCorner<3, 3>(), ref.pose.topLeftCorner<3, 3>());
        if (p_err > kResidualPos || r_err > kResidualRot) {
          interval_ok = false;
          if (p_err / kResidualPos + r_err / kResidualRot > worst_p / kResidualPos + worst_r / kResidualRot) {
            worst_p = p_err;
            worst_r = r_err;
            worst_u = um;
          }
        }
      }
      if (!interval_ok) {
        accurate = false;
        Knot mid_kn;
        mid_kn.u = 0.5 * (knots[k].u + knots[k + 1].u);
        refined.push_back(mid_kn);
      }
      refined.push_back(knots[k + 1]);
    }
    if (accurate) {
      return true;
    }
    knots.swap(refined);
  }
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6)
      << "Joint path deviates from the Cartesian path at " << (100.0 * worst_u)
      << "% (pos_err=" << worst_p << " m, rot_err=" << worst_r << " rad) after knot refinement";
  add_diag(
    out_diagnostics, D::STAGE_PLANNING, cart_path.eval_normalized(worst_u).loc,
    "ERR_SPLINE_RESIDUAL", oss.str());
  return false;
}

bool ProgramCompiler::plan_cartesian_path(
  const CartesianPath & cart_path,
  const std::array<double, 6> & start_q,
  const MotionOptions & default_options,
  const SourceLocation & default_loc,
  bool is_impedance,
  SampledTrajectory1kHz & out_sampled,
  std::vector<Diagnostic> & out_diagnostics) const
{
  C3JointSpline joint_path;
  if (!build_cartesian_joint_path(cart_path, start_q, joint_path, out_diagnostics)) {
    return false;
  }
  std::string topp_err;
  if (!TOPPJerkSolver::solve(
        joint_path, &cart_path, motion_limits(is_impedance), engine_.get_cartesian_limits(),
        default_options.vel_scale, default_options.acc_scale, default_options.jerk_scale,
        default_loc, out_sampled, topp_err))
  {
    add_diag(out_diagnostics, D::STAGE_PLANNING, default_loc, "ERR_TOPP_SOLVER", topp_err);
    return false;
  }
  return verify_sampled_trajectory_1khz(
    out_sampled, /*check_singularity=*/true, out_diagnostics, is_impedance);
}

double ProgramCompiler::tcp_blend_trim(
  const std::array<double, 6> & corner_q,
  const std::array<double, 6> & neighbor_q,
  double blend_radius) const
{
  if (!(blend_radius > 1e-9)) {
    return 0.0;
  }
  auto workspace = engine_.collision_workspace();
  const Vec6 c = to_vec6(corner_q);
  const Vec6 d = to_vec6(neighbor_q) - c;
  const double len = d.norm();
  if (len < 1e-12) {
    return 0.0;
  }
  const Vec6 e = d / len;
  const double d_max = 0.45 * len;

  // Measure strictly monotonic cumulative TCP pseudo-arc-length (combining translation [m]
  // and kCharLength * rotation [m]) along the joint line from corner_q toward neighbor_q.
  // This handles pure-orientation joint moves and >180 deg rotations without non-monotonicity.
  const int steps = std::max(16, static_cast<int>(std::ceil(d_max / 0.04)));
  Eigen::Matrix4d prev_fk = engine_.forward_pose(corner_q, workspace.data);
  double prev_s = 0.0;
  double acc_len = 0.0;

  for (int k = 1; k <= steps; ++k) {
    const double s = d_max * static_cast<double>(k) / steps;
    const Eigen::Matrix4d curr_fk = engine_.forward_pose(to_array(c + s * e), workspace.data);
    const double dp = (curr_fk.topRightCorner<3, 1>() - prev_fk.topRightCorner<3, 1>()).norm();
    const double dr = rot_diff(prev_fk.topLeftCorner<3, 3>(), curr_fk.topLeftCorner<3, 3>());
    const double seg_len = std::max(dp, kCharLength * dr);
    if (acc_len + seg_len >= blend_radius) {
      // Refine within this small monotonic subinterval [prev_s, s].
      const double rem = blend_radius - acc_len;
      double lo = prev_s;
      double hi = s;
      for (int it = 0; it < 24; ++it) {
        const double mid = 0.5 * (lo + hi);
        const Eigen::Matrix4d fk_m = engine_.forward_pose(to_array(c + mid * e), workspace.data);
        const double sub_p = (fk_m.topRightCorner<3, 1>() - prev_fk.topRightCorner<3, 1>()).norm();
        const double sub_r = rot_diff(prev_fk.topLeftCorner<3, 3>(), fk_m.topLeftCorner<3, 3>());
        (std::max(sub_p, kCharLength * sub_r) < rem ? lo : hi) = mid;
      }
      return 0.5 * (lo + hi);
    }
    acc_len += seg_len;
    prev_fk = curr_fk;
    prev_s = s;
  }
  return d_max;
}

bool ProgramCompiler::build_joint_group_path(
  const std::vector<MotionStatement> & group,
  const std::array<double, 6> & start_q,
  std::vector<std::array<double, 6>> & out_waypoints,
  JointCompositePath & out_path,
  std::vector<Diagnostic> & out_diagnostics) const
{
  const auto & limits = engine_.get_joint_limits();
  auto workspace = engine_.collision_workspace();
  std::vector<std::array<double, 6>> raw_waypoints;
  raw_waypoints.reserve(group.size() + 1);
  raw_waypoints.push_back(start_q);

  for (const auto & st : group) {
    if (st.type == MotionCommandType::MOVE_J) {
      for (size_t i = 0; i < 6; ++i) {
        if (st.joint_target[i] < limits[i].min_pos - lite6_planner::tolerance::kJointPosition ||
            st.joint_target[i] > limits[i].max_pos + lite6_planner::tolerance::kJointPosition) {
          std::ostringstream oss;
          oss << std::fixed << std::setprecision(5)
              << "MoveJ target violates joint" << (i + 1) << " position limit: value="
              << st.joint_target[i] << " rad, allowed range=[" << limits[i].min_pos << ", "
              << limits[i].max_pos << "] rad";
          add_diag(
            out_diagnostics, D::STAGE_KINEMATIC_LIMIT, st.loc, "ERR_JOINT_POS_LIMIT", oss.str());
          return false;
        }
      }
      std::string col_detail;
      if (engine_.check_collision(st.joint_target, workspace, &col_detail)) {
        add_diag(
          out_diagnostics, D::STAGE_COLLISION, st.loc, "ERR_COLLISION",
          "MoveJ target configuration is in collision: " + col_detail);
        return false;
      }
      raw_waypoints.push_back(st.joint_target);
    } else {
      // MoveP: resolve Cartesian target via optimal IK relative to previous waypoint,
      // trying alternative IK branches in cost order if the closest branch collides along
      // the straight joint-space route.
      std::vector<std::array<double, 6>> sols;
      lite6_planner::IKDiagnostics ik_diag;
      const auto status = engine_.solve_optimal_ik(
        st.pose_target, raw_waypoints.back(), sols, /*continuous=*/false, workspace, &ik_diag);
      if (status != IKResult::SUCCESS) {
        add_diag(
          out_diagnostics, D::STAGE_PLANNING, st.loc, ik_error_code(status),
          "MoveP target IK failed: " + ik_diag.summary());
        return false;
      }
      std::array<double, 6> chosen = sols.front();
      for (const auto & cand : sols) {
        if (segment_collision_free(engine_, workspace, raw_waypoints.back(), cand)) {
          chosen = cand;
          break;
        }
      }
      raw_waypoints.push_back(chosen);
    }
  }

  // Filter redundant zero-length segments inside multi-statement blended groups.
  out_waypoints.clear();
  out_waypoints.push_back(raw_waypoints.front());
  std::vector<MotionOptions> opts;
  std::vector<SourceLocation> locs;
  for (size_t i = 0; i < group.size(); ++i) {
    const Vec6 d = to_vec6(raw_waypoints[i + 1]) - to_vec6(out_waypoints.back());
    if (group.size() > 1 && d.norm() < 1e-9) {
      if (!opts.empty()) {
        opts.back().blend_radius = (i + 1 < group.size()) ?
          std::min(opts.back().blend_radius, group[i].options.blend_radius) :
          0.0;
      }
      continue;
    }
    out_waypoints.push_back(raw_waypoints[i + 1]);
    opts.push_back(group[i].options);
    locs.push_back(group[i].loc);
  }
  if (out_waypoints.size() == 1) {
    // All statements in the group were zero-length: preserve one stationary segment.
    out_waypoints.push_back(raw_waypoints.back());
    opts.push_back(group.back().options);
    locs.push_back(group.back().loc);
  }
  if (out_waypoints.size() <= 2) {
    return true;
  }

  // Explicit corner blends sized by the TCP blend radius of the statement ending there.
  std::vector<JointCompositePath::Corner> corners;
  for (size_t k = 1; k + 1 < out_waypoints.size(); ++k) {
    const double r = opts[k - 1].blend_radius;
    JointCompositePath::Corner c;
    c.trim_in = tcp_blend_trim(out_waypoints[k], out_waypoints[k - 1], r);
    c.trim_out = tcp_blend_trim(out_waypoints[k], out_waypoints[k + 1], r);
    corners.push_back(c);
  }
  std::string err;
  if (!out_path.build(out_waypoints, opts, locs, corners, err)) {
    add_diag(out_diagnostics, D::STAGE_PLANNING, group.front().loc, "ERR_JOINT_PATH", err);
    return false;
  }
  return true;
}

bool ProgramCompiler::compile_joint_group(
  const std::vector<MotionStatement> & group,
  const std::array<double, 6> & start_q,
  ExecutionBlock & out_block,
  std::vector<Diagnostic> & out_diagnostics) const
{
  std::vector<std::array<double, 6>> waypoints;
  JointCompositePath path;
  if (!build_joint_group_path(group, start_q, waypoints, path, out_diagnostics)) {
    return false;
  }

  SampledTrajectory1kHz sampled;
  if (waypoints.size() <= 2) {
    // Single PTP motion (or collapsed single segment): phase-synchronized time-optimal S-curve.
    const auto & st = group.front();
    MultiAxisSCurve scurve;
    scurve.build(
      start_q, waypoints.back(), engine_.get_joint_limits(),
      st.options.vel_scale, st.options.acc_scale, st.options.jerk_scale);
    sampled = scurve_samples(scurve, st.loc, 0.0, 1.0);
  } else {
    std::string topp_err;
    if (!TOPPJerkSolver::solve(
          path, nullptr, engine_.get_joint_limits(), engine_.get_cartesian_limits(),
          group.front().options.vel_scale, group.front().options.acc_scale,
          group.front().options.jerk_scale, group.front().loc, sampled, topp_err))
    {
      add_diag(
        out_diagnostics, D::STAGE_PLANNING, group.front().loc, "ERR_TOPP_SOLVER", topp_err);
      return false;
    }
  }

  if (!verify_sampled_trajectory_1khz(sampled, /*check_singularity=*/false, out_diagnostics)) {
    return false;
  }

  out_block = ExecutionBlock();
  out_block.source_kind = BlockSourceKind::JOINT_GROUP;
  out_block.source_motions = group;
  out_block.path_start_joints = start_q;
  populate_execution_block(sampled, false, ImpedanceParams{}, out_block);
  return true;
}

bool ProgramCompiler::compile_pva_statement(
  const MovePVAStatement & pva,
  const std::array<double, 6> & start_q,
  bool is_impedance,
  const ImpedanceParams & imp_params,
  ExecutionBlock & out_block,
  std::vector<Diagnostic> & out_diagnostics) const
{
  const auto limits = motion_limits(is_impedance);
  const auto & cart_limits = engine_.get_cartesian_limits();
  const double sing_thresh = engine_.get_singularity_threshold();
  const auto & model = engine_.get_pin_model();
  const auto tcp = engine_.tcp_frame_id();
  auto workspace = engine_.collision_workspace();
  const size_t M = pva.waypoints.size();

  // Warn whenever the user supplies non-zero terminal velocity or acceleration on the final
  // MovePVA waypoint, since controller settling requires ending at rest (v=0, a=0).
  {
    const auto & last_wp = pva.waypoints.back();
    bool last_nonzero_va = false;
    if (last_wp.velocity.has_value()) {
      for (double v : *last_wp.velocity) {
        if (std::abs(v) > 1e-9) {
          last_nonzero_va = true;
          break;
        }
      }
    }
    if (!last_nonzero_va && last_wp.acceleration.has_value()) {
      for (double a : *last_wp.acceleration) {
        if (std::abs(a) > 1e-9) {
          last_nonzero_va = true;
          break;
        }
      }
    }
    if (last_nonzero_va) {
      add_warning(
        out_diagnostics, D::STAGE_SEMANTIC, last_wp.loc, "WARN_PVA_FINAL_NONZERO_VA",
        "Final MovePVA waypoint must terminate at rest (v=0, a=0) for safe controller settling; "
        "non-zero terminal velocity/acceleration was clamped to zero");
    }
  }

  bool any_vel_or_acc_or_dt = false;
  for (size_t k = 0; k < M; ++k) {
    const auto & wp = pva.waypoints[k];
    if (wp.velocity.has_value() || wp.acceleration.has_value() || wp.dt.has_value()) {
      any_vel_or_acc_or_dt = true;
    }
  }

  SampledTrajectory1kHz sampled;

  if (pva.space == SpaceType::CARTESIAN) {
    // Pure (P) Cartesian MovePVA: interpolate directly in SE(3) via C3CartesianSpline and
    // run 3rd-order TOPP with full Cartesian and joint kinematic limits.
    if (!any_vel_or_acc_or_dt) {
      std::vector<Eigen::Matrix4d> poses;
      std::vector<SourceLocation> seg_locs;
      poses.reserve(M + 1);
      seg_locs.reserve(M);
      poses.push_back(engine_.forward_pose(start_q, workspace.data));
      for (const auto & wp : pva.waypoints) {
        poses.push_back(wp.cart_pose);
        seg_locs.push_back(wp.loc);
      }
      C3CartesianSpline cart_spline;
      std::string spline_err;
      if (!cart_spline.build(poses, seg_locs, pva.options, spline_err)) {
        add_diag(out_diagnostics, D::STAGE_PLANNING, pva.loc, "ERR_PVA_CARTESIAN_SPLINE", spline_err);
        return false;
      }
      if (!plan_cartesian_path(
            cart_spline, start_q, pva.options, pva.loc, is_impedance, sampled, out_diagnostics))
      {
        return false;
      }
      out_block = ExecutionBlock();
      out_block.source_kind = BlockSourceKind::MOVE_PVA;
      out_block.source_pva = pva;
      out_block.cartesian_path = std::make_shared<C3CartesianSpline>(std::move(cart_spline));
      out_block.path_start_joints = start_q;
      populate_execution_block(sampled, is_impedance, imp_params, out_block);
      return true;
    }

    // Explicit (P, V), (P, V, A), or dt Cartesian MovePVA: interpolate directly in SE(3)
    // using CartesianHermiteSegment, enforcing both scaled Cartesian and scaled joint limits
    // plus normalized SVD singularity checks at every waypoint and 1 kHz sample.
    const double vs = pva.options.vel_scale;
    const double as = pva.options.acc_scale;
    const double js = pva.options.jerk_scale;
    const double v_trans_lim = cart_limits.max_trans_vel * vs;
    const double v_rot_lim = cart_limits.max_rot_vel * vs;
    const double a_trans_lim = cart_limits.max_trans_acc * as;
    const double a_rot_lim = cart_limits.max_rot_acc * as;

    std::vector<Eigen::Matrix4d> T_nodes(M + 1);
    std::vector<std::array<double, 6>> q_nodes(M + 1);
    std::vector<Vec6> V_nodes(M + 1, Vec6::Zero());
    std::vector<Vec6> A_nodes(M + 1, Vec6::Zero());
    std::vector<std::array<double, 6>> dq_nodes(M + 1, std::array<double, 6>{});
    std::vector<std::array<double, 6>> ddq_nodes(M + 1, std::array<double, 6>{});

    q_nodes[0] = start_q;
    T_nodes[0] = engine_.forward_pose(start_q, workspace.data);

    for (size_t k = 0; k < M; ++k) {
      const auto & wp = pva.waypoints[k];
      T_nodes[k + 1] = wp.cart_pose;
      std::vector<std::array<double, 6>> sols;
      lite6_planner::IKDiagnostics ik_diag;
      // Sparse MovePVA waypoints may span the full workspace (well beyond the per-step
      // quadrant_jump_threshold); branch continuity is enforced densely along each
      // interpolated segment inside eval_cart_segment().
      const auto status = engine_.solve_optimal_ik(
        wp.cart_pose, q_nodes[k], sols, /*continuous=*/false, workspace, &ik_diag);
      if (status != IKResult::SUCCESS) {
        add_diag(
          out_diagnostics, D::STAGE_PLANNING, wp.loc, ik_error_code(status),
          "MovePVA Cartesian waypoint " + std::to_string(k + 1) + " IK failed: " +
            ik_diag.summary());
        return false;
      }
      Vec6 q_ref;
      if (newton_ik(model, workspace.data, tcp, wp.cart_pose, to_vec6(sols.front()), q_ref)) {
        q_nodes[k + 1] = to_array(q_ref);
      } else {
        q_nodes[k + 1] = sols.front();
      }
      if (k + 1 < M && wp.velocity.has_value()) {
        V_nodes[k + 1] = to_vec6(*wp.velocity);
        if (wp.acceleration.has_value()) {
          A_nodes[k + 1] = to_vec6(*wp.acceleration);
        }
        // Check waypoint Cartesian velocity/acceleration against scaled Cartesian limits.
        if (V_nodes[k + 1].head<3>().norm() > v_trans_lim * (1.0 + 1e-6)) {
          std::ostringstream oss;
          oss << std::fixed << std::setprecision(4)
              << "MovePVA waypoint " << (k + 1) << " translational speed="
              << V_nodes[k + 1].head<3>().norm() << " m/s exceeds scaled limit="
              << v_trans_lim << " m/s";
          add_diag(out_diagnostics, D::STAGE_KINEMATIC_LIMIT, wp.loc, "ERR_PVA_CART_VEL_LIMIT", oss.str());
          return false;
        }
        if (V_nodes[k + 1].tail<3>().norm() > v_rot_lim * (1.0 + 1e-6)) {
          std::ostringstream oss;
          oss << std::fixed << std::setprecision(4)
              << "MovePVA waypoint " << (k + 1) << " rotational speed="
              << V_nodes[k + 1].tail<3>().norm() << " rad/s exceeds scaled limit="
              << v_rot_lim << " rad/s";
          add_diag(out_diagnostics, D::STAGE_KINEMATIC_LIMIT, wp.loc, "ERR_PVA_CART_VEL_LIMIT", oss.str());
          return false;
        }
        if (A_nodes[k + 1].head<3>().norm() > a_trans_lim * (1.0 + 1e-6)) {
          std::ostringstream oss;
          oss << std::fixed << std::setprecision(4)
              << "MovePVA waypoint " << (k + 1) << " translational acceleration="
              << A_nodes[k + 1].head<3>().norm() << " m/s^2 exceeds scaled limit="
              << a_trans_lim << " m/s^2";
          add_diag(out_diagnostics, D::STAGE_KINEMATIC_LIMIT, wp.loc, "ERR_PVA_CART_ACC_LIMIT", oss.str());
          return false;
        }
        if (A_nodes[k + 1].tail<3>().norm() > a_rot_lim * (1.0 + 1e-6)) {
          std::ostringstream oss;
          oss << std::fixed << std::setprecision(4)
              << "MovePVA waypoint " << (k + 1) << " rotational acceleration="
              << A_nodes[k + 1].tail<3>().norm() << " rad/s^2 exceeds scaled limit="
              << a_rot_lim << " rad/s^2";
          add_diag(out_diagnostics, D::STAGE_KINEMATIC_LIMIT, wp.loc, "ERR_PVA_CART_ACC_LIMIT", oss.str());
          return false;
        }
        // Validate normalized SVD Jacobian and check singularity at waypoint (always compute
        // joint acceleration including -J^-1 dJ dq even when Cartesian acceleration is zero).
        double sigma_min = 0.0;
        const std::array<double, 6> vel_arr = to_array(V_nodes[k + 1]);
        const std::array<double, 6> acc_arr = to_array(A_nodes[k + 1]);
        if (!cartesian_twist_to_joint(
              model, workspace.data, tcp, q_nodes[k + 1], vel_arr, &acc_arr,
              sing_thresh, dq_nodes[k + 1], &ddq_nodes[k + 1], &sigma_min))
        {
          std::ostringstream oss;
          oss << std::fixed << std::setprecision(6)
              << "MovePVA Cartesian waypoint " << (k + 1)
              << " is near singularity (normalized Jacobian sigma_min=" << sigma_min
              << " < singularity_threshold=" << sing_thresh << ")";
          add_diag(out_diagnostics, D::STAGE_KINEMATIC_LIMIT, wp.loc, "ERR_SINGULARITY", oss.str());
          return false;
        }
        for (size_t j = 0; j < 6; ++j) {
          if (std::abs(dq_nodes[k + 1][j]) > limits[j].max_vel * vs * (1.0 + 1e-6)) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(5)
                << "MovePVA Cartesian waypoint " << (k + 1) << " twist requires joint" << (j + 1)
                << " velocity=" << std::abs(dq_nodes[k + 1][j]) << " rad/s > scaled limit="
                << (limits[j].max_vel * vs) << " rad/s";
            add_diag(out_diagnostics, D::STAGE_KINEMATIC_LIMIT, wp.loc, "ERR_PVA_VEL_LIMIT", oss.str());
            return false;
          }
          if (std::abs(ddq_nodes[k + 1][j]) > limits[j].max_acc * as * (1.0 + 1e-6)) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(5)
                << "MovePVA Cartesian waypoint " << (k + 1) << " kinematics require joint"
                << (j + 1) << " accel=" << std::abs(ddq_nodes[k + 1][j])
                << " rad/s^2 > scaled limit=" << (limits[j].max_acc * as) << " rad/s^2";
            add_diag(out_diagnostics, D::STAGE_KINEMATIC_LIMIT, wp.loc, "ERR_PVA_ACC_LIMIT", oss.str());
            return false;
          }
        }
      }
    }

    std::vector<CartesianHermiteSegment> retained_segments;
    std::vector<SourceLocation> retained_locations;
    double t_accum = 0.0;
    std::array<double, 6> running_q = start_q;
    for (size_t seg_idx = 0; seg_idx < M; ++seg_idx) {
      const auto & wp = pva.waypoints[seg_idx];
      const Eigen::Matrix4d & T0 = T_nodes[seg_idx];
      const Eigen::Matrix4d & T1 = T_nodes[seg_idx + 1];
      const Vec6 & V0 = V_nodes[seg_idx];
      const Vec6 & A0 = A_nodes[seg_idx];
      const Vec6 & V1 = V_nodes[seg_idx + 1];
      const Vec6 & A1 = A_nodes[seg_idx + 1];

      // Evaluates a CartesianHermiteSegment of duration T_cand; when full_1khz is false,
      // checks Cartesian bounds first (fast rejection) then a coarse IK/joint sweep.
      // When full_1khz is true, emits all 1 kHz samples and verifies every scaled limit.
      auto eval_cart_segment = [&](
        double T_cand,
        bool full_1khz,
        double safety,
        std::vector<SampledTrajectory1kHz::Sample> * out_seg,
        double & out_ratio,
        std::string & out_code,
        std::string & out_msg) -> bool
      {
        out_ratio = 0.0;
        CartesianHermiteSegment seg;
        if (!seg.build_with_duration(T0, V0, A0, T1, V1, A1, T_cand)) {
          out_ratio = std::numeric_limits<double>::infinity();
          out_code = "ERR_PVA_ORIENTATION";
          out_msg = "MovePVA Cartesian segment orientation change is too close to 180 deg";
          return false;
        }
        const int ticks = std::max(2, static_cast<int>(std::llround(seg.duration() * 1000.0)));
        const int steps = full_1khz ? ticks : std::clamp(ticks, 24, 64);
        const double dt_step = seg.duration() / static_cast<double>(steps);

        // Allow exact boundary speeds/accels that lie in (safety, 1.0] of the scaled limit.
        const double vt_chk = std::max(
          v_trans_lim * safety,
          std::min(v_trans_lim, std::max(V0.head<3>().norm(), V1.head<3>().norm()))) * (1.0 + 1e-6);
        const double vr_chk = std::max(
          v_rot_lim * safety,
          std::min(v_rot_lim, std::max(V0.tail<3>().norm(), V1.tail<3>().norm()))) * (1.0 + 1e-6);
        const double at_chk = std::max(
          a_trans_lim * safety,
          std::min(a_trans_lim, std::max(A0.head<3>().norm(), A1.head<3>().norm()))) * (1.0 + 1e-6);
        const double ar_chk = std::max(
          a_rot_lim * safety,
          std::min(a_rot_lim, std::max(A0.tail<3>().norm(), A1.tail<3>().norm()))) * (1.0 + 1e-6);

        // Phase 1: fast pure-Cartesian limit sweep.
        if (!full_1khz) {
          for (int k = 0; k <= steps; ++k) {
            Eigen::Matrix4d pose;
            Vec6 vel, acc;
            Eigen::Vector3d pj;
            seg.eval(k * dt_step, pose, vel, acc, pj);
            out_ratio = std::max({
              out_ratio,
              vel.head<3>().norm() / vt_chk,
              vel.tail<3>().norm() / vr_chk,
              acc.head<3>().norm() / at_chk,
              acc.tail<3>().norm() / ar_chk});
          }
          if (out_ratio > 1.0) {
            return false;
          }
        }

        // Phase 2: continuous IK + differential kinematics + joint & Cartesian verification.
        if (out_seg != nullptr) {
          out_seg->clear();
          out_seg->reserve(static_cast<size_t>(ticks + 1));
        }
        Vec6 q_prev = to_vec6(running_q);
        Vec6 dq_prev = to_vec6(dq_nodes[seg_idx]);
        Vec6 ddq_prev = to_vec6(ddq_nodes[seg_idx]);
        bool ok = true;

        for (int k = 0; k <= steps; ++k) {
          const double t_loc = full_1khz ? (static_cast<double>(k) * 0.001) : (k * dt_step);
          Eigen::Matrix4d pose;
          Vec6 vel, acc;
          Eigen::Vector3d pj;
          seg.eval(t_loc, pose, vel, acc, pj);
          if (seg_idx + 1 == M && k == steps) {
            vel.setZero();
            acc.setZero();
          }

          const double r_vt = vel.head<3>().norm() / vt_chk;
          const double r_vr = vel.tail<3>().norm() / vr_chk;
          const double r_at = acc.head<3>().norm() / at_chk;
          const double r_ar = acc.tail<3>().norm() / ar_chk;
          out_ratio = std::max({out_ratio, r_vt, r_vr, r_at, r_ar});
          if (full_1khz && (r_vt > 1.0 || r_vr > 1.0)) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(4)
                << "MovePVA Cartesian segment " << (seg_idx + 1) << " exceeds scaled Cartesian "
                << "velocity limit at t=" << (t_accum + t_loc) << " s (trans="
                << vel.head<3>().norm() << "/" << v_trans_lim << " m/s, rot="
                << vel.tail<3>().norm() << "/" << v_rot_lim << " rad/s)";
            out_code = "ERR_PVA_CART_VEL_LIMIT";
            out_msg = oss.str();
            return false;
          }
          if (full_1khz && (r_at > 1.0 || r_ar > 1.0)) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(4)
                << "MovePVA Cartesian segment " << (seg_idx + 1) << " exceeds scaled Cartesian "
                << "acceleration limit at t=" << (t_accum + t_loc) << " s (trans="
                << acc.head<3>().norm() << "/" << a_trans_lim << " m/s^2, rot="
                << acc.tail<3>().norm() << "/" << a_rot_lim << " rad/s^2)";
            out_code = "ERR_PVA_CART_ACC_LIMIT";
            out_msg = oss.str();
            return false;
          }

          Vec6 q_curr;
          if (k == 0) {
            if (!newton_ik(model, workspace.data, tcp, pose, q_prev, q_curr)) {
              q_curr = q_prev;
            }
          } else {
            const Vec6 guess = q_prev + dt_step * (dq_prev + 0.5 * dt_step * ddq_prev);
            if (!newton_ik(model, workspace.data, tcp, pose, guess, q_curr)) {
              std::vector<std::array<double, 6>> sols;
              const auto st_ik = engine_.solve_optimal_ik(
                pose, to_array(guess), sols, /*continuous=*/true, workspace);
              if (st_ik != IKResult::SUCCESS) {
                out_ratio = std::numeric_limits<double>::infinity();
                out_code = ik_error_code(st_ik);
                out_msg = "MovePVA Cartesian segment " + std::to_string(seg_idx + 1) +
                  " IK failed at t=" + std::to_string(t_accum + t_loc) + " s";
                return false;
              }
              if (!newton_ik(model, workspace.data, tcp, pose, to_vec6(sols.front()), q_curr)) {
                q_curr = to_vec6(sols.front());
              }
            }
            if ((q_curr - guess).cwiseAbs().maxCoeff() >
                0.05 + 0.5 * (q_curr - q_prev).cwiseAbs().maxCoeff())
            {
              out_ratio = std::numeric_limits<double>::infinity();
              out_code = "ERR_QUADRANT_JUMP";
              out_msg = "IK branch jump along MovePVA Cartesian segment " +
                std::to_string(seg_idx + 1);
              return false;
            }
          }

          const double sigma = engine_.check_singularity(to_array(q_curr), workspace.data);
          Vec6 dq_curr = Vec6::Zero(), ddq_curr = Vec6::Zero();
          if (sigma < sing_thresh ||
              !differential_kinematics(model, workspace.data, tcp, q_curr, vel, acc, dq_curr, ddq_curr))
          {
            out_ratio = std::numeric_limits<double>::infinity();
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(6)
                << "Kinematic singularity in MovePVA Cartesian segment " << (seg_idx + 1)
                << " at t=" << (t_accum + t_loc) << " s (sigma_min=" << sigma
                << " < singularity_threshold=" << sing_thresh << ")";
            out_code = "ERR_SINGULARITY";
            out_msg = oss.str();
            return false;
          }

          for (size_t j = 0; j < 6; ++j) {
            const int jj = static_cast<int>(j);
            const double pos_exc = std::max({
              0.0,
              limits[j].min_pos - q_curr[jj],
              q_curr[jj] - limits[j].max_pos});
            if (pos_exc > lite6_planner::tolerance::kTrajectoryPosition) {
              out_ratio = std::max(out_ratio, 1.0 + 100.0 * pos_exc);
              if (full_1khz) {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(5)
                    << "MovePVA Cartesian segment " << (seg_idx + 1) << " violates joint"
                    << (j + 1) << " position limit=" << q_curr[jj] << " rad";
                out_code = "ERR_JOINT_POS_LIMIT";
                out_msg = oss.str();
                return false;
              }
              ok = false;
            }
            const double jv_lim = limits[j].max_vel * vs;
            const double ja_lim = limits[j].max_acc * as;
            const double jj_lim = limits[j].max_jerk * js;
            const double jv_chk = std::max(
              jv_lim * safety,
              std::min(jv_lim, std::max(std::abs(dq_nodes[seg_idx][j]), std::abs(dq_nodes[seg_idx + 1][j]))));
            const double ja_chk = std::max(
              ja_lim * safety,
              std::min(ja_lim, std::max(std::abs(ddq_nodes[seg_idx][j]), std::abs(ddq_nodes[seg_idx + 1][j]))));
            const double rv_j = std::abs(dq_curr[jj]) / (jv_chk * (1.0 + 1e-6));
            const double ra_j = std::abs(ddq_curr[jj]) / (ja_chk * (1.0 + 1e-6));
            out_ratio = std::max({out_ratio, rv_j, ra_j});
            if (full_1khz && std::abs(dq_curr[jj]) > jv_lim + lite6_planner::tolerance::kTrajectoryVelocity) {
              std::ostringstream oss;
              oss << std::fixed << std::setprecision(5)
                  << "MovePVA Cartesian segment " << (seg_idx + 1) << " joint" << (j + 1)
                  << " |dq|=" << std::abs(dq_curr[jj]) << " rad/s exceeds scaled limit="
                  << jv_lim << " rad/s";
              out_code = "ERR_PVA_VEL_LIMIT";
              out_msg = oss.str();
              return false;
            }
            if (full_1khz && std::abs(ddq_curr[jj]) > ja_lim + lite6_planner::tolerance::kTrajectoryAcceleration) {
              std::ostringstream oss;
              oss << std::fixed << std::setprecision(5)
                  << "MovePVA Cartesian segment " << (seg_idx + 1) << " joint" << (j + 1)
                  << " |ddq|=" << std::abs(ddq_curr[jj]) << " rad/s^2 exceeds scaled limit="
                  << ja_lim << " rad/s^2";
              out_code = "ERR_PVA_ACC_LIMIT";
              out_msg = oss.str();
              return false;
            }
            if (k > 0) {
              const double disc_jrk = std::abs(ddq_curr[jj] - ddq_prev[jj]) / dt_step;
              const double rj_j = disc_jrk / (jj_lim * safety);
              out_ratio = std::max(out_ratio, rj_j);
              if (full_1khz && disc_jrk > jj_lim * 1.001 + 1e-3) {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(3)
                    << "MovePVA Cartesian segment " << (seg_idx + 1) << " joint" << (j + 1)
                    << " |jerk|=" << disc_jrk << " rad/s^3 exceeds scaled limit="
                    << jj_lim << " rad/s^3";
                out_code = "ERR_PVA_JERK_LIMIT";
                out_msg = oss.str();
                return false;
              }
            }
          }

          if (out_seg != nullptr) {
            SampledTrajectory1kHz::Sample smp;
            smp.time_sec = t_accum + t_loc;
            smp.path_u = (static_cast<double>(seg_idx) +
                          static_cast<double>(k) / ticks) / static_cast<double>(M);
            smp.q = to_array(q_curr);
            smp.dq = (seg_idx + 1 == M && k == ticks) ? std::array<double, 6>{} : to_array(dq_curr);
            smp.ddq = (seg_idx + 1 == M && k == ticks) ? std::array<double, 6>{} : to_array(ddq_curr);
            smp.loc = wp.loc;
            out_seg->push_back(smp);
          }

          q_prev = q_curr;
          dq_prev = dq_curr;
          ddq_prev = ddq_curr;
        }
        return ok && out_ratio <= 1.0;
      };

      std::vector<SampledTrajectory1kHz::Sample> seg_samples;
      if (wp.dt.has_value()) {
        double ratio = 0.0;
        std::string err_code, err_msg;
        if (!eval_cart_segment(*wp.dt, /*full_1khz=*/true, 1.0, &seg_samples, ratio, err_code, err_msg)) {
          const uint8_t stage = (err_code == "ERR_PVA_ORIENTATION" || err_code == "ERR_QUADRANT_JUMP" ||
                                 err_code == "ERR_NO_IK_SOLUTION") ?
            D::STAGE_PLANNING : D::STAGE_KINEMATIC_LIMIT;
          add_diag(out_diagnostics, stage, wp.loc, err_code, err_msg);
          return false;
        }
      } else {
        // Non-monotonic minimum-time search in [t_min, 60 s] for Cartesian Hermite segment.
        constexpr double kMaxSegDuration = 60.0;
        const double dp = (T1.topRightCorner<3, 1>() - T0.topRightCorner<3, 1>()).norm();
        const double dr = rot_diff(T0.topLeftCorner<3, 3>(), T1.topLeftCorner<3, 3>());
        double t_min = std::max({
          0.002,
          dp / v_trans_lim,
          dr / v_rot_lim,
          (V1.head<3>() - V0.head<3>()).norm() / a_trans_lim,
          (V1.tail<3>() - V0.tail<3>()).norm() / a_rot_lim});
        bool all_rest = (V0.norm() < 1e-9 && V1.norm() < 1e-9 && A0.norm() < 1e-9 && A1.norm() < 1e-9);
        for (size_t j = 0; j < 6; ++j) {
          t_min = std::max({
            t_min,
            std::abs(q_nodes[seg_idx + 1][j] - running_q[j]) / (limits[j].max_vel * vs),
            std::abs(dq_nodes[seg_idx + 1][j] - dq_nodes[seg_idx][j]) / (limits[j].max_acc * as)});
        }
        if (all_rest) {
          t_min = std::max({t_min, std::sqrt(dp / a_trans_lim), std::sqrt(dr / a_rot_lim)});
        }
        std::vector<double> cands;
        for (double t = t_min; t < kMaxSegDuration; t *= 1.03) {
          cands.push_back(t);
        }
        cands.push_back(kMaxSegDuration);
        const Eigen::Vector3d v_avg = 0.5 * (V0.head<3>() + V1.head<3>());
        const Eigen::Vector3d dpos = T1.topRightCorner<3, 1>() - T0.topRightCorner<3, 1>();
        if (v_avg.dot(dpos) > 1e-12) {
          const double ts = dpos.squaredNorm() / v_avg.dot(dpos);
          if (ts >= t_min && ts <= kMaxSegDuration) {
            cands.push_back(ts);
          }
        }
        std::sort(cands.begin(), cands.end());
        cands.erase(
          std::unique(cands.begin(), cands.end(), [](double a, double b) {
            return std::abs(a - b) <= 1e-6 * std::max(1.0, b);
          }),
          cands.end());

        auto try_accept = [&](double lo, double hi) -> bool {
          double r_tmp = 0.0;
          std::string ec, em;
          for (int it = 0; it < 20; ++it) {
            const double mid = 0.5 * (lo + hi);
            if (eval_cart_segment(mid, false, 0.985, nullptr, r_tmp, ec, em)) {
              hi = mid;
            } else {
              lo = mid;
            }
          }
          const double t_ms = std::max(0.002, std::ceil(hi * 1000.0 - 1e-9) * 0.001);
          for (int step = 0; step <= 8; ++step) {
            const double t_try = t_ms + step * 0.001;
            if (t_try <= kMaxSegDuration &&
                eval_cart_segment(t_try, true, 1.0, &seg_samples, r_tmp, ec, em))
            {
              return true;
            }
          }
          return false;
        };

        bool found = false;
        std::vector<double> ratios(cands.size(), std::numeric_limits<double>::infinity());
        std::string last_code, last_msg;
        for (size_t idx = 0; idx < cands.size() && !found; ++idx) {
          std::string ec, em;
          const bool ok_c = eval_cart_segment(cands[idx], false, 0.985, nullptr, ratios[idx], ec, em);
          if (!ec.empty()) {
            last_code = ec;
            last_msg = em;
          }
          if (ok_c) {
            const double lo = (idx > 0) ? cands[idx - 1] : t_min;
            found = try_accept(lo, cands[idx]);
          } else if (idx >= 2 && ratios[idx - 1] < ratios[idx - 2] && ratios[idx - 1] < ratios[idx]) {
            double a = cands[idx - 2], b = cands[idx];
            constexpr double kPhi = 0.6180339887498949;
            for (int gs = 0; gs < 12 && !found; ++gs) {
              const double m1 = b - kPhi * (b - a);
              const double m2 = a + kPhi * (b - a);
              double r1 = 0.0, r2 = 0.0;
              if (eval_cart_segment(m1, false, 0.985, nullptr, r1, ec, em)) {
                found = try_accept(a, m1);
                break;
              }
              if (eval_cart_segment(m2, false, 0.985, nullptr, r2, ec, em)) {
                found = try_accept(m1, m2);
                break;
              }
              if (r1 < r2) {
                b = m2;
              } else {
                a = m1;
              }
            }
          }
        }
        if (!found) {
          add_diag(
            out_diagnostics, D::STAGE_PLANNING, wp.loc, "ERR_PVA_INFEASIBLE",
            "MovePVA Cartesian segment " + std::to_string(seg_idx + 1) +
              " has no feasible duration within 60 s satisfying scaled Cartesian/joint limits" +
              (last_msg.empty() ? "" : (" (" + last_msg + ")")));
          return false;
        }
      }

      CartesianHermiteSegment retained;
      if (!retained.build_with_duration(
            T0, V0, A0, T1, V1, A1,
            seg_samples.back().time_sec - seg_samples.front().time_sec))
      {
        add_diag(out_diagnostics, D::STAGE_PLANNING, wp.loc,
          "ERR_PVA_GEOMETRY", "Cannot retain accepted Cartesian PVA geometry");
        return false;
      }
      retained_segments.push_back(std::move(retained));
      retained_locations.push_back(wp.loc);
      const size_t start_k = (seg_idx == 0) ? 0 : 1;
      for (size_t k = start_k; k < seg_samples.size(); ++k) {
        sampled.samples.push_back(seg_samples[k]);
      }
      running_q = seg_samples.back().q;
      q_nodes[seg_idx + 1] = running_q;
      dq_nodes[seg_idx + 1] = seg_samples.back().dq;
      ddq_nodes[seg_idx + 1] = seg_samples.back().ddq;
      t_accum = seg_samples.back().time_sec;
    }
    sampled.duration_sec = t_accum;
    for (auto & sample : sampled.samples) {
      sample.path_u = sample.time_sec / t_accum;
    }

    if (!verify_sampled_trajectory_1khz(
          sampled, /*check_singularity=*/true, out_diagnostics, is_impedance))
    {
      return false;
    }
    out_block = ExecutionBlock();
    out_block.source_kind = BlockSourceKind::MOVE_PVA;
    out_block.source_pva = pva;
    out_block.cartesian_path = std::make_shared<HermiteCartesianPath>(
      std::move(retained_segments), std::move(retained_locations), pva.options);
    out_block.path_start_joints = start_q;
    populate_execution_block(sampled, is_impedance, imp_params, out_block);
    return true;
  }

  // SpaceType::JOINT MovePVA
  std::vector<std::array<double, 6>> q_nodes(M + 1);
  q_nodes[0] = start_q;
  for (size_t k = 0; k < M; ++k) {
    const auto & wp = pva.waypoints[k];
    for (size_t i = 0; i < 6; ++i) {
      if (wp.joint_pos[i] < limits[i].min_pos - lite6_planner::tolerance::kJointPosition ||
          wp.joint_pos[i] > limits[i].max_pos + lite6_planner::tolerance::kJointPosition) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(5)
            << "MovePVA waypoint " << (k + 1) << " violates joint" << (i + 1)
            << " position limit: value=" << wp.joint_pos[i] << " rad, range=["
            << limits[i].min_pos << ", " << limits[i].max_pos << "] rad";
        add_diag(
          out_diagnostics, D::STAGE_KINEMATIC_LIMIT, wp.loc, "ERR_JOINT_POS_LIMIT", oss.str());
        return false;
      }
    }
    q_nodes[k + 1] = wp.joint_pos;
  }

  if (!any_vel_or_acc_or_dt) {
    if (M == 1) {
      MultiAxisSCurve scurve;
      scurve.build(
        start_q, q_nodes[1], limits,
        pva.options.vel_scale, pva.options.acc_scale, pva.options.jerk_scale);
      sampled = scurve_samples(scurve, pva.waypoints.front().loc, 0.0, 1.0);
    } else {
      // Pure (P) sequence: C^3 joint spline + 3rd-order direct-transcription TOPP timing.
      C3JointSpline spline;
      spline.build(q_nodes);
      std::string topp_err;
      if (!TOPPJerkSolver::solve(
            spline, nullptr, limits, cart_limits,
            pva.options.vel_scale, pva.options.acc_scale, pva.options.jerk_scale,
            pva.loc, sampled, topp_err))
      {
        add_diag(out_diagnostics, D::STAGE_PLANNING, pva.loc, "ERR_TOPP_SOLVER", topp_err);
        return false;
      }
      const std::vector<double> knot_u = C3JointSpline::chord_length_parameters(q_nodes);
      for (auto & s : sampled.samples) {
        const auto it = std::upper_bound(knot_u.begin() + 1, knot_u.end() - 1, s.path_u);
        const size_t wp_idx = std::min(
          M - 1, static_cast<size_t>(std::distance(knot_u.begin() + 1, it)));
        s.loc = pva.waypoints[wp_idx].loc;
      }
    }
  } else {
    // Explicit (P, V), (P, V, A), or dt joint sequence: chained C^3 Septic Hermite segments.
    std::vector<std::array<double, 6>> v_nodes(M + 1, std::array<double, 6>{});
    std::vector<std::array<double, 6>> a_nodes(M + 1, std::array<double, 6>{});
    for (size_t k = 0; k + 1 < M; ++k) {
      if (pva.waypoints[k].velocity.has_value()) {
        v_nodes[k + 1] = *pva.waypoints[k].velocity;
        if (pva.waypoints[k].acceleration.has_value()) {
          a_nodes[k + 1] = *pva.waypoints[k].acceleration;
        }
      }
    }
    // Endpoint terminates at rest for safe controller settling (warning already emitted if nonzero).
    v_nodes[M].fill(0.0);
    a_nodes[M].fill(0.0);

    double t_accum = 0.0;
    for (size_t seg_idx = 0; seg_idx < M; ++seg_idx) {
      const auto & wp = pva.waypoints[seg_idx];
      SepticHermiteSegment seg;
      if (wp.dt.has_value()) {
        seg.build_with_duration(
          q_nodes[seg_idx], v_nodes[seg_idx], a_nodes[seg_idx],
          q_nodes[seg_idx + 1], v_nodes[seg_idx + 1], a_nodes[seg_idx + 1], *wp.dt);
      } else {
        if (!seg.build_time_optimal(
              q_nodes[seg_idx], v_nodes[seg_idx], a_nodes[seg_idx],
              q_nodes[seg_idx + 1], v_nodes[seg_idx + 1], a_nodes[seg_idx + 1],
              limits, pva.options.vel_scale, pva.options.acc_scale, pva.options.jerk_scale))
        {
          add_diag(
            out_diagnostics, D::STAGE_PLANNING, wp.loc, "ERR_PVA_INFEASIBLE",
            "MovePVA segment " + std::to_string(seg_idx + 1) +
              " has no feasible duration within 60 s satisfying scaled joint (pos/vel/acc/jerk) limits");
          return false;
        }
      }

      const int seg_ticks = std::max(1, static_cast<int>(std::llround(seg.duration() * 1000.0)));
      const int start_tick = (seg_idx == 0) ? 0 : 1;
      std::array<double, 6> prev_ddq = a_nodes[seg_idx];
      for (int tick = 0; tick <= seg_ticks; ++tick) {
        const double t_local = static_cast<double>(tick) * 0.001;
        SampledTrajectory1kHz::Sample smp;
        smp.time_sec = t_accum + t_local;
        smp.path_u = (static_cast<double>(seg_idx) +
                      static_cast<double>(tick) / seg_ticks) / static_cast<double>(M);
        std::array<double, 6> jrk{};
        seg.eval(t_local, smp.q, smp.dq, smp.ddq, jrk);
        if (seg_idx + 1 == M && tick == seg_ticks) {
          smp.dq.fill(0.0);
          smp.ddq.fill(0.0);
        }
        // Enforce MovePVA scaled vel/acc/jerk options when explicit wp.dt is specified.
        if (wp.dt.has_value()) {
          for (size_t i = 0; i < 6; ++i) {
            const double v_scaled = limits[i].max_vel * pva.options.vel_scale;
            const double a_scaled = limits[i].max_acc * pva.options.acc_scale;
            const double j_scaled = limits[i].max_jerk * pva.options.jerk_scale;
            if (std::abs(smp.dq[i]) > v_scaled + lite6_planner::tolerance::kTrajectoryVelocity) {
              std::ostringstream oss;
              oss << std::fixed << std::setprecision(5)
                  << "MovePVA explicit dt=" << *wp.dt << " s violates scaled joint" << (i + 1)
                  << " velocity limit at t=" << smp.time_sec << " s: |dq|=" << std::abs(smp.dq[i])
                  << " rad/s > " << v_scaled << " rad/s (vel=" << pva.options.vel_scale << ")";
              add_diag(out_diagnostics, D::STAGE_KINEMATIC_LIMIT, wp.loc, "ERR_PVA_VEL_LIMIT", oss.str());
              return false;
            }
            if (std::abs(smp.ddq[i]) > a_scaled + lite6_planner::tolerance::kTrajectoryAcceleration) {
              std::ostringstream oss;
              oss << std::fixed << std::setprecision(5)
                  << "MovePVA explicit dt=" << *wp.dt << " s violates scaled joint" << (i + 1)
                  << " acceleration limit at t=" << smp.time_sec << " s: |ddq|="
                  << std::abs(smp.ddq[i]) << " rad/s^2 > " << a_scaled
                  << " rad/s^2 (acc=" << pva.options.acc_scale << ")";
              add_diag(out_diagnostics, D::STAGE_KINEMATIC_LIMIT, wp.loc, "ERR_PVA_ACC_LIMIT", oss.str());
              return false;
            }
            if (tick > 0) {
              const double disc_jrk = std::abs(smp.ddq[i] - prev_ddq[i]) / 0.001;
              if (disc_jrk > j_scaled * 1.001 + 1e-3 || std::abs(jrk[i]) > j_scaled * 1.001 + 1e-3) {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(3)
                    << "MovePVA explicit dt=" << *wp.dt << " s violates scaled joint" << (i + 1)
                    << " jerk limit at t=" << smp.time_sec << " s: |jerk|="
                    << std::max(disc_jrk, std::abs(jrk[i])) << " rad/s^3 > " << j_scaled
                    << " rad/s^3 (jerk=" << pva.options.jerk_scale << ")";
                add_diag(out_diagnostics, D::STAGE_KINEMATIC_LIMIT, wp.loc, "ERR_PVA_JERK_LIMIT", oss.str());
                return false;
              }
            }
          }
        }
        prev_ddq = smp.ddq;
        if (tick >= start_tick) {
          smp.loc = wp.loc;
          sampled.samples.push_back(smp);
        }
      }
      t_accum += static_cast<double>(seg_ticks) * 0.001;
    }
    sampled.duration_sec = t_accum;
  }

  if (!verify_sampled_trajectory_1khz(
        sampled, /*check_singularity=*/false, out_diagnostics, is_impedance))
  {
    return false;
  }

  out_block = ExecutionBlock();
  out_block.source_kind = BlockSourceKind::MOVE_PVA;
  out_block.source_pva = pva;
  out_block.path_start_joints = start_q;
  populate_execution_block(sampled, is_impedance, imp_params, out_block);
  return true;
}

bool ProgramCompiler::compile(
  const std::string & source_code,
  const std::array<double, 6> & start_joints,
  CompiledProgram & out_program,
  std::vector<Diagnostic> & out_diagnostics) const
{
  out_program = CompiledProgram{};
  out_diagnostics.clear();

  // Stages 1 & 2: Lexical, Syntax, and Semantic Analysis
  ASTProgram ast;
  if (!DSLParser::parse(source_code, ast, out_diagnostics)) {
    return false;
  }

  // Validate initial robot joint configuration
  std::array<double, 6> curr_q{};
  if (!validate_and_clamp_start_joints(
        start_joints, ast.statements.front().loc, curr_q, out_diagnostics)) {
    return false;
  }

  out_program.source_code = source_code;
  out_program.total_statements = static_cast<uint32_t>(ast.total_leaf_statements);
  out_program.initial_joints = curr_q;

  // Recursive statement list compiler supporting corner-blended groups
  auto compile_stmt_list = [&](
      const std::vector<ASTStatement> & stmts,
      bool is_impedance,
      const ImpedanceParams & imp_params,
      auto & self_ref) -> bool {
    size_t i = 0;
    while (i < stmts.size()) {
      const auto & st = stmts[i];
      if (st.kind == StatementKind::MOTION) {
        const auto mtype = st.motion->type;
        const bool is_cart =
          (mtype == MotionCommandType::MOVE_L ||
           mtype == MotionCommandType::MOVE_C_THREE_POINT ||
           mtype == MotionCommandType::MOVE_C_CENTER_ANGLE);

        std::vector<MotionStatement> group;
        group.push_back(*st.motion);
        while (group.back().options.blend_radius > 1e-5 && i + 1 < stmts.size() &&
               stmts[i + 1].kind == StatementKind::MOTION) {
          const auto next_type = stmts[i + 1].motion->type;
          const bool next_cart =
            (next_type == MotionCommandType::MOVE_L ||
             next_type == MotionCommandType::MOVE_C_THREE_POINT ||
             next_type == MotionCommandType::MOVE_C_CENTER_ANGLE);
          if (next_cart != is_cart) {
            break;
          }
          ++i;
          group.push_back(*stmts[i].motion);
        }

        ExecutionBlock blk;
        const bool ok = is_cart ?
          compile_cartesian_group(group, curr_q, is_impedance, imp_params, blk, out_diagnostics) :
          compile_joint_group(group, curr_q, blk, out_diagnostics);
        if (!ok) {
          return false;
        }
        curr_q = blk.end_joints;
        out_program.blocks.push_back(std::move(blk));
        ++i;
      } else if (st.kind == StatementKind::MOVE_PVA) {
        ExecutionBlock blk;
        if (!compile_pva_statement(
              *st.move_pva, curr_q, is_impedance, imp_params, blk, out_diagnostics)) {
          return false;
        }
        curr_q = blk.end_joints;
        out_program.blocks.push_back(std::move(blk));
        ++i;
      } else if (st.kind == StatementKind::IMPEDANCE_BLOCK) {
        if (!self_ref(st.body, true, *st.impedance_params, self_ref)) {
          return false;
        }
        ++i;
      } else if (st.kind == StatementKind::GRIPPER) {
        ExecutionBlock blk;
        blk.type = ExecutionBlockType::GRIPPER_CMD;
        blk.loc = st.loc;
        blk.start_joints = curr_q;
        blk.end_joints = curr_q;
        blk.gripper_cmd = *st.gripper;
        blk.duration_sec = st.gripper->wait_for_completion ? 0.6 : 0.01;
        out_program.blocks.push_back(std::move(blk));
        ++i;
      } else if (st.kind == StatementKind::WAIT) {
        ExecutionBlock blk;
        blk.type = ExecutionBlockType::WAIT_CMD;
        blk.loc = st.loc;
        blk.start_joints = curr_q;
        blk.end_joints = curr_q;
        blk.wait_cmd = *st.wait_stmt;
        blk.duration_sec = st.wait_stmt->duration_sec;
        out_program.blocks.push_back(std::move(blk));
        ++i;
      } else if (st.kind == StatementKind::SET_PAYLOAD) {
        if (!st.payload_stmt || !lite6_interfaces::valid_payload_params(
              st.payload_stmt->mass_kg, st.payload_stmt->com_tcp)) {
          add_diag(out_diagnostics, D::STAGE_SEMANTIC, st.loc, "ERR_PAYLOAD_RANGE",
            "Additional payload or TCP-frame center of mass exceeds simulation bounds");
          return false;
        }
        ExecutionBlock blk;
        blk.type = ExecutionBlockType::PAYLOAD_CMD;
        blk.loc = st.loc;
        blk.start_joints = curr_q;
        blk.end_joints = curr_q;
        blk.payload_cmd = *st.payload_stmt;
        blk.duration_sec = 0.05;
        out_program.blocks.push_back(std::move(blk));
        ++i;
      }
    }
    return true;
  };

  if (!compile_stmt_list(ast.statements, false, ImpedanceParams{}, compile_stmt_list)) {
    return false;
  }

  out_program.total_blocks = static_cast<uint32_t>(out_program.blocks.size());
  double total_dur = 0.0;
  uint32_t total_samples = 0;
  for (const auto & blk : out_program.blocks) {
    total_dur += blk.duration_sec;
    if (blk.type == ExecutionBlockType::RIGID_TRAJECTORY) {
      total_samples += static_cast<uint32_t>(blk.rigid_trajectory.points.size());
    } else if (blk.type == ExecutionBlockType::IMPEDANCE_TRAJECTORY) {
      total_samples += static_cast<uint32_t>(blk.impedance_goal.points.size());
    }
  }
  out_program.estimated_duration_sec = total_dur;
  out_program.total_samples_1khz = total_samples;

  const auto stamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  out_program.program_id = "prog_" + std::to_string(stamp_ms);
  return true;
}

bool ProgramCompiler::plan_alignment_bridge(
  const std::array<double, 6> & q_from,
  const std::array<double, 6> & q_to,
  double vel_scale,
  double acc_scale,
  trajectory_msgs::msg::JointTrajectory & out_traj,
  std::string & out_error) const
{
  const auto & limits = engine_.get_joint_limits();
  std::array<double, 6> q0 = q_from;
  std::array<double, 6> q1 = q_to;
  for (size_t i = 0; i < 6; ++i) {
    if (!std::isfinite(q0[i]) || !std::isfinite(q1[i]) ||
        q0[i] < limits[i].min_pos - lite6_planner::tolerance::kObservedPosition ||
        q0[i] > limits[i].max_pos + lite6_planner::tolerance::kObservedPosition ||
        q1[i] < limits[i].min_pos - lite6_planner::tolerance::kObservedPosition ||
        q1[i] > limits[i].max_pos + lite6_planner::tolerance::kObservedPosition) {
      out_error = "Alignment bridge endpoints exceed joint position limits";
      return false;
    }
    q0[i] = std::clamp(q0[i], limits[i].min_pos, limits[i].max_pos);
    q1[i] = std::clamp(q1[i], limits[i].min_pos, limits[i].max_pos);
  }

  MultiAxisSCurve scurve;
  scurve.build(q0, q1, limits, std::clamp(vel_scale, 0.05, 1.0), std::clamp(acc_scale, 0.05, 1.0));
  const SourceLocation bridge_loc{0, 0, -1, "<alignment_bridge>"};
  const SampledTrajectory1kHz sampled = scurve_samples(scurve, bridge_loc, 0.0, 1.0);

  std::vector<Diagnostic> diags;
  if (!verify_sampled_trajectory_1khz(sampled, false, diags)) {
    out_error = first_error_message(diags, "Alignment bridge failed verification");
    return false;
  }

  ExecutionBlock blk;
  populate_execution_block(sampled, false, ImpedanceParams{}, blk);
  out_traj = std::move(blk.rigid_trajectory);
  return true;
}

ProgramCompiler::PauseSpliceStatus ProgramCompiler::plan_pause_splice(
  const ExecutionBlock & block,
  double t_switch_sec,
  trajectory_msgs::msg::JointTrajectory & out_tail,
  size_t & out_end_sample,
  std::string & out_error) const
{
  // Share of the acceleration limit used for the initial stop-duration guess, and the 1 kHz grid.
  constexpr double kLimitShare = 0.8;
  constexpr double kDt = 0.001;
  constexpr int kMaxSearch = 48;

  const auto & pts = block.rigid_trajectory.points;
  if (block.type != ExecutionBlockType::RIGID_TRAJECTORY || pts.size() < 3) {
    out_error = "Pause splice requires a rigid trajectory block";
    return PauseSpliceStatus::FAILED;
  }
  auto t_of = [](const trajectory_msgs::msg::JointTrajectoryPoint & p) {
    return p.time_from_start.sec + 1e-9 * p.time_from_start.nanosec;
  };
  const size_t n = pts.size();
  const double t_end = t_of(pts.back());

  // Switch sample: the first original sample at or after t_switch_sec.
  size_t k_s = 0;
  {
    const auto it = std::lower_bound(
      pts.begin(), pts.end(), t_switch_sec - 1e-9,
      [&t_of](const trajectory_msgs::msg::JointTrajectoryPoint & p, double t) {
        return t_of(p) < t;
      });
    k_s = static_cast<size_t>(std::distance(pts.begin(), it));
  }
  if (k_s + 2 >= n) {
    return PauseSpliceStatus::LET_FINISH;
  }
  const double t_sw = t_of(pts[k_s]);
  const double remaining = t_end - t_sw;

  // Resolve the programmed vel/acc/jerk scales active at any sample time tau.
  auto options_at_time = [&](double tau) -> MotionOptions {
    MotionOptions full;
    full.vel_scale = 1.0;
    full.acc_scale = 1.0;
    full.jerk_scale = 1.0;
    if (block.source_kind == BlockSourceKind::MOVE_PVA) {
      return block.source_pva.options;
    }
    if ((block.source_kind == BlockSourceKind::CARTESIAN_GROUP ||
         block.source_kind == BlockSourceKind::JOINT_GROUP) &&
        !block.source_motions.empty())
    {
      const size_t idx = std::min(
        n - 1, static_cast<size_t>(std::max<long long>(0LL, std::llround(tau / kDt))));
      if (idx < block.sample_locations.size()) {
        const int stmt_idx = block.sample_locations[idx].statement_index;
        for (const auto & m : block.source_motions) {
          if (m.loc.statement_index == stmt_idx) {
            return m.options;
          }
        }
      }
      return block.source_motions.front().options;
    }
    return full;
  };

  // Original reference at block time tau: cubic Hermite position from (q, dq) and
  // linear velocity/acceleration between neighboring 1 kHz samples.
  auto eval_orig = [&](double tau, Vec6 & q, Vec6 & dq, Vec6 & ddq) {
    tau = std::clamp(tau, 0.0, t_end);
    size_t a = std::min(n - 2, static_cast<size_t>(std::max(0.0, std::floor(tau / kDt))));
    while (a + 1 < n - 1 && t_of(pts[a + 1]) <= tau) {
      ++a;
    }
    while (a > 0 && t_of(pts[a]) > tau) {
      --a;
    }
    const auto & pa = pts[a];
    const auto & pb = pts[a + 1];
    const double h = std::max(1e-9, t_of(pb) - t_of(pa));
    const double s = std::clamp((tau - t_of(pa)) / h, 0.0, 1.0);
    const double s2 = s * s;
    const double s3 = s2 * s;
    const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
    const double h10 = s3 - 2.0 * s2 + s;
    const double h01 = -2.0 * s3 + 3.0 * s2;
    const double h11 = s3 - s2;
    for (int i = 0; i < 6; ++i) {
      const size_t j = static_cast<size_t>(i);
      q[i] = h00 * pa.positions[j] + h10 * h * pa.velocities[j] +
        h01 * pb.positions[j] + h11 * h * pb.velocities[j];
      dq[i] = (1.0 - s) * pa.velocities[j] + s * pb.velocities[j];
      ddq[i] = (1.0 - s) * pa.accelerations[j] + s * pb.accelerations[j];
    }
  };

  // sigma(x) = 1 - S7(x), S7 = 35x^4 - 84x^5 + 70x^6 - 20x^7 (C^3 smoothstep);
  // integral of S7 over [0, x] = 7x^5 - 14x^6 + 10x^7 - 2.5x^8, so the stop consumes
  // exactly T_d / 2 of block time.
  auto s7 = [](double x) {
    const double x4 = x * x * x * x;
    return x4 * (35.0 + x * (-84.0 + x * (70.0 - 20.0 * x)));
  };
  auto s7_d = [](double x) {
    const double x3 = x * x * x;
    return x3 * (140.0 + x * (-420.0 + x * (420.0 - 140.0 * x)));
  };
  auto s7_int = [](double x) {
    const double x5 = x * x * x * x * x;
    return x5 * (7.0 + x * (-14.0 + x * (10.0 - 2.5 * x)));
  };

  const auto & limits = engine_.get_joint_limits();
  const auto & cart_limits = engine_.get_cartesian_limits();
  const bool check_cart =
    (block.source_kind == BlockSourceKind::CARTESIAN_GROUP) ||
    (block.source_kind == BlockSourceKind::MOVE_PVA &&
     block.source_pva.space == SpaceType::CARTESIAN);
  const auto & model = engine_.get_pin_model();
  const auto tcp = engine_.tcp_frame_id();
  auto data = engine_.create_data_workspace();

  auto eval_cartesian_kin = [&](
    const Vec6 & q, const Vec6 & v, const Vec6 & a,
    Eigen::Vector3d & v_lin, Eigen::Vector3d & w_ang,
    Eigen::Vector3d & a_lin, Eigen::Vector3d & alpha_ang)
  {
    pinocchio::Data::Matrix6x J(6, 6), dJ(6, 6);
    J.setZero();
    dJ.setZero();
    pinocchio::computeFrameJacobian(model, data, q, tcp, pinocchio::LOCAL_WORLD_ALIGNED, J);
    pinocchio::computeJointJacobiansTimeVariation(model, data, q, v);
    pinocchio::updateFramePlacement(model, data, tcp);
    pinocchio::getFrameJacobianTimeVariation(model, data, tcp, pinocchio::LOCAL_WORLD_ALIGNED, dJ);
    const Vec6 V = J * v;
    const Vec6 A = J * a + dJ * v;
    v_lin = V.head<3>();
    w_ang = V.tail<3>();
    a_lin = A.head<3>();
    alpha_ang = A.tail<3>();
  };

  const MotionOptions opts0 = options_at_time(t_sw);
  double t_d = 0.05;
  for (size_t i = 0; i < 6; ++i) {
    // Peak sigma_dot of the septic smoothstep is 2.1875 / T_d.
    const double a_eff = std::max(1e-4, opts0.acc_scale * limits[i].max_acc);
    t_d = std::max(t_d, 2.1875 * std::abs(pts[k_s].velocities[i]) / (kLimitShare * a_eff));
  }
  if (check_cart) {
    Vec6 q0, v0, a0;
    for (int i = 0; i < 6; ++i) {
      q0[i] = pts[k_s].positions[static_cast<size_t>(i)];
      v0[i] = pts[k_s].velocities[static_cast<size_t>(i)];
      a0[i] = pts[k_s].accelerations[static_cast<size_t>(i)];
    }
    Eigen::Vector3d vl0, wa0, al0, aa0;
    eval_cartesian_kin(q0, v0, a0, vl0, wa0, al0, aa0);
    t_d = std::max({
      t_d,
      2.1875 * vl0.norm() / std::max(1e-4, kLimitShare * opts0.acc_scale * cart_limits.max_trans_acc),
      2.1875 * wa0.norm() / std::max(1e-4, kLimitShare * opts0.acc_scale * cart_limits.max_rot_acc)});
  }

  for (int attempt = 0; attempt < kMaxSearch; ++attempt, t_d *= 1.12) {
    const int ticks = static_cast<int>(std::ceil(t_d / kDt - 1e-9));
    const double td = ticks * kDt;
    if (0.5 * td >= remaining - kDt) {
      // The programmed motion stops sooner than a path-preserving stop could.
      return PauseSpliceStatus::LET_FINISH;
    }
    trajectory_msgs::msg::JointTrajectory tail;
    tail.joint_names = block.rigid_trajectory.joint_names;
    tail.points.reserve(static_cast<size_t>(ticks) + 1);
    tail.points.push_back(pts[k_s]);  // verbatim: seamless controller switch
    tail.points.front().effort.clear();
    const int64_t t0_ns = static_cast<int64_t>(pts[k_s].time_from_start.sec) * 1000000000LL +
      static_cast<int64_t>(pts[k_s].time_from_start.nanosec);
    bool ok = true;
    Vec6 prev_ddq;
    for (int i = 0; i < 6; ++i) {
      prev_ddq[i] = pts[k_s].accelerations[static_cast<size_t>(i)];
    }
    double tau_end = t_sw;
    for (int k = 1; k <= ticks && ok; ++k) {
      const double x = static_cast<double>(k) / ticks;
      const double sigma = 1.0 - s7(x);
      const double sigma_dot = -s7_d(x) / td;
      const double tau = t_sw + td * (x - s7_int(x));
      Vec6 q, dq_o, ddq_o;
      eval_orig(tau, q, dq_o, ddq_o);
      const MotionOptions opts = options_at_time(tau);
      trajectory_msgs::msg::JointTrajectoryPoint p;
      p.positions.resize(6);
      p.velocities.resize(6);
      p.accelerations.resize(6);
      const int64_t t_ns = t0_ns + static_cast<int64_t>(k) * 1000000LL;
      p.time_from_start.sec = static_cast<int32_t>(t_ns / 1000000000LL);
      p.time_from_start.nanosec = static_cast<uint32_t>(t_ns % 1000000000LL);
      Vec6 v_vec = Vec6::Zero(), a_vec = Vec6::Zero();
      for (int i = 0; i < 6; ++i) {
        const size_t j = static_cast<size_t>(i);
        const bool last = (k == ticks);
        const double v = last ? 0.0 : dq_o[i] * sigma;
        const double a = last ? 0.0 : ddq_o[i] * sigma * sigma + dq_o[i] * sigma_dot;
        const auto & lim = limits[j];
        const double v_lim = lim.max_vel * opts.vel_scale;
        const double a_lim = lim.max_acc * opts.acc_scale;
        const double j_lim = lim.max_jerk * opts.jerk_scale;
        if (std::abs(v) > v_lim + lite6_planner::tolerance::kTrajectoryVelocity ||
            std::abs(a) > a_lim + lite6_planner::tolerance::kTrajectoryAcceleration ||
            std::abs(a - prev_ddq[i]) / kDt > j_lim * 1.001 + 1e-3)
        {
          ok = false;
          break;
        }
        p.positions[j] = q[i];
        p.velocities[j] = v;
        p.accelerations[j] = a;
        v_vec[i] = v;
        a_vec[i] = a;
        prev_ddq[i] = a;
      }
      if (ok && check_cart && (k % 5 == 0 || k == 1 || k == ticks)) {
        Eigen::Vector3d vl, wa, al, aa;
        eval_cartesian_kin(q, v_vec, a_vec, vl, wa, al, aa);
        if (vl.norm() > cart_limits.max_trans_vel * opts.vel_scale + 1e-4 ||
            wa.norm() > cart_limits.max_rot_vel * opts.vel_scale + 1e-4 ||
            al.norm() > cart_limits.max_trans_acc * opts.acc_scale + 1e-3 ||
            aa.norm() > cart_limits.max_rot_acc * opts.acc_scale + 1e-3)
        {
          ok = false;
        }
      }
      tau_end = tau;
      tail.points.push_back(std::move(p));
    }
    // Full 1 kHz Cartesian confirmation on the accepted candidate.
    if (ok && check_cart) {
      for (int k = 1; k <= ticks && ok; ++k) {
        if (k % 5 == 0 || k == 1 || k == ticks) {
          continue;
        }
        const double x = static_cast<double>(k) / ticks;
        const double tau = t_sw + td * (x - s7_int(x));
        const MotionOptions opts = options_at_time(tau);
        Vec6 q_vec, v_vec, a_vec;
        for (int i = 0; i < 6; ++i) {
          q_vec[i] = tail.points[static_cast<size_t>(k)].positions[static_cast<size_t>(i)];
          v_vec[i] = tail.points[static_cast<size_t>(k)].velocities[static_cast<size_t>(i)];
          a_vec[i] = tail.points[static_cast<size_t>(k)].accelerations[static_cast<size_t>(i)];
        }
        Eigen::Vector3d vl, wa, al, aa;
        eval_cartesian_kin(q_vec, v_vec, a_vec, vl, wa, al, aa);
        if (vl.norm() > cart_limits.max_trans_vel * opts.vel_scale + 1e-4 ||
            wa.norm() > cart_limits.max_rot_vel * opts.vel_scale + 1e-4 ||
            al.norm() > cart_limits.max_trans_acc * opts.acc_scale + 1e-3 ||
            aa.norm() > cart_limits.max_rot_acc * opts.acc_scale + 1e-3)
        {
          ok = false;
        }
      }
    }
    if (!ok) {
      continue;
    }
    out_tail = std::move(tail);
    out_end_sample = std::min(n - 1, static_cast<size_t>(std::llround(tau_end / kDt)));
    return PauseSpliceStatus::OK;
  }
  out_error = "No admissible path-preserving stop within joint/Cartesian limits";
  return PauseSpliceStatus::FAILED;
}

bool ProgramCompiler::replan_remaining_block(
  const ExecutionBlock & original_block,
  size_t pause_sample_idx,
  const std::array<double, 6> & q_actual,
  ExecutionBlock & out_resumed_block,
  std::string & out_error) const
{
  const bool is_imp = (original_block.type == ExecutionBlockType::IMPEDANCE_TRAJECTORY);
  const size_t total_pts = is_imp ?
    original_block.impedance_goal.points.size() :
    original_block.rigid_trajectory.points.size();
  if (total_pts < 2) {
    out_error = "Original trajectory block is empty";
    return false;
  }
  auto get_pt_q = [&](size_t idx) -> std::array<double, 6> {
    std::array<double, 6> q{};
    if (is_imp) {
      q = original_block.impedance_goal.points[idx].positions;
    } else {
      for (size_t i = 0; i < 6; ++i) {
        q[i] = original_block.rigid_trajectory.points[idx].positions[i];
      }
    }
    return q;
  };
  auto path_u_at = [&](size_t idx) {
    return original_block.sample_path_u.size() == total_pts ?
      original_block.sample_path_u[idx] :
      static_cast<double>(idx) / static_cast<double>(total_pts - 1);
  };
  auto loc_at = [&](size_t idx) {
    return idx < original_block.sample_locations.size() ?
      original_block.sample_locations[idx] : original_block.loc;
  };

  // 1. Resume point: the reference sample closest to the actual configuration. The
  //    reported pause sample lags (50 Hz feedback) and the arm brakes past it, so search
  //    a window around it instead of trusting the index.
  const size_t pause_idx = std::min(pause_sample_idx, total_pts - 1);
  const size_t lo = pause_idx > 500 ? pause_idx - 500 : 0;
  const size_t hi = std::min(total_pts - 1, pause_idx + 2500);
  size_t resume_idx = pause_idx;
  double best = std::numeric_limits<double>::infinity();
  for (size_t idx = lo; idx <= hi; ++idx) {
    const auto q = get_pt_q(idx);
    double d2 = 0.0;
    for (size_t i = 0; i < 6; ++i) {
      d2 += (q[i] - q_actual[i]) * (q[i] - q_actual[i]);
    }
    if (d2 < best) {
      best = d2;
      resume_idx = idx;
    }
  }
  const double u_r = path_u_at(resume_idx);
  const SourceLocation loc_r = loc_at(resume_idx);

  // 2. Programmed options of the interrupted motion (never exceeded on resume).
  MotionOptions opts;
  opts.vel_scale = 0.3;
  opts.acc_scale = 0.3;
  if ((original_block.source_kind == BlockSourceKind::CARTESIAN_GROUP ||
       original_block.source_kind == BlockSourceKind::JOINT_GROUP) &&
      !original_block.source_motions.empty())
  {
    opts = original_block.source_motions.front().options;
    for (const auto & m : original_block.source_motions) {
      opts.vel_scale = std::min(opts.vel_scale, m.options.vel_scale);
      opts.acc_scale = std::min(opts.acc_scale, m.options.acc_scale);
      opts.jerk_scale = std::min(opts.jerk_scale, m.options.jerk_scale);
    }
  } else if (original_block.source_kind == BlockSourceKind::MOVE_PVA) {
    opts = original_block.source_pva.options;
  }
  const auto limits = motion_limits(is_imp);
  auto diag_error = [&](const std::vector<Diagnostic> & diags, const std::string & fallback) {
    out_error = first_error_message(diags, fallback);
    return false;
  };
  auto map_path_u = [u_r](SampledTrajectory1kHz & traj) {
    for (auto & smp : traj.samples) {
      smp.path_u = u_r + smp.path_u * (1.0 - u_r);
    }
  };

  // 3. Remaining geometric path from u_r, re-parameterized with the programmed options.
  SampledTrajectory1kHz rest;
  const bool has_cartesian_path = static_cast<bool>(original_block.cartesian_path);
  bool check_singularity = has_cartesian_path;
  bool straight_to_end = has_cartesian_path ? u_r >= 1.0 : (total_pts - 1 - resume_idx) <= 5;
  if (!straight_to_end) {
    std::vector<Diagnostic> diags;
    if (has_cartesian_path) {
      const RestrictedCartesianPath sub(original_block.cartesian_path, u_r);
      if (!plan_cartesian_path(sub, get_pt_q(resume_idx), opts, loc_r, is_imp, rest, diags)) {
        return diag_error(diags, "Resumed Cartesian path planning failed");
      }
      map_path_u(rest);
    } else {
      switch (original_block.source_kind) {
        case BlockSourceKind::CARTESIAN_GROUP:
          out_error = "Cartesian block has no retained path for resume";
          return false;
        case BlockSourceKind::JOINT_GROUP: {
          if (original_block.source_motions.size() < 2) {
            // A single PTP motion stays a straight joint line to its target.
            straight_to_end = true;
            break;
          }
          std::vector<std::array<double, 6>> waypoints;
          JointCompositePath full;
          if (!build_joint_group_path(
                original_block.source_motions, original_block.path_start_joints,
                waypoints, full, diags))
          {
            return diag_error(diags, "Cannot rebuild joint path for resume");
          }
          const JointCompositePath sub = full.restricted(u_r);
          std::string topp_err;
          if (!TOPPJerkSolver::solve(
                sub, nullptr, limits, engine_.get_cartesian_limits(),
                opts.vel_scale, opts.acc_scale, opts.jerk_scale, loc_r, rest, topp_err))
          {
            out_error = "Resumed joint path planning failed: " + topp_err;
            return false;
          }
          map_path_u(rest);
          break;
        }
        default: {
          // Joint MovePVA or source-less blocks: follow the remaining reference geometry
          // through a chord-length spline, re-timed with the programmed scales.
          // Knots at uniform joint-space arc length: sample strides would crowd knots where
          // the reference decelerates and create spurious kinks.
          std::vector<double> arc(total_pts, 0.0);
          for (size_t idx = resume_idx + 1; idx < total_pts; ++idx) {
            const auto qa = get_pt_q(idx - 1);
            const auto qb = get_pt_q(idx);
            double d2 = 0.0;
            for (size_t i = 0; i < 6; ++i) {
              d2 += (qb[i] - qa[i]) * (qb[i] - qa[i]);
            }
            arc[idx] = arc[idx - 1] + std::sqrt(d2);
          }
          const double total_arc = arc[total_pts - 1];
          const double spacing = std::max(total_arc / 60.0, 0.01);
          std::vector<std::array<double, 6>> knots{get_pt_q(resume_idx)};
          std::vector<size_t> knot_idx{resume_idx};
          for (size_t idx = resume_idx + 1; idx + 1 < total_pts; ++idx) {
            if (arc[idx] - arc[knot_idx.back()] >= spacing &&
                total_arc - arc[idx] >= 0.5 * spacing)
            {
              knots.push_back(get_pt_q(idx));
              knot_idx.push_back(idx);
            }
          }
          knots.push_back(original_block.end_joints);
          knot_idx.push_back(total_pts - 1);
          C3JointSpline spline;
          spline.build(knots);
          std::string topp_err;
          if (!TOPPJerkSolver::solve(
                spline, nullptr, limits, engine_.get_cartesian_limits(),
                opts.vel_scale, opts.acc_scale, opts.jerk_scale, loc_r, rest, topp_err))
          {
            out_error = "Resumed trajectory planning failed: " + topp_err;
            return false;
          }
          const std::vector<double> knot_u = C3JointSpline::chord_length_parameters(knots);
          for (auto & smp : rest.samples) {
            const auto it = std::upper_bound(knot_u.begin() + 1, knot_u.end() - 1, smp.path_u);
            const size_t k = static_cast<size_t>(std::distance(knot_u.begin() + 1, it));
            const double span = knot_u[k + 1] - knot_u[k];
            const double frac = span > 1e-12 ? std::clamp((smp.path_u - knot_u[k]) / span, 0.0, 1.0) : 0.0;
            const size_t idx = knot_idx[k] + static_cast<size_t>(std::llround(
              frac * static_cast<double>(knot_idx[k + 1] - knot_idx[k])));
            smp.path_u = path_u_at(idx);
            smp.loc = loc_at(idx);
          }
          break;
        }
      }
    }

  }

  // 4. Rejoin: rest-to-rest straight joint motion from the actual configuration.
  SampledTrajectory1kHz combined;
  const std::array<double, 6> join_q =
    straight_to_end ? original_block.end_joints : rest.samples.front().q;
  double join_dist = 0.0;
  for (size_t i = 0; i < 6; ++i) {
    join_dist = std::max(join_dist, std::abs(join_q[i] - q_actual[i]));
  }
  if (straight_to_end || join_dist > 1e-5) {
    // A pure correction never runs faster than 30 % nor faster than the programmed motion;
    // the remaining stroke of a single PTP motion keeps its programmed scales.
    const bool full_motion = straight_to_end && (total_pts - 1 - resume_idx) > 5;
    const double vs = full_motion ? opts.vel_scale : std::min(opts.vel_scale, 0.3);
    const double as = full_motion ? opts.acc_scale : std::min(opts.acc_scale, 0.3);
    std::array<double, 6> q_from = q_actual;
    for (size_t i = 0; i < 6; ++i) {
      if (q_from[i] < limits[i].min_pos - lite6_planner::tolerance::kObservedPosition ||
          q_from[i] > limits[i].max_pos + lite6_planner::tolerance::kObservedPosition)
      {
        out_error = "Actual configuration is outside joint limits; cannot resume";
        return false;
      }
      q_from[i] = std::clamp(q_from[i], limits[i].min_pos, limits[i].max_pos);
    }
    MultiAxisSCurve scurve;
    scurve.build(q_from, join_q, limits, vs, as, opts.jerk_scale);
    combined = scurve_samples(
      scurve, loc_r, u_r, straight_to_end ? 1.0 : u_r);
  }
  if (!straight_to_end) {
    append_at_rest(combined, rest);
  }

  std::vector<Diagnostic> diags;
  if (!verify_sampled_trajectory_1khz(combined, check_singularity, diags, is_imp)) {
    return diag_error(diags, "Resumed trajectory failed verification");
  }

  populate_execution_block(
    combined, is_imp, original_block.impedance_params, out_resumed_block);
  out_resumed_block.source_kind = original_block.source_kind;
  out_resumed_block.source_motions = original_block.source_motions;
  out_resumed_block.source_pva = original_block.source_pva;
  out_resumed_block.cartesian_path = original_block.cartesian_path;
  out_resumed_block.path_start_joints = original_block.path_start_joints;
  return true;
}

bool ProgramCompiler::replan_block_from_start(
  const ExecutionBlock & original_block,
  const std::array<double, 6> & q_actual,
  ExecutionBlock & out_replanned_block,
  std::vector<Diagnostic> & out_diagnostics) const
{
  out_diagnostics.clear();
  std::array<double, 6> start_q{};
  if (!validate_and_clamp_start_joints(q_actual, original_block.loc, start_q, out_diagnostics)) {
    return false;
  }

  const bool is_imp = (original_block.type == ExecutionBlockType::IMPEDANCE_TRAJECTORY);
  switch (original_block.source_kind) {
    case BlockSourceKind::CARTESIAN_GROUP:
      return compile_cartesian_group(
        original_block.source_motions, start_q, is_imp, original_block.impedance_params,
        out_replanned_block, out_diagnostics);
    case BlockSourceKind::JOINT_GROUP:
      return compile_joint_group(
        original_block.source_motions, start_q, out_replanned_block, out_diagnostics);
    case BlockSourceKind::MOVE_PVA:
      return compile_pva_statement(
        original_block.source_pva, start_q, is_imp, original_block.impedance_params,
        out_replanned_block, out_diagnostics);
    case BlockSourceKind::NONE:
    default:
      add_diag(
        out_diagnostics, D::STAGE_PLANNING, original_block.loc, "ERR_NO_REPLAN_SOURCE",
        "Execution block does not have source AST primitives for start-pose replanning");
      return false;
  }
}

}  // namespace lite6_studio_planner
