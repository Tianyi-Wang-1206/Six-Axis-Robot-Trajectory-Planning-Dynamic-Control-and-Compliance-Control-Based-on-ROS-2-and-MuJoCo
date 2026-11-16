#include "lite6_studio_planner/program_compiler.hpp"
#include "lite6_planner/motion_tolerances.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <rclcpp/duration.hpp>
#include <Eigen/SVD>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace lite6_studio_planner
{
namespace
{

using D = lite6_interfaces::msg::CompileDiagnostic;
using IKResult = lite6_planner::IKResult;

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

void add_diag(
  std::vector<D> & diags,
  uint8_t stage,
  const SourceLocation & loc,
  const std::string & code,
  const std::string & msg)
{
  D d;
  d.severity = D::SEVERITY_ERROR;
  d.stage = stage;
  d.line_number = loc.line;
  d.column_number = loc.column;
  d.statement_index = loc.statement_index;
  d.command_text = loc.text;
  d.code = code;
  d.message = msg;
  diags.push_back(d);
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

  if (is_impedance) {
    blk.impedance_params = imp_params;
    blk.impedance_goal.joint_names = kJointNames;
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

// Map a desired Cartesian spatial twist V=[vx,vy,vz,wx,wy,wz] and optional spatial acceleration
// at joint configuration q into joint velocities and accelerations via damped SVD Jacobian inversion.
bool cartesian_twist_to_joint(
  const pinocchio::Model & model,
  const std::array<double, 6> & q,
  const std::array<double, 6> & spatial_vel,
  const std::array<double, 6> * spatial_acc,
  std::array<double, 6> & out_dq,
  std::array<double, 6> * out_ddq)
{
  pinocchio::Data data(model);
  const Eigen::Map<const Eigen::Matrix<double, 6, 1>> qe(q.data());
  const pinocchio::FrameIndex frame_id = model.getFrameId("link_eef");
  if (frame_id >= static_cast<pinocchio::FrameIndex>(model.nframes)) {
    return false;
  }

  pinocchio::Data::Matrix6x J(6, 6);
  J.setZero();
  pinocchio::computeFrameJacobian(
    model, data, qe, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);

  const Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(
    J, Eigen::ComputeFullU | Eigen::ComputeFullV);
  auto solve_dls = [&](const Eigen::Matrix<double, 6, 1> & rhs) {
    Eigen::Matrix<double, 6, 1> proj = svd.matrixU().transpose() * rhs;
    for (int i = 0; i < 6; ++i) {
      const double s = svd.singularValues()[i];
      constexpr double lambda = 1e-4;
      proj[i] *= s / (s * s + lambda * lambda);
    }
    return Eigen::Matrix<double, 6, 1>(svd.matrixV() * proj);
  };

  Eigen::Matrix<double, 6, 1> V;
  for (int i = 0; i < 6; ++i) {
    V[i] = spatial_vel[static_cast<size_t>(i)];
  }
  const Eigen::Matrix<double, 6, 1> dq = solve_dls(V);
  if (!dq.allFinite()) {
    return false;
  }
  for (size_t i = 0; i < 6; ++i) {
    out_dq[i] = dq[static_cast<int>(i)];
  }

  if (spatial_acc != nullptr && out_ddq != nullptr) {
    pinocchio::computeJointJacobiansTimeVariation(model, data, qe, dq);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::Data::Matrix6x dJ(6, 6);
    dJ.setZero();
    pinocchio::getFrameJacobianTimeVariation(
      model, data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, dJ);
    Eigen::Matrix<double, 6, 1> A;
    for (int i = 0; i < 6; ++i) {
      A[i] = (*spatial_acc)[static_cast<size_t>(i)];
    }
    const Eigen::Matrix<double, 6, 1> ddq = solve_dls(A - dJ * dq);
    if (!ddq.allFinite()) {
      return false;
    }
    for (size_t i = 0; i < 6; ++i) {
      (*out_ddq)[i] = ddq[static_cast<int>(i)];
    }
  }
  return true;
}

}  // namespace

ProgramCompiler::ProgramCompiler(const lite6_planner::KinematicsEngine & engine)
: engine_(engine)
{
}

bool ProgramCompiler::verify_sampled_trajectory_1khz(
  const SampledTrajectory1kHz & sampled,
  bool check_singularity_along_path,
  std::vector<Diagnostic> & diags) const
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

  const auto & limits = engine_.get_joint_limits();
  const double sing_thresh = engine_.get_singularity_threshold();

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
            << " rad/s";
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
      const double sigma = engine_.check_singularity(s.q);
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
  auto workspace = engine_.collision_workspace();
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
  const Eigen::Matrix4d start_pose = engine_.forward_pose(start_q);
  CompositeCartesianPath cart_path;
  SourceLocation err_loc = group.front().loc;
  std::string err_code, err_msg;
  if (!cart_path.build(start_pose, group, err_loc, err_code, err_msg)) {
    add_diag(out_diagnostics, D::STAGE_PLANNING, err_loc, err_code, err_msg);
    return false;
  }

  size_t n_knots = std::max<size_t>(
    12, static_cast<size_t>(std::ceil(cart_path.total_length() / 0.004)));
  n_knots = std::min<size_t>(n_knots, 4000);

  C3JointSpline spline;
  bool spline_accurate = false;
  for (int refine = 0; refine < 3; ++refine) {
    std::vector<std::array<double, 6>> knots(n_knots + 1);
    knots[0] = start_q;
    for (size_t k = 1; k <= n_knots; ++k) {
      const double u = static_cast<double>(k) / static_cast<double>(n_knots);
      const CartesianSample cs = cart_path.eval_normalized(u);
      std::vector<std::array<double, 6>> sols;
      lite6_planner::IKDiagnostics ik_diag;
      const auto status =
        engine_.solve_optimal_ik(cs.pose, knots[k - 1], sols, /*continuous=*/true, &ik_diag);
      if (status != IKResult::SUCCESS) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4)
            << "Cartesian IK failed at path progress " << (100.0 * u) << "% (xyz=["
            << cs.pose(0, 3) << ", " << cs.pose(1, 3) << ", " << cs.pose(2, 3)
            << "] m): " << ik_diag.summary();
        add_diag(
          out_diagnostics, D::STAGE_PLANNING, cs.loc, ik_error_code(status), oss.str());
        return false;
      }
      knots[k] = sols.front();
    }

    spline.build(knots);
    spline_accurate = true;
    for (size_t k = 0; k < n_knots; ++k) {
      const double u_mid = (static_cast<double>(k) + 0.5) / static_cast<double>(n_knots);
      Vec6 qm, q1, q2, q3;
      spline.eval(u_mid, qm, q1, q2, q3);
      std::array<double, 6> q_arr{};
      for (int j = 0; j < 6; ++j) {
        q_arr[static_cast<size_t>(j)] = qm[j];
      }
      const Eigen::Matrix4d fk = engine_.forward_pose(q_arr);
      const CartesianSample ref = cart_path.eval_normalized(u_mid);
      const double p_err = (fk.topRightCorner<3, 1>() - ref.pose.topRightCorner<3, 1>()).norm();
      const double r_err = rot_diff(fk.topLeftCorner<3, 3>(), ref.pose.topLeftCorner<3, 3>());
      if (p_err > 7e-4 || r_err > 7e-3) {
        spline_accurate = false;
        if (refine == 2) {
          std::ostringstream oss;
          oss << std::fixed << std::setprecision(5)
              << "Cartesian spline tracking residual exceeded tolerance at "
              << (100.0 * u_mid) << "% (pos_err=" << p_err << " m, rot_err=" << r_err << " rad)";
          add_diag(
            out_diagnostics, D::STAGE_PLANNING, ref.loc, "ERR_SPLINE_RESIDUAL", oss.str());
          return false;
        }
        break;
      }
    }
    if (spline_accurate) {
      break;
    }
    n_knots = std::min<size_t>(n_knots * 2, 8000);
  }

  SampledTrajectory1kHz sampled;
  std::string topp_err;
  if (!TOPPJerkSolver::solve(
        spline, &cart_path, engine_.get_joint_limits(), engine_.get_cartesian_limits(),
        group.front().options.vel_scale, group.front().options.acc_scale,
        group.front().options.jerk_scale, group.front().loc, sampled, topp_err)) {
    add_diag(
      out_diagnostics, D::STAGE_PLANNING, group.front().loc, "ERR_TOPP_SOLVER", topp_err);
    return false;
  }

  if (!verify_sampled_trajectory_1khz(sampled, /*check_singularity=*/true, out_diagnostics)) {
    return false;
  }

  out_block = ExecutionBlock();
  out_block.source_kind = BlockSourceKind::CARTESIAN_GROUP;
  out_block.source_motions = group;
  populate_execution_block(sampled, is_impedance, imp_params, out_block);
  return true;
}

bool ProgramCompiler::compile_joint_group(
  const std::vector<MotionStatement> & group,
  const std::array<double, 6> & start_q,
  ExecutionBlock & out_block,
  std::vector<Diagnostic> & out_diagnostics) const
{
  const auto & limits = engine_.get_joint_limits();
  std::vector<std::array<double, 6>> target_waypoints;
  target_waypoints.reserve(group.size() + 1);
  target_waypoints.push_back(start_q);

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
      if (engine_.check_collision(st.joint_target, &col_detail)) {
        add_diag(
          out_diagnostics, D::STAGE_COLLISION, st.loc, "ERR_COLLISION",
          "MoveJ target configuration is in collision: " + col_detail);
        return false;
      }
      target_waypoints.push_back(st.joint_target);
    } else {
      // MoveP: resolve Cartesian target via optimal IK relative to previous waypoint
      std::vector<std::array<double, 6>> sols;
      lite6_planner::IKDiagnostics ik_diag;
      const auto status = engine_.solve_optimal_ik(
        st.pose_target, target_waypoints.back(), sols, /*continuous=*/false, &ik_diag);
      if (status != IKResult::SUCCESS) {
        add_diag(
          out_diagnostics, D::STAGE_PLANNING, st.loc, ik_error_code(status),
          "MoveP target IK failed: " + ik_diag.summary());
        return false;
      }
      target_waypoints.push_back(sols.front());
    }
  }

  SampledTrajectory1kHz sampled;
  if (group.size() == 1) {
    const auto & st = group.front();
    MultiAxisSCurve scurve;
    scurve.build(
      start_q, target_waypoints.back(), limits,
      st.options.vel_scale, st.options.acc_scale, st.options.jerk_scale);
    const double dur = scurve.duration();
    const int total_ticks = std::max(1, static_cast<int>(std::llround(dur * 1000.0)));
    sampled.duration_sec = static_cast<double>(total_ticks) * 0.001;
    sampled.samples.reserve(static_cast<size_t>(total_ticks + 1));

    for (int tick = 0; tick <= total_ticks; ++tick) {
      const double t = static_cast<double>(tick) * 0.001;
      SampledTrajectory1kHz::Sample smp;
      smp.time_sec = t;
      smp.path_u = (total_ticks > 0) ? (static_cast<double>(tick) / total_ticks) : 1.0;
      std::array<double, 6> jrk{};
      scurve.eval(t, smp.q, smp.dq, smp.ddq, jrk);
      if (tick == 0 || tick == total_ticks) {
        smp.dq.fill(0.0);
        smp.ddq.fill(0.0);
      }
      smp.loc = st.loc;
      sampled.samples.push_back(smp);
    }
  } else {
    // Multi-segment blended joint-space trajectory via C^3 spline + TOPP-Jerk solver
    std::vector<std::array<double, 6>> dense_knots;
    dense_knots.push_back(target_waypoints[0]);

    for (size_t seg = 0; seg < group.size(); ++seg) {
      const auto & q_a = target_waypoints[seg];
      const auto & q_b = target_waypoints[seg + 1];
      constexpr int kSubKnots = 6;
      for (int sub = 1; sub <= kSubKnots; ++sub) {
        const double s = static_cast<double>(sub) / kSubKnots;
        // If this corner is blended, stop slightly before the sharp vertex so C^3 spline rounds it
        if (sub == kSubKnots && seg + 1 < group.size() && group[seg].options.blend_radius > 1e-4) {
          continue;
        }
        std::array<double, 6> qm{};
        for (size_t j = 0; j < 6; ++j) {
          qm[j] = (1.0 - s) * q_a[j] + s * q_b[j];
        }
        dense_knots.push_back(qm);
      }
    }

    C3JointSpline spline;
    spline.build(dense_knots);
    double min_vs = 1.0, min_as = 1.0, min_js = 1.0;
    for (const auto & st : group) {
      min_vs = std::min(min_vs, st.options.vel_scale);
      min_as = std::min(min_as, st.options.acc_scale);
      min_js = std::min(min_js, st.options.jerk_scale);
    }
    std::string topp_err;
    if (!TOPPJerkSolver::solve(
          spline, nullptr, limits, engine_.get_cartesian_limits(),
          min_vs, min_as, min_js, group.front().loc, sampled, topp_err)) {
      add_diag(
        out_diagnostics, D::STAGE_PLANNING, group.front().loc, "ERR_TOPP_SOLVER", topp_err);
      return false;
    }
    for (auto & s : sampled.samples) {
      const size_t seg_idx = std::min(
        group.size() - 1,
        static_cast<size_t>(std::floor(s.path_u * static_cast<double>(group.size()))));
      s.loc = group[seg_idx].loc;
    }
  }

  if (!verify_sampled_trajectory_1khz(sampled, /*check_singularity=*/false, out_diagnostics)) {
    return false;
  }

  out_block = ExecutionBlock();
  out_block.source_kind = BlockSourceKind::JOINT_GROUP;
  out_block.source_motions = group;
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
  const auto & limits = engine_.get_joint_limits();
  const size_t M = pva.waypoints.size();
  std::vector<std::array<double, 6>> q_nodes(M + 1);
  q_nodes[0] = start_q;

  bool any_vel_or_acc_or_dt = false;
  for (size_t k = 0; k < M; ++k) {
    const auto & wp = pva.waypoints[k];
    if (wp.velocity.has_value() || wp.acceleration.has_value() || wp.dt.has_value()) {
      any_vel_or_acc_or_dt = true;
    }
    if (pva.space == SpaceType::JOINT) {
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
    } else {
      std::vector<std::array<double, 6>> sols;
      lite6_planner::IKDiagnostics ik_diag;
      const auto status = engine_.solve_optimal_ik(
        wp.cart_pose, q_nodes[k], sols, /*continuous=*/true, &ik_diag);
      if (status != IKResult::SUCCESS) {
        add_diag(
          out_diagnostics, D::STAGE_PLANNING, wp.loc, ik_error_code(status),
          "MovePVA Cartesian waypoint " + std::to_string(k + 1) + " IK failed: " +
            ik_diag.summary());
        return false;
      }
      q_nodes[k + 1] = sols.front();
    }
  }

  SampledTrajectory1kHz sampled;
  if (!any_vel_or_acc_or_dt && M >= 2) {
    // Pure (P) sequence: C^3 joint spline + TOPP-Jerk optimal time parameterization
    C3JointSpline spline;
    spline.build(q_nodes);
    std::string topp_err;
    if (!TOPPJerkSolver::solve(
          spline, nullptr, limits, engine_.get_cartesian_limits(),
          pva.options.vel_scale, pva.options.acc_scale, pva.options.jerk_scale,
          pva.loc, sampled, topp_err)) {
      add_diag(out_diagnostics, D::STAGE_PLANNING, pva.loc, "ERR_TOPP_SOLVER", topp_err);
      return false;
    }
    for (auto & s : sampled.samples) {
      const size_t wp_idx = std::min(
        M - 1, static_cast<size_t>(std::floor(s.path_u * static_cast<double>(M))));
      s.loc = pva.waypoints[wp_idx].loc;
    }
  } else {
    // Explicit (P, V), (P, V, A), or dt sequence: chained C^3 Septic Hermite segments
    std::vector<std::array<double, 6>> v_nodes(M + 1, std::array<double, 6>{});
    std::vector<std::array<double, 6>> a_nodes(M + 1, std::array<double, 6>{});

    for (size_t k = 0; k < M; ++k) {
      if (pva.waypoints[k].velocity.has_value()) {
        if (pva.space == SpaceType::CARTESIAN) {
          const std::array<double, 6> * acc_ptr =
            pva.waypoints[k].acceleration.has_value() ? &(*pva.waypoints[k].acceleration) : nullptr;
          if (!cartesian_twist_to_joint(
                engine_.get_pin_model(), q_nodes[k + 1], *pva.waypoints[k].velocity,
                acc_ptr, v_nodes[k + 1], acc_ptr ? &a_nodes[k + 1] : nullptr)) {
            add_diag(
              out_diagnostics, D::STAGE_PLANNING, pva.waypoints[k].loc, "ERR_PVA_JACOBIAN",
              "Failed to map Cartesian velocity/acceleration to joint space at waypoint " +
                std::to_string(k + 1));
            return false;
          }
        } else {
          v_nodes[k + 1] = *pva.waypoints[k].velocity;
          if (pva.waypoints[k].acceleration.has_value()) {
            a_nodes[k + 1] = *pva.waypoints[k].acceleration;
          }
        }
      }
    }
    // Endpoint must come to rest for safe controller settling
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
        seg.build_time_optimal(
          q_nodes[seg_idx], v_nodes[seg_idx], a_nodes[seg_idx],
          q_nodes[seg_idx + 1], v_nodes[seg_idx + 1], a_nodes[seg_idx + 1],
          limits, pva.options.vel_scale, pva.options.acc_scale, pva.options.jerk_scale);
      }

      const int seg_ticks = std::max(1, static_cast<int>(std::llround(seg.duration() * 1000.0)));
      const int start_tick = (seg_idx == 0) ? 0 : 1;
      for (int tick = start_tick; tick <= seg_ticks; ++tick) {
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
        smp.loc = wp.loc;
        sampled.samples.push_back(smp);
      }
      t_accum += static_cast<double>(seg_ticks) * 0.001;
    }
    sampled.duration_sec = t_accum;
  }

  if (!verify_sampled_trajectory_1khz(
        sampled, /*check_singularity=*/(pva.space == SpaceType::CARTESIAN), out_diagnostics)) {
    return false;
  }

  out_block = ExecutionBlock();
  out_block.source_kind = BlockSourceKind::MOVE_PVA;
  out_block.source_pva = pva;
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
  const int total_ticks = std::max(1, static_cast<int>(std::llround(scurve.duration() * 1000.0)));

  SampledTrajectory1kHz sampled;
  sampled.duration_sec = static_cast<double>(total_ticks) * 0.001;
  sampled.samples.reserve(static_cast<size_t>(total_ticks + 1));

  SourceLocation bridge_loc{0, 0, -1, "<alignment_bridge>"};
  for (int tick = 0; tick <= total_ticks; ++tick) {
    const double t = static_cast<double>(tick) * 0.001;
    SampledTrajectory1kHz::Sample smp;
    smp.time_sec = t;
    smp.path_u = static_cast<double>(tick) / total_ticks;
    std::array<double, 6> jrk{};
    scurve.eval(t, smp.q, smp.dq, smp.ddq, jrk);
    if (tick == 0 || tick == total_ticks) {
      smp.dq.fill(0.0);
      smp.ddq.fill(0.0);
    }
    smp.loc = bridge_loc;
    sampled.samples.push_back(smp);
  }

  std::vector<Diagnostic> diags;
  if (!verify_sampled_trajectory_1khz(sampled, false, diags)) {
    out_error = diags.empty() ? "Alignment bridge failed verification" : diags.front().message;
    return false;
  }

  ExecutionBlock blk;
  populate_execution_block(sampled, false, ImpedanceParams{}, blk);
  out_traj = std::move(blk.rigid_trajectory);
  return true;
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

  const size_t resume_idx = std::min(pause_sample_idx, total_pts - 1);
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

  const size_t remaining_ticks = total_pts - 1 - resume_idx;
  if (remaining_ticks <= 5) {
    // Near the very end of the block: simple S-curve to block endpoint
    trajectory_msgs::msg::JointTrajectory bridge;
    if (!plan_alignment_bridge(q_actual, original_block.end_joints, 0.3, 0.3, bridge, out_error)) {
      return false;
    }
    SampledTrajectory1kHz sampled;
    sampled.duration_sec =
      static_cast<double>(bridge.points.size() - 1) * 0.001;
    for (size_t k = 0; k < bridge.points.size(); ++k) {
      SampledTrajectory1kHz::Sample smp;
      smp.time_sec = static_cast<double>(k) * 0.001;
      for (size_t i = 0; i < 6; ++i) {
        smp.q[i] = bridge.points[k].positions[i];
        smp.dq[i] = bridge.points[k].velocities[i];
        smp.ddq[i] = bridge.points[k].accelerations[i];
      }
      smp.loc = original_block.sample_locations.back();
      sampled.samples.push_back(smp);
    }
    populate_execution_block(
      sampled, is_imp, original_block.impedance_params, out_resumed_block);
    return true;
  }

  // Subsample the remaining trajectory knots and build a C^3 spline starting at q_actual
  const size_t stride = std::max<size_t>(1, remaining_ticks / 40);
  std::vector<std::array<double, 6>> knots;
  std::vector<SourceLocation> knot_locs;
  knots.push_back(q_actual);
  knot_locs.push_back(original_block.sample_locations[resume_idx]);

  for (size_t idx = resume_idx + stride; idx < total_pts - 1; idx += stride) {
    knots.push_back(get_pt_q(idx));
    knot_locs.push_back(original_block.sample_locations[idx]);
  }
  knots.push_back(original_block.end_joints);
  knot_locs.push_back(original_block.sample_locations.back());

  C3JointSpline spline;
  spline.build(knots);

  SampledTrajectory1kHz sampled;
  if (!TOPPJerkSolver::solve(
        spline, nullptr, engine_.get_joint_limits(), engine_.get_cartesian_limits(),
        0.5, 0.5, 1.0, knot_locs.front(), sampled, out_error)) {
    return false;
  }
  for (auto & s : sampled.samples) {
    const size_t k_idx = std::min(
      knot_locs.size() - 1,
      static_cast<size_t>(std::floor(s.path_u * static_cast<double>(knot_locs.size()))));
    s.loc = knot_locs[k_idx];
  }

  std::vector<Diagnostic> diags;
  if (!verify_sampled_trajectory_1khz(sampled, false, diags)) {
    out_error = diags.empty() ? "Resumed trajectory failed verification" : diags.front().message;
    return false;
  }

  populate_execution_block(
    sampled, is_imp, original_block.impedance_params, out_resumed_block);
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
