#include "lite6_planner/motion_planner.hpp"
#include <rclcpp/duration.hpp>
#include <ruckig/ruckig.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>

namespace lite6_planner {
namespace {
using Action=lite6_interfaces::action::IndustrialMotion;
using Vec6=Eigen::Matrix<double,6,1>;
constexpr double kPi=3.14159265358979323846;
constexpr double kDt=0.01;
constexpr double kTiny=1e-10;
constexpr double kPathPosTolerance=7e-4;
constexpr double kPathRotTolerance=7e-3;
constexpr size_t kMaxPoints=30000;
constexpr size_t kMaxIKSamples=2000;

std::array<double,6> from_eigen(const Vec6& x) {
  std::array<double,6> result{};
  for(int i=0;i<6;++i) result[i]=x[i];
  return result;
}
Vec6 to_eigen(const std::array<double,6>& x) {
  return Eigen::Map<const Vec6>(x.data());
}
trajectory_msgs::msg::JointTrajectoryPoint make_point(const Vec6& q,
    const Vec6& v,const Vec6& a,double t) {
  trajectory_msgs::msg::JointTrajectoryPoint point;
  for(int i=0;i<6;++i) {
    point.positions.push_back(q[i]);
    point.velocities.push_back(v[i]);
    point.accelerations.push_back(a[i]);
  }
  point.time_from_start=rclcpp::Duration::from_seconds(t);
  return point;
}
double seconds(const builtin_interfaces::msg::Duration& d) {
  return static_cast<double>(d.sec)+1e-9*d.nanosec;
}
void fail(int32_t code,const std::string& msg) {throw MotionError(code,msg);}
std::string range_error(const std::string& context,size_t joint,double value,
    double lower,double upper,double allowance) {
  std::ostringstream out;
  out<<std::setprecision(12)<<context<<": joint"<<joint+1<<" value="<<value
     <<", range=["<<lower<<", "<<upper<<"], tolerance="<<allowance
     <<", excess="<<std::max({lower-value,value-upper,0.0});
  return out.str();
}
std::array<double,6> bounded_positions(const std::array<double,6>& q,
    const std::array<JointLimit,6>& limits,double allowance,int32_t code,
    const std::string& context) {
  std::array<double,6> result{};
  for(size_t i=0;i<6;++i) {
    if(!std::isfinite(q[i]))
      fail(code,context+": joint"+std::to_string(i+1)+" position is not finite");
    const auto& lim=limits[i];
    if(q[i]<lim.min_pos-allowance||q[i]>lim.max_pos+allowance)
      fail(code,range_error(context,i,q[i],lim.min_pos,lim.max_pos,allowance));
    result[i]=std::clamp(q[i],lim.min_pos,lim.max_pos);
  }
  return result;
}
int32_t ik_code(IKResult e) {
  if(e==IKResult::ERR_LIMIT_OR_COLLISION) return Action::Result::ERR_LIMIT_VIOLATION;
  if(e==IKResult::ERR_SINGULARITY) return Action::Result::ERR_SINGULARITY;
  if(e==IKResult::ERR_QUADRANT_JUMP) return Action::Result::ERR_QUADRANT_JUMP;
  return Action::Result::ERR_NO_IK_SOLUTION;
}
double angle_difference(const Eigen::Matrix3d& a,const Eigen::Matrix3d& b) {
  return std::abs(Eigen::AngleAxisd(a.transpose()*b).angle());
}
}

MotionPlanner::MotionPlanner(const KinematicsEngine& engine):engine_(engine){}
void MotionPlanner::check_cancelled(const Cancellation& c) {
  if(c&&c()) fail(Action::Result::ERR_CANCELLED,"Motion cancelled by supervisor");
}
Eigen::Matrix4d MotionPlanner::pose_matrix(const geometry_msgs::msg::Pose& pose) const {
  Eigen::Matrix4d T=Eigen::Matrix4d::Identity();
  const auto& p=pose.position;
  const auto& o=pose.orientation;
  if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||
     !std::isfinite(o.x)||!std::isfinite(o.y)||!std::isfinite(o.z)||!std::isfinite(o.w))
    fail(Action::Result::ERR_INVALID_INPUT,"Non-finite Cartesian target");
  Eigen::Quaterniond q(o.w,o.x,o.y,o.z);
  const double n=q.norm();
  if(n<1e-9||std::abs(n-1.0)>1e-3)
    fail(Action::Result::ERR_INVALID_INPUT,"Pose quaternion must be normalized");
  q.normalize();
  T.topLeftCorner<3,3>()=q.toRotationMatrix();
  T.topRightCorner<3,1>()=Eigen::Vector3d(p.x,p.y,p.z);
  return T;
}

trajectory_msgs::msg::JointTrajectory MotionPlanner::plan(const Goal& goal,
    const InitialState& initial,const Cancellation& cancel) const {
  if(!std::isfinite(goal.velocity_scale)||!std::isfinite(goal.acceleration_scale)||
     goal.velocity_scale<0.01||goal.velocity_scale>1.0||
     goal.acceleration_scale<0.01||goal.acceleration_scale>1.0)
    fail(Action::Result::ERR_INVALID_INPUT,"Scaling factors must lie in [0.01,1]");
  const auto& limits=engine_.get_joint_limits();
  InitialState start=initial;
  start.q=bounded_positions(initial.q,limits,tolerance::kObservedPosition,
      Action::Result::ERR_NOT_READY,"Current joint position [rad]");
  // No acceleration estimate is exported by joint_states. Require near-rest
  // before an independently generated stop-to-stop plan to avoid fake initial acceleration.
  for(size_t i=0;i<6;++i) {
    const double v=initial.dq[i];
    if(!std::isfinite(v)) fail(Action::Result::ERR_NOT_READY,
        "Current joint"+std::to_string(i+1)+" velocity is not finite");
    if(std::abs(v)>tolerance::kStationaryVelocity)
      fail(Action::Result::ERR_NOT_READY,range_error("Robot must be stationary [rad/s]",
          i,v,-tolerance::kStationaryVelocity,tolerance::kStationaryVelocity,0.0));
    if(std::abs(v)<=tolerance::kZeroVelocity) start.dq[i]=0.0;
    // Projection must not hide real outward motion at a joint boundary.
    if((start.q[i]<=limits[i].min_pos&&start.dq[i]<0.0)||
       (start.q[i]>=limits[i].max_pos&&start.dq[i]>0.0))
      fail(Action::Result::ERR_NOT_READY,
          "Joint"+std::to_string(i+1)+" must settle: velocity points outside joint range");
  }
  // Check the measured pose too, so normalization cannot conceal a collision.
  if(engine_.check_collision(initial.q)||engine_.check_collision(start.q))
    fail(Action::Result::ERR_LIMIT_VIOLATION,"Start configuration is in collision");
  check_cancelled(cancel);
  if(goal.command_type==Goal::MOVEJ||goal.command_type==Goal::MOVEP) {
    Goal target=goal;
    // Recovery/SysID alignment may send a snapshot of measured joint positions
    // back as the target. Apply the observation allowance only to such local
    // alignment requests, never to an arbitrary normal-motion target.
    bool observation_target=goal.command_type==Goal::MOVEJ&&
        (goal.context==Goal::CONTEXT_RECOVERY||goal.context==Goal::CONTEXT_SYSID);
    for(size_t i=0;i<6;++i)
      observation_target=observation_target&&std::isfinite(goal.joint_target[i])&&
          std::abs(goal.joint_target[i]-initial.q[i])<=tolerance::kObservedPosition;
    if(observation_target)
      target.joint_target=bounded_positions(goal.joint_target,limits,
          tolerance::kObservedPosition,Action::Result::ERR_LIMIT_VIOLATION,
          "Alignment joint target [rad]");
    return plan_ptp(target,start,cancel);
  }
  if(goal.command_type==Goal::MOVEL||goal.command_type==Goal::MOVEC)
    return plan_cartesian(goal,start,cancel);
  fail(Action::Result::ERR_INVALID_INPUT,"Unsupported motion command");
  return {};
}

trajectory_msgs::msg::JointTrajectory MotionPlanner::ruckig_ptp(
    const InitialState& initial,const std::array<double,6>& target,
    double vs,double as,const Cancellation& cancel) const {
  const auto bounded_target=bounded_positions(target,engine_.get_joint_limits(),
      tolerance::kJointPosition,Action::Result::ERR_LIMIT_VIOLATION,"PTP joint target [rad]");
  trajectory_msgs::msg::JointTrajectory msg;
  msg.joint_names={"joint1","joint2","joint3","joint4","joint5","joint6"};
  ruckig::InputParameter<6> input;
  for(size_t i=0;i<6;++i) {
    const auto& lim=engine_.get_joint_limits()[i];
    input.current_position[i]=initial.q[i];
    input.current_velocity[i]=initial.dq[i];
    input.current_acceleration[i]=0.0; // stationary guard, no measured ddq available
    input.target_position[i]=bounded_target[i];
    input.target_velocity[i]=0.0;
    input.target_acceleration[i]=0.0;
    input.max_velocity[i]=lim.max_vel*vs;
    input.max_acceleration[i]=lim.max_acc*as;
    input.max_jerk[i]=lim.max_jerk; // Exact YAML physical limit, never 10*acceleration
    if(std::abs(initial.dq[i])>input.max_velocity[i]+1e-9)
      fail(Action::Result::ERR_NOT_READY,"Initial velocity exceeds scaled limit");
  }
  ruckig::Ruckig<6> otg(kDt);
  ruckig::Trajectory<6> output;
  auto result=otg.calculate(input,output);
  if(static_cast<int>(result)<0) fail(Action::Result::ERR_RUCKIG_FAILED,"6-axis Ruckig calculation failed");
  const double duration=output.get_duration();
  if(!std::isfinite(duration)||duration>300.0||duration<0)
    fail(Action::Result::ERR_RUCKIG_FAILED,"Invalid Ruckig duration");
  auto record=[&](double t) {
    std::array<double,6> q{},v{},a{};
    output.at_time(t,q,v,a);
    msg.points.push_back(make_point(to_eigen(q),to_eigen(v),to_eigen(a),t));
  };
  if(duration<kTiny) {
    msg.points.push_back(make_point(to_eigen(initial.q),Vec6::Zero(),Vec6::Zero(),0.0));
    msg.points.push_back(make_point(to_eigen(bounded_target),Vec6::Zero(),Vec6::Zero(),kDt));
  } else {
    record(0.0); // preserve actual initial state
    for(double t=kDt;t<duration-1e-9;t+=kDt) {
      check_cancelled(cancel);
      if(msg.points.size()>kMaxPoints) fail(Action::Result::ERR_RUCKIG_FAILED,"Too many Ruckig samples");
      record(t);
    }
    record(duration); // use Ruckig's actual zero-velocity/acceleration endpoint
  }
  verify_trajectory(msg,true,cancel);
  return msg;
}

trajectory_msgs::msg::JointTrajectory MotionPlanner::plan_ptp(const Goal& goal,
    const InitialState& initial,const Cancellation& cancel) const {
  if(goal.command_type==Goal::MOVEJ)
    return ruckig_ptp(initial,goal.joint_target,goal.velocity_scale,goal.acceleration_scale,cancel);
  const Eigen::Matrix4d T=pose_matrix(goal.pose_target);
  const auto current=engine_.forward_pose(initial.q);
  if((current.topRightCorner<3,1>()-T.topRightCorner<3,1>()).norm()<1e-5&&
      angle_difference(current.topLeftCorner<3,3>(),T.topLeftCorner<3,3>())<1e-4)
    return ruckig_ptp(initial,initial.q,goal.velocity_scale,goal.acceleration_scale,cancel);
  std::vector<std::array<double,6>> solutions;
  const auto status=engine_.solve_optimal_ik(T,initial.q,solutions,false);
  if(status!=IKResult::SUCCESS) fail(ik_code(status),"MoveP goal has no valid IK solution");
  for(const auto& q:solutions) {
    check_cancelled(cancel);
    try {
      return ruckig_ptp(initial,q,goal.velocity_scale,goal.acceleration_scale,cancel);
    } catch(const MotionError& e) {
      if(e.code==Action::Result::ERR_CANCELLED) throw;
      if(e.code!=Action::Result::ERR_LIMIT_VIOLATION) throw;
      // Try the next analytic IK branch only if this route collides/violates limits.
    }
  }
  fail(Action::Result::ERR_LIMIT_VIOLATION,"All MoveP joint-space routes collide or violate limits");
  return {};
}

struct MotionPlanner::CartesianPath {
  Eigen::Vector3d p0,p1,center,xaxis,yaxis,axis_world;
  Eigen::Matrix3d R0;
  Eigen::Vector3d rotvec_local;
  double angle=0, radius=0, trans_length=0, rot_length=0;
  bool circular=false;
  Eigen::Matrix4d at(double s) const {
    Eigen::Matrix4d T=Eigen::Matrix4d::Identity();
    if(circular) {
      double theta=angle*s;
      T.topRightCorner<3,1>()=center+radius*(std::cos(theta)*xaxis+std::sin(theta)*yaxis);
    } else T.topRightCorner<3,1>()=p0+s*(p1-p0);
    const double rot_angle=rotvec_local.norm();
    if(rot_angle<1e-12) T.topLeftCorner<3,3>()=R0;
    else T.topLeftCorner<3,3>()=R0*Eigen::AngleAxisd(s*rot_angle,rotvec_local/rot_angle).toRotationMatrix();
    return T;
  }
  // Norm bounds of p',p''; angular derivative has norm rot_length.
  double path_first() const {return trans_length;}
  double path_second() const {return circular?radius*angle*angle:0.0;}
};

// Natural cubic spline over a uniformly sampled IK branch, C^2 in path variable s.
// Piecewise jerk is well-defined and bounded; geometric residual is checked afterwards.
struct MotionPlanner::JointSpline {
  struct Segment {Vec6 a,b,c,d;};
  std::vector<Segment> segments;
  double h=0;
  Vec6 q1max=Vec6::Zero(),q2max=Vec6::Zero(),q3max=Vec6::Zero();
  explicit JointSpline(const std::vector<std::array<double,6>>& knots) {
    const int n=static_cast<int>(knots.size())-1;
    if(n<1) throw std::runtime_error("Invalid IK knots");
    h=1.0/n;
    std::vector<Vec6> m(n+1,Vec6::Zero());
    // Thomas solve for natural spline second derivatives, in vector-valued RHS.
    std::vector<double> diag(n+1,0.0),upper(n+1,0.0);
    std::vector<Vec6> rhs(n+1,Vec6::Zero());
    for(int i=1;i<n;++i) {
      diag[i]=4.0; upper[i]=1.0;
      rhs[i]=6.0/(h*h)*(to_eigen(knots[i+1])-2*to_eigen(knots[i])+to_eigen(knots[i-1]));
    }
    for(int i=2;i<n;++i) {
      double w=1.0/diag[i-1]; diag[i]-=w*upper[i-1]; rhs[i]-=w*rhs[i-1];
    }
    for(int i=n-1;i>=1;--i) m[i]=(rhs[i]-upper[i]*m[i+1])/diag[i];
    segments.reserve(n);
    for(int i=0;i<n;++i) {
      const Vec6 qa=to_eigen(knots[i]),qb=to_eigen(knots[i+1]);
      Segment seg;
      seg.a=qa;
      seg.b=(qb-qa)/h-h*(2.0*m[i]+m[i+1])/6.0;
      seg.c=0.5*m[i];
      seg.d=(m[i+1]-m[i])/(6.0*h);
      for(int j=0;j<6;++j) {
        q1max[j]=std::max(q1max[j],std::abs(seg.b[j])+2*std::abs(seg.c[j])*h+3*std::abs(seg.d[j])*h*h);
        q2max[j]=std::max(q2max[j],2*std::abs(seg.c[j])+6*std::abs(seg.d[j])*h);
        q3max[j]=std::max(q3max[j],6*std::abs(seg.d[j]));
      }
      segments.push_back(seg);
    }
  }
  bool within_joint_limits(const std::array<JointLimit,6>& limits) const {
    // A cubic's extrema occur at its endpoints or roots of its derivative.
    for(const auto& x:segments) for(int j=0;j<6;++j) {
      auto within=[&](double u) {
        const double q=x.a[j]+u*(x.b[j]+u*(x.c[j]+u*x.d[j]));
        return std::isfinite(q)&&q>=limits[j].min_pos-tolerance::kJointPosition&&
            q<=limits[j].max_pos+tolerance::kJointPosition;
      };
      if(!within(0)||!within(h)) return false;
      const double a=3*x.d[j],b=2*x.c[j],c=x.b[j];
      if(std::abs(a)<1e-14) {
        if(std::abs(b)>1e-14) {
          const double r=-c/b;
          if(r>0&&r<h&&!within(r)) return false;
        }
      } else {
        const double discriminant=b*b-4*a*c;
        if(discriminant>=0) {
          const double root=std::sqrt(discriminant);
          for(const double r:{(-b-root)/(2*a),(-b+root)/(2*a)})
            if(r>0&&r<h&&!within(r)) return false;
        }
      }
    }
    return true;
  }
  void eval(double s,Vec6& q,Vec6& q_s,Vec6& q_ss) const {
    const int n=static_cast<int>(segments.size());
    const int i=std::clamp(static_cast<int>(std::floor(s*n)),0,n-1);
    const double u=std::clamp(s-i*h,0.0,h);
    const auto& x=segments[i];
    q=x.a+u*(x.b+u*(x.c+u*x.d));
    q_s=x.b+u*(2.0*x.c+3.0*u*x.d);
    q_ss=2.0*x.c+6.0*u*x.d;
  }
};

trajectory_msgs::msg::JointTrajectory MotionPlanner::plan_cartesian(const Goal& goal,
    const InitialState& initial,const Cancellation& cancel) const {
  CartesianPath path;
  const auto T0=engine_.forward_pose(initial.q);
  const auto T1=pose_matrix(goal.pose_target);
  path.R0=T0.topLeftCorner<3,3>();
  path.p0=T0.topRightCorner<3,1>(); path.p1=T1.topRightCorner<3,1>();
  const Eigen::AngleAxisd aa(path.R0.transpose()*T1.topLeftCorner<3,3>());
  path.rotvec_local=aa.angle()*aa.axis();
  path.rot_length=path.rotvec_local.norm();
  if(goal.command_type==Goal::MOVEC) {
    path.circular=true;
    const auto Ta=pose_matrix(goal.pose_aux);
    const Eigen::Vector3d p2=Ta.topRightCorner<3,1>();
    const Eigen::Vector3d u=p2-path.p0,v=path.p1-path.p0;
    const double L=std::max({u.norm(),v.norm(),(path.p1-p2).norm()});
    if(L<1e-5) fail(Action::Result::ERR_INVALID_INPUT,"Circular points coincide");
    Eigen::Vector3d normal=u.cross(v);
    if(normal.norm()<1e-5*L*L) fail(Action::Result::ERR_INVALID_INPUT,"Collinear circular points");
    normal.normalize();
    Eigen::Vector3d e1=u.normalized(),e2=normal.cross(e1);
    const double x=v.dot(e1),y=v.dot(e2);
    if(std::abs(y)<1e-8*L) fail(Action::Result::ERR_INVALID_INPUT,"Degenerate circle");
    const double cx=u.norm()/2.0;
    const double cy=(x*x+y*y-2*x*cx)/(2*y);
    path.center=path.p0+cx*e1+cy*e2;
    path.radius=(path.p0-path.center).norm();
    if(!(std::isfinite(path.radius)&&path.radius>1e-5))
      fail(Action::Result::ERR_INVALID_INPUT,"Invalid circle radius");
    path.xaxis=(path.p0-path.center)/path.radius;
    path.yaxis=normal.cross(path.xaxis);
    auto theta=[&](const Eigen::Vector3d& p){
      double t=std::atan2((p-path.center).dot(path.yaxis),(p-path.center).dot(path.xaxis));
      return t<0?t+2*kPi:t;
    };
    const double th_aux=theta(p2);
    double th_end=theta(path.p1);
    if(th_end+1e-9<th_aux) th_end+=2*kPi;
    if(th_aux<1e-6||th_end-th_aux<1e-6||th_end>2*kPi+1e-6)
      fail(Action::Result::ERR_INVALID_INPUT,"Auxiliary point not strictly on chosen arc");
    path.angle=th_end;
    path.trans_length=path.radius*path.angle;
    const Eigen::Vector3d check=path.at(th_aux/path.angle).topRightCorner<3,1>();
    if((check-p2).norm()>1e-6*L)
      fail(Action::Result::ERR_INVALID_INPUT,"Circle misses auxiliary point");
  } else path.trans_length=(path.p1-path.p0).norm();
  if(path.trans_length<1e-7&&path.rot_length<1e-7)
    return ruckig_ptp(initial,initial.q,goal.velocity_scale,goal.acceleration_scale,cancel);

  // Use sufficiently fine IK knots; refine on FK path residual instead of
  // claiming that a spline through sparse IK points is an exact Cartesian path.
  size_t samples=std::max<size_t>(8,static_cast<size_t>(std::ceil(std::max(
      path.trans_length/0.005,path.rot_length/0.025))));
  if(samples>kMaxIKSamples) fail(Action::Result::ERR_INVALID_INPUT,"Cartesian path too long for configured resolution");
  std::unique_ptr<JointSpline> spline;
  for(int refinement=0;refinement<4;++refinement) {
    check_cancelled(cancel);
    std::vector<std::array<double,6>> knots(samples+1);
    knots[0]=initial.q;
    for(size_t k=1;k<=samples;++k) {
      if(k%8==0) check_cancelled(cancel);
      std::vector<std::array<double,6>> solutions;
      const auto status=engine_.solve_optimal_ik(path.at(static_cast<double>(k)/samples),
          knots[k-1],solutions,true);
      if(status!=IKResult::SUCCESS) fail(ik_code(status),"Cartesian IK branch lost at path sample");
      knots[k]=solutions.front();
    }
    spline=std::make_unique<JointSpline>(knots);
    bool accurate=spline->within_joint_limits(engine_.get_joint_limits());
    for(size_t k=0;k<samples;++k) {
      if(k%16==0) check_cancelled(cancel);
      for(const double off:{0.25,0.5,0.75}) {
        const double s=(static_cast<double>(k)+off)/samples;
        Vec6 q,qs,qss; spline->eval(s,q,qs,qss);
        const auto qarr=from_eigen(q);
        if(!engine_.joint_valid(qarr)) {accurate=false;break;}
        const Eigen::Matrix4d fk=engine_.forward_pose(qarr), ref=path.at(s);
        if((fk.topRightCorner<3,1>()-ref.topRightCorner<3,1>()).norm()>kPathPosTolerance||
            angle_difference(fk.topLeftCorner<3,3>(),ref.topLeftCorner<3,3>())>kPathRotTolerance) {
          accurate=false;break;
        }
      }
      if(!accurate) break;
    }
    if(accurate) break;
    if(refinement==3||samples*2>kMaxIKSamples)
      fail(Action::Result::ERR_NO_IK_SOLUTION,"Spline cannot approximate Cartesian path within tolerance");
    samples*=2;
  }

  // Conservative bounds for q'=dq/ds, q''=d²q/ds², q'''=d³q/ds³.
  // qdot=q' sdot; qddot=q'' sdot²+q' sddot;
  // qjerk=q''' sdot³ + 3 q'' sdot sddot + q' sjerk.
  // Split each bound into conservative budgets; no arbitrary retry-count scaling.
  const auto& c=engine_.get_cartesian_limits();
  double V=std::numeric_limits<double>::infinity();
  double A=std::numeric_limits<double>::infinity();
  double J=std::numeric_limits<double>::infinity();
  const double vs=goal.velocity_scale,as=goal.acceleration_scale;
  if(path.trans_length>kTiny) {
    V=std::min(V,vs*c.max_trans_vel/path.path_first());
    A=std::min(A,0.45*as*c.max_trans_acc/path.path_first());
    if(path.path_second()>kTiny)
      V=std::min(V,std::sqrt(0.45*as*c.max_trans_acc/path.path_second()));
  }
  if(path.rot_length>kTiny) {
    V=std::min(V,vs*c.max_rot_vel/path.rot_length);
    A=std::min(A,as*c.max_rot_acc/path.rot_length);
  }
  for(int i=0;i<6;++i) {
    const auto& lim=engine_.get_joint_limits()[i];
    const double d1=spline->q1max[i],d2=spline->q2max[i],d3=spline->q3max[i];
    if(d1>kTiny) V=std::min(V,0.95*vs*lim.max_vel/d1);
    if(d2>kTiny) V=std::min(V,std::sqrt(0.45*as*lim.max_acc/d2));
    if(d3>kTiny) V=std::min(V,std::cbrt(0.30*lim.max_jerk/d3));
  }
  if(!std::isfinite(V)||V<=0) fail(Action::Result::ERR_RUCKIG_FAILED,"Cannot obtain positive path-speed bound");
  for(int i=0;i<6;++i) {
    const auto& lim=engine_.get_joint_limits()[i];
    const double d1=spline->q1max[i],d2=spline->q2max[i];
    if(d1>kTiny) {
      A=std::min(A,0.45*as*lim.max_acc/d1);
      J=std::min(J,0.30*lim.max_jerk/d1);
    }
    if(d2>kTiny) A=std::min(A,0.30*lim.max_jerk/(3*d2*V));
  }
  // For a pure stationary path the planner returns earlier, so these are finite.
  if(!std::isfinite(A)||A<=0||!std::isfinite(J)||J<=0)
    fail(Action::Result::ERR_RUCKIG_FAILED,"Cannot obtain positive acceleration/jerk bounds");
  ruckig::InputParameter<1> in;
  in.current_position[0]=0.0; in.current_velocity[0]=0.0; in.current_acceleration[0]=0.0;
  in.target_position[0]=1.0; in.target_velocity[0]=0.0; in.target_acceleration[0]=0.0;
  in.max_velocity[0]=V; in.max_acceleration[0]=A; in.max_jerk[0]=J;
  ruckig::Ruckig<1> otg(kDt);
  ruckig::Trajectory<1> scalar;
  const auto status=otg.calculate(in,scalar);
  if(static_cast<int>(status)<0) fail(Action::Result::ERR_RUCKIG_FAILED,"Cartesian path time-law Ruckig failed");
  const double duration=scalar.get_duration();
  if(!std::isfinite(duration)||duration<=0||duration>300.0)
    fail(Action::Result::ERR_RUCKIG_FAILED,"Invalid Cartesian time law duration");
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names={"joint1","joint2","joint3","joint4","joint5","joint6"};
  auto sample=[&](double t) {
    std::array<double,1> ss{},sv{},sa{};
    scalar.at_time(t,ss,sv,sa);
    Vec6 q,qs,qss;
    spline->eval(std::clamp(ss[0],0.0,1.0),q,qs,qss);
    const Vec6 dq=qs*sv[0];
    const Vec6 ddq=qss*sv[0]*sv[0]+qs*sa[0];
    traj.points.push_back(make_point(q,dq,ddq,t));
  };
  sample(0);
  for(double t=kDt;t<duration-1e-9;t+=kDt) {
    check_cancelled(cancel);
    if(traj.points.size()>=kMaxPoints) fail(Action::Result::ERR_RUCKIG_FAILED,"Trajectory exceeds point budget");
    sample(t);
  }
  sample(duration);
  verify_trajectory(traj,true,cancel);
  return traj;
}

void MotionPlanner::verify_trajectory(const trajectory_msgs::msg::JointTrajectory& traj,
    bool jerk,const Cancellation& cancel) const {
  const std::vector<std::string> names={"joint1","joint2","joint3","joint4","joint5","joint6"};
  if(traj.joint_names!=names||traj.points.size()<2)
    fail(Action::Result::ERR_INVALID_INPUT,"Invalid six-axis joint-name/order or point count");
  if(traj.points.size()>kMaxPoints)
    fail(Action::Result::ERR_INVALID_INPUT,"Trajectory exceeds point limit");
  double prev_time=-1.0;
  std::array<double,6> prev_q{},prev_acc{};
  for(size_t k=0;k<traj.points.size();++k) {
    if(k%32==0) check_cancelled(cancel);
    const auto& p=traj.points[k];
    if(p.positions.size()!=6||p.velocities.size()!=6||p.accelerations.size()!=6)
      fail(Action::Result::ERR_INVALID_INPUT,"Incomplete joint trajectory point");
    const double time=seconds(p.time_from_start);
    if(!std::isfinite(time)||time<=prev_time+1e-12)
      fail(Action::Result::ERR_INVALID_INPUT,"Non-monotone trajectory timestamps");
    std::array<double,6> q{};
    for(size_t i=0;i<6;++i) {
      q[i]=p.positions[i];
      const auto& lim=engine_.get_joint_limits()[i];
      if(!std::isfinite(q[i])||!std::isfinite(p.velocities[i])||!std::isfinite(p.accelerations[i]))
        fail(Action::Result::ERR_INVALID_INPUT,"NaN/Inf in trajectory");
      if(q[i]<lim.min_pos-tolerance::kTrajectoryPosition||
         q[i]>lim.max_pos+tolerance::kTrajectoryPosition)
        fail(Action::Result::ERR_LIMIT_VIOLATION,range_error(
            "Trajectory position [rad] at t="+std::to_string(time),i,q[i],
            lim.min_pos,lim.max_pos,tolerance::kTrajectoryPosition));
      if(std::abs(p.velocities[i])>lim.max_vel+tolerance::kTrajectoryVelocity)
        fail(Action::Result::ERR_LIMIT_VIOLATION,range_error(
            "Trajectory velocity [rad/s] at t="+std::to_string(time),i,p.velocities[i],
            -lim.max_vel,lim.max_vel,tolerance::kTrajectoryVelocity));
      if(std::abs(p.accelerations[i])>lim.max_acc+tolerance::kTrajectoryAcceleration)
        fail(Action::Result::ERR_LIMIT_VIOLATION,range_error(
            "Trajectory acceleration [rad/s^2] at t="+std::to_string(time),i,p.accelerations[i],
            -lim.max_acc,lim.max_acc,tolerance::kTrajectoryAcceleration));
      // Mean acceleration difference is only a discrete diagnostic. Continuous
      // jerk is bounded analytically by Ruckig/PTP or spline derivatives/time law.
      if(jerk&&k>0&&std::abs((p.accelerations[i]-prev_acc[i])/(time-prev_time))>
                        lim.max_jerk*1.001+1e-3)
        fail(Action::Result::ERR_LIMIT_VIOLATION,"Joint jerk check failed");
      prev_acc[i]=p.accelerations[i];
    }
    if(k==0) {
      if(engine_.check_collision(q)) fail(Action::Result::ERR_LIMIT_VIOLATION,"Collision at start");
    } else {
      double biggest=0;
      for(int j=0;j<6;++j) biggest=std::max(biggest,std::abs(q[j]-prev_q[j]));
      const int pieces=std::max(1,static_cast<int>(std::ceil(biggest/0.04)));
      for(int z=1;z<=pieces;++z) {
        std::array<double,6> mid{};
        const double s=static_cast<double>(z)/pieces;
        for(int j=0;j<6;++j) mid[j]=(1-s)*prev_q[j]+s*q[j];
        if(engine_.check_collision(mid)) fail(Action::Result::ERR_LIMIT_VIOLATION,
          "Trajectory collision (joint-space subdivision)");
      }
    }
    prev_q=q;
    prev_time=time;
  }
}

} // namespace lite6_planner
