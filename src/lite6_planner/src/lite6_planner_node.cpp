#include "lite6_planner/kinematics_engine.hpp"
#include "lite6_planner/motion_planner.hpp"
#include "lite6_planner/supervisor.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <std_msgs/msg/int32.hpp>
#include <lite6_interfaces/action/industrial_motion.hpp>
#include <lite6_interfaces/msg/controller_status.hpp>
#include <lite6_interfaces/msg/supervisor_state.hpp>
#include <lite6_interfaces/msg/motion_rejection.hpp>
#include <lite6_interfaces/srv/supervisor_command.hpp>
#include <lite6_interfaces/msg/compliance_command.hpp>
#include <lite6_interfaces/compliance_validation.hpp>
#include <lite6_interfaces/shm_registers.hpp>
#include <lite6_interfaces/srv/submit_sys_id_trajectory.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

using namespace std::chrono_literals;
namespace lite6_planner {
namespace {
using Clock=std::chrono::steady_clock;
using Motion=lite6_interfaces::action::IndustrialMotion;
using FJT=control_msgs::action::FollowJointTrajectory;
using FJTHandle=rclcpp_action::ClientGoalHandle<FJT>;
using MotionHandle=rclcpp_action::ServerGoalHandle<Motion>;
using S=lite6_interfaces::msg::SupervisorState;
std::string hex_uuid(const rclcpp_action::GoalUUID& id) {
  std::ostringstream stream;
  stream<<std::hex<<std::setfill('0');
  for(auto byte:id) stream<<std::setw(2)<<static_cast<unsigned>(byte);
  return stream.str();
}
double sec(const builtin_interfaces::msg::Duration& d) {
  return static_cast<double>(d.sec)+1e-9*d.nanosec;
}
}

class IndustrialPlannerNode final:public rclcpp::Node {
public:
  IndustrialPlannerNode():rclcpp::Node("lite6_industrial_planner"),planner_(kinematics_) {
    const auto desc=ament_index_cpp::get_package_share_directory("lite6_description");
    const auto pkg=ament_index_cpp::get_package_share_directory("lite6_planner");
    if(!kinematics_.load_ik_config(pkg+"/config/ik_config.yaml")||
       !kinematics_.init_robot_model(desc+"/urdf/lite6_arm.urdf",pkg+"/config/lite6.srdf")||
       !kinematics_.load_limits(pkg+"/config/kinematic_limits.yaml"))
      throw std::runtime_error("Robot planning model / YAML initialization failed");
    io_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    validation_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    const auto state_qos=rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    state_pub_=create_publisher<S>("/supervisor/state",state_qos);
    rejection_pub_=create_publisher<lite6_interfaces::msg::MotionRejection>(
      "/supervisor/motion_rejections",rclcpp::QoS(32).reliable());
    cmd_pub_=create_publisher<std_msgs::msg::Int32>("/ctc_controller/system_cmd",10);
    compliance_pub_=create_publisher<lite6_interfaces::msg::ComplianceCommand>(
      "/ctc_controller/compliance_cmd",10);
    rclcpp::SubscriptionOptions sensor_options;
    sensor_options.callback_group=io_group_;
    controller_sub_=create_subscription<lite6_interfaces::msg::ControllerStatus>(
      "/ctc_controller/status",rclcpp::QoS(10),
      [this](const lite6_interfaces::msg::ControllerStatus::SharedPtr msg) {
        const auto stamp=rclcpp::Time(msg->stamp).nanoseconds();
        if(stamp<=last_controller_stamp_) return;
        last_controller_stamp_=stamp;
        {
          std::lock_guard<std::mutex> transition(transition_mutex_);
          do_effects(supervisor_.observe_controller(*msg));
        }
        {
          std::lock_guard<std::mutex> lock(ctc_mutex_);
          if(operation_&&operation_->downstream&&
              operation_->downstream->get_goal_id()==msg->trajectory_id)
            supervisor_.observe_excitation_start(operation_->key,msg->trajectory_start);
        }
        publish_state();
      },sensor_options);
    if(!state_shm_.open(false))
      throw std::runtime_error("Failed to open read-only state shared memory");
    joint_poll_timer_=create_wall_timer(10ms,[this]{poll_joint_state_shm();},io_group_);
    poll_joint_state_shm();
    ctc_=rclcpp_action::create_client<FJT>(this,"/ctc_controller/follow_joint_trajectory");
    action_server_=rclcpp_action::create_server<Motion>(this,"industrial_motion",
      [this](const rclcpp_action::GoalUUID& id,std::shared_ptr<const Motion::Goal> goal){
        return on_goal(id,std::move(goal));
      },
      [this](const std::shared_ptr<MotionHandle>& gh){return on_cancel(gh);},
      [this](const std::shared_ptr<MotionHandle>& gh){on_accepted(gh);});
    command_service_=create_service<lite6_interfaces::srv::SupervisorCommand>(
      "/supervisor/command",
      [this](const std::shared_ptr<lite6_interfaces::srv::SupervisorCommand::Request> req,
             std::shared_ptr<lite6_interfaces::srv::SupervisorCommand::Response> resp) {
        std::lock_guard<std::mutex> transition(transition_mutex_);
        const bool compliance=req->command>=8&&req->command<=10;
        if(compliance&&!lite6_interfaces::valid_compliance_settings(req->compliance)) {
          resp->accepted=false;resp->message="Invalid compliance gains / sensitivity";return;
        }
        auto reply=supervisor_.command(req->command,req->session_id,req->reason);
        if(reply.accepted&&compliance) {
          lite6_interfaces::msg::ComplianceCommand msg;
          msg.command=reply.effects.ctc_command;msg.settings=req->compliance;
          compliance_pub_->publish(msg);
          reply.effects.ctc_command=-1;
        }
        resp->accepted=reply.accepted;resp->session_id=reply.session_id;
        resp->revision=reply.revision;resp->message=reply.message;
        do_effects(reply.effects);publish_state();
      });
    sysid_service_=create_service<lite6_interfaces::srv::SubmitSysIdTrajectory>(
      "/supervisor/submit_sysid_trajectory",
      [this](const std::shared_ptr<lite6_interfaces::srv::SubmitSysIdTrajectory::Request> req,
             std::shared_ptr<lite6_interfaces::srv::SubmitSysIdTrajectory::Response> resp){
        on_sysid_trajectory(*req,*resp);
      },rmw_qos_profile_services_default,validation_group_);
    heartbeat_=create_wall_timer(100ms,[this]{on_heartbeat();},io_group_);
    RCLCPP_INFO(get_logger(),"Supervisor-owned Robot planner created (awaiting CTC status)");
  }
  ~IndustrialPlannerNode() override {
    shutdown_.store(true);
    if(planner_thread_.joinable()) planner_thread_.join();
  }

private:
  struct CTCOperation {
    std::string key;
    std::shared_ptr<MotionHandle> upper;
    FJTHandle::SharedPtr downstream;
    Clock::time_point deadline;
    Clock::time_point dispatch_time;
    double duration=0.0;
    bool cancel_pending=false;
  };
  KinematicsEngine kinematics_;
  MotionPlanner planner_;
  Supervisor supervisor_;
  std::mutex joints_mutex_,ctc_mutex_,planning_model_mutex_;
  // Serialize a state transition with its mode-command publication. Otherwise
  // a pending NORMAL command could be published after a concurrent ESTOP.
  std::mutex transition_mutex_;
  int64_t last_controller_stamp_=0;
  uint64_t last_joint_seq_=0;
  int64_t last_joint_steady_ns_=0;
  MotionPlanner::InitialState actual_{};
  std::unique_ptr<CTCOperation> operation_;
  std::thread planner_thread_;
  rclcpp::CallbackGroup::SharedPtr io_group_;
  rclcpp::CallbackGroup::SharedPtr validation_group_;
  std::atomic<bool> shutdown_{false};
  lite6_interfaces::shm::ShmHandle<lite6_interfaces::shm::StateShmSegment> state_shm_;
  rclcpp::TimerBase::SharedPtr joint_poll_timer_;
  rclcpp::Publisher<lite6_interfaces::msg::ComplianceCommand>::SharedPtr compliance_pub_;
  rclcpp::Publisher<S>::SharedPtr state_pub_;
  rclcpp::Publisher<lite6_interfaces::msg::MotionRejection>::SharedPtr rejection_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr cmd_pub_;
  rclcpp::Subscription<lite6_interfaces::msg::ControllerStatus>::SharedPtr controller_sub_;
  rclcpp_action::Client<FJT>::SharedPtr ctc_;
  rclcpp_action::Server<Motion>::SharedPtr action_server_;
  rclcpp::Service<lite6_interfaces::srv::SupervisorCommand>::SharedPtr command_service_;
  rclcpp::Service<lite6_interfaces::srv::SubmitSysIdTrajectory>::SharedPtr sysid_service_;
  rclcpp::TimerBase::SharedPtr heartbeat_;

  void publish_state() {
    auto state=supervisor_.snapshot();
    state.stamp=get_clock()->now();
    state_pub_->publish(state);
  }
  bool read_shm_joints(MotionPlanner::InitialState& out, double& max_speed,
      uint64_t& out_seq, int64_t& out_steady_ns) const {
    if(!state_shm_.valid()) return false;
    lite6_interfaces::shm::JointStatePod pod{};
    if(!state_shm_->joint_state.read(pod,out_seq)||pod.valid==0) return false;
    const int64_t now_ns=lite6_interfaces::shm::steady_now_ns();
    if(pod.steady_stamp_ns<=0||now_ns-pod.steady_stamp_ns>=800000000LL) return false;
    max_speed=0.0;
    for(int i=0;i<6;++i) {
      out.q[i]=pod.position[i];out.dq[i]=pod.velocity[i];
      if(!std::isfinite(out.q[i])||!std::isfinite(out.dq[i])) return false;
      max_speed=std::max(max_speed,std::abs(out.dq[i]));
    }
    out_steady_ns=pod.steady_stamp_ns;
    return true;
  }
  void poll_joint_state_shm() {
    MotionPlanner::InitialState candidate;
    double max_speed=0.0;
    uint64_t seq=0;
    int64_t steady_ns=0;
    if(!read_shm_joints(candidate,max_speed,seq,steady_ns)) return;
    if(seq==last_joint_seq_||steady_ns<=last_joint_steady_ns_) return;
    {
      std::lock_guard<std::mutex> lock(joints_mutex_);
      actual_=candidate;
    }
    last_joint_seq_=seq;
    last_joint_steady_ns_=steady_ns;
    supervisor_.observe_joint_state(max_speed);
  }
  void issue_ctc_command(int value) {
    std_msgs::msg::Int32 msg;
    msg.data=value;
    cmd_pub_->publish(msg);
  }
  void do_effects(const Supervisor::Effects& effects) {
    // CTC ESTOP is requested immediately, before slower cancellation paths.
    if(effects.ctc_command>=0) issue_ctc_command(effects.ctc_command);
    if(effects.cancel_active) cancel_ctc(effects.cancel_key);
  }
  void cancel_ctc(const std::string& key) {
    FJTHandle::SharedPtr downstream;
    {
      std::lock_guard<std::mutex> lock(ctc_mutex_);
      if(!operation_||operation_->key!=key) return;
      operation_->cancel_pending=true;
      downstream=operation_->downstream;
    }
    if(downstream) {
      try {ctc_->async_cancel_goal(downstream);}
      catch(const std::exception& e) {RCLCPP_WARN(get_logger(),"CTC cancel: %s",e.what());}
    }
  }
  rclcpp_action::GoalResponse on_goal(const rclcpp_action::GoalUUID& id,
      std::shared_ptr<const Motion::Goal> goal) {
    const auto key=hex_uuid(id);
    auto reject=[&](int32_t code,const std::string& reason) {
      lite6_interfaces::msg::MotionRejection message;
      message.goal_id=key;message.error_code=code;message.error_string=reason;
      rejection_pub_->publish(message);
      RCLCPP_WARN(get_logger(),"Goal %s rejected: %s",key.c_str(),reason.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    };
    if(goal->command_type>Motion::Goal::MOVEC)
      return reject(Motion::Result::ERR_INVALID_INPUT,"Unsupported motion command_type="+
          std::to_string(goal->command_type));
    if(!std::isfinite(goal->velocity_scale)||
       !std::isfinite(goal->acceleration_scale)||goal->velocity_scale<0.01||
       goal->velocity_scale>1.0||goal->acceleration_scale<0.01||
       goal->acceleration_scale>1.0)
      return reject(Motion::Result::ERR_INVALID_INPUT,"Scaling factors must be finite and in [0.01, 1]; velocity_scale="+
          std::to_string(goal->velocity_scale)+", acceleration_scale="+std::to_string(goal->acceleration_scale));
    if(goal->context!=Motion::Goal::CONTEXT_NORMAL&&
       goal->context!=Motion::Goal::CONTEXT_SYSID&&
       goal->context!=Motion::Goal::CONTEXT_RECOVERY)
      return reject(Motion::Result::ERR_INVALID_INPUT,"Unknown motion context="+std::to_string(goal->context));
    if(goal->context!=Motion::Goal::CONTEXT_NORMAL&&goal->command_type!=Motion::Goal::MOVEJ)
      return reject(Motion::Result::ERR_SUPERVISOR_DENIED,"SYSID / RECOVERY context only permits MoveJ reference alignment");
    std::string reason;
    if(!supervisor_.reserve_motion(key,goal->context,goal->session_id,reason)) {
      return reject(Motion::Result::ERR_SUPERVISOR_DENIED,reason);
    }
    publish_state();
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse on_cancel(const std::shared_ptr<MotionHandle>& gh) {
    const auto key=hex_uuid(gh->get_goal_id());
    if(!supervisor_.request_cancel(key)) return rclcpp_action::CancelResponse::REJECT;
    cancel_ctc(key);publish_state();
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void on_accepted(const std::shared_ptr<MotionHandle>& gh) {
    // There is only one admitted goal, hence the previous planning thread has
    // finished before a new admission. Thread is joined, not detached.
    if(planner_thread_.joinable()) planner_thread_.join();
    planner_thread_=std::thread([this,gh]{compute_and_dispatch(gh);});
  }
  void compute_and_dispatch(const std::shared_ptr<MotionHandle>& gh) {
    const auto key=hex_uuid(gh->get_goal_id());
    try {
      MotionPlanner::InitialState start;
      double dummy_speed=0.0; uint64_t dummy_seq=0; int64_t dummy_ns=0;
      if(!read_shm_joints(start,dummy_speed,dummy_seq,dummy_ns)) {
        std::lock_guard<std::mutex> lock(joints_mutex_);
        start=actual_;
      }
      if(shutdown_.load()||!supervisor_.permitted(key))
        throw MotionError(Motion::Result::ERR_CANCELLED,"Supervisor withdrew authorization");
      trajectory_msgs::msg::JointTrajectory traj;
      {
        std::lock_guard<std::mutex> lock(planning_model_mutex_);
        traj=planner_.plan(*gh->get_goal(),start,[this,key]{
          return shutdown_.load()||!supervisor_.permitted(key);
        });
      }
      if(!supervisor_.permitted(key))
        throw MotionError(Motion::Result::ERR_CANCELLED,"Goal cancelled during planning");
      const auto feedback=std::make_shared<Motion::Feedback>();
      feedback->current_state=Motion::Feedback::STATE_WAITING_CTC;
      feedback->percent_complete=0;
      gh->publish_feedback(feedback);
      dispatch_ctc(key,traj,gh);
    } catch(const MotionError& error) {
      finish_upper(key,gh,error.code,error.what(),0);
    } catch(const std::exception& error) {
      finish_upper(key,gh,Motion::Result::ERR_INTERNAL,error.what(),0);
    }
  }
  void dispatch_ctc(const std::string& key,
      const trajectory_msgs::msg::JointTrajectory& trajectory,
      std::shared_ptr<MotionHandle> upper) {
    if(!supervisor_.permitted(key)) {
      finish_upper(key,upper,Motion::Result::ERR_CANCELLED,"Authorization revoked",0);return;
    }
    if(!ctc_->wait_for_action_server(2s)) {
      finish_upper(key,upper,Motion::Result::ERR_EXECUTION_ABORTED,"CTC unavailable",0);return;
    }
    if(!supervisor_.change_motion_phase(key,S::MOTION_WAITING_CTC)) {
      finish_upper(key,upper,Motion::Result::ERR_CANCELLED,"Motion canceled before CTC dispatch",0);return;
    }
    const double duration=sec(trajectory.points.back().time_from_start);
    {
      std::lock_guard<std::mutex> lock(ctc_mutex_);
      operation_=std::make_unique<CTCOperation>();
      operation_->key=key;operation_->upper=upper;
      operation_->duration=duration;
      operation_->dispatch_time=Clock::now();
      operation_->deadline=Clock::now()+std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(duration+12.0));
    }
    // Cancellation may have arrived after the initial authorization check,
    // while the downstream goal slot was being prepared.
    if(!supervisor_.permitted(key)) {
      finish_ctc(key,Motion::Result::ERR_CANCELLED,"Authorization revoked before CTC send");
      return;
    }
    FJT::Goal g;g.trajectory=trajectory;
    rclcpp_action::Client<FJT>::SendGoalOptions options;
    options.goal_response_callback=[this,key](const FJTHandle::SharedPtr& downstream) {
      if(!downstream) {
        finish_ctc(key,Motion::Result::ERR_EXECUTION_ABORTED,"CTC rejected goal");return;
      }
      bool should_cancel=false;
      {
        std::lock_guard<std::mutex> lock(ctc_mutex_);
        if(!operation_||operation_->key!=key) should_cancel=true;
        else {
          operation_->downstream=downstream;
          should_cancel=operation_->cancel_pending;
        }
      }
      if(should_cancel||!supervisor_.permitted(key)) {
        try {ctc_->async_cancel_goal(downstream);}
        catch(const std::exception& e) {RCLCPP_WARN(get_logger(),"CTC cancellation: %s",e.what());}
      }
    };
    options.feedback_callback=[this,key](FJTHandle::SharedPtr,
        const std::shared_ptr<const FJT::Feedback> feedback) {
      std::shared_ptr<MotionHandle> upper_goal;
      double duration=0;
      {
        std::lock_guard<std::mutex> lock(ctc_mutex_);
        if(!operation_||operation_->key!=key) return;
        upper_goal=operation_->upper;duration=operation_->duration;
      }
      supervisor_.change_motion_phase(key,S::MOTION_EXECUTING);
      if(upper_goal&&upper_goal->is_active()) {
        auto fb=std::make_shared<Motion::Feedback>();
        fb->current_state=Motion::Feedback::STATE_EXECUTING;
        fb->percent_complete=duration>0?
          std::clamp(100.0*sec(feedback->desired.time_from_start)/duration,0.0,99.9):0.0;
        upper_goal->publish_feedback(fb);
      }
    };
    options.result_callback=[this,key](const FJTHandle::WrappedResult& wrapped) {
      const std::string detail="CTC action status="+std::to_string(static_cast<int>(wrapped.code))+
          (wrapped.result?", error_code="+std::to_string(wrapped.result->error_code)+
              ": "+wrapped.result->error_string:", result payload unavailable");
      if(wrapped.code==rclcpp_action::ResultCode::SUCCEEDED&&wrapped.result&&
         wrapped.result->error_code==FJT::Result::SUCCESSFUL)
        finish_ctc(key,Motion::Result::SUCCESS,"CTC trajectory completed");
      else if(wrapped.code==rclcpp_action::ResultCode::CANCELED)
        finish_ctc(key,Motion::Result::ERR_CANCELLED,"CTC trajectory canceled\n"+detail);
      else finish_ctc(key,Motion::Result::ERR_EXECUTION_ABORTED,"CTC execution failed\n"+detail);
    };
    try {ctc_->async_send_goal(g,options);}
    catch(const std::exception& e) {
      finish_ctc(key,Motion::Result::ERR_EXECUTION_ABORTED,e.what());
    }
    publish_state();
  }
  void finish_upper(const std::string& key,const std::shared_ptr<MotionHandle>& gh,
      int32_t code,const std::string& message,double duration) {
    Supervisor::Effects effects;
    {
      std::lock_guard<std::mutex> transition(transition_mutex_);
      effects=supervisor_.finish_motion(key,code==Motion::Result::SUCCESS,code,message);
      do_effects(effects);
    }
    auto result=std::make_shared<Motion::Result>();
    result->error_code=effects.terminal_success?Motion::Result::SUCCESS:
      (code==Motion::Result::SUCCESS?Motion::Result::ERR_EXECUTION_ABORTED:code);
    result->error_string=message;
    result->execution_time=duration;
    if(gh&&gh->is_active()) {
      if(effects.terminal_success) gh->succeed(result);
      else if(gh->is_canceling()) gh->canceled(result);
      else gh->abort(result);
    }
    publish_state();
  }
  void finish_ctc(const std::string& key,int32_t code,const std::string& message) {
    std::shared_ptr<MotionHandle> upper;
    double elapsed=0;
    {
      std::lock_guard<std::mutex> lock(ctc_mutex_);
      if(!operation_||operation_->key!=key) return; // stale callback / timed out
      upper=operation_->upper;
      elapsed=std::chrono::duration<double>(Clock::now()-operation_->dispatch_time).count();
      operation_.reset();
    }
    if(upper) finish_upper(key,upper,code,message,elapsed);
    else {
      std::lock_guard<std::mutex> transition(transition_mutex_);
      const auto effects=supervisor_.finish_motion(key,code==Motion::Result::SUCCESS,
          code,message);
      do_effects(effects);publish_state();
    }
  }
  void on_sysid_trajectory(const lite6_interfaces::srv::SubmitSysIdTrajectory::Request& req,
      lite6_interfaces::srv::SubmitSysIdTrajectory::Response& resp) {
    std::string key,why;
    if(!supervisor_.begin_excitation_validation(req.session_id,why)) {
      resp.accepted=false;resp.message=why;
      resp.revision=supervisor_.snapshot().revision;return;
    }
    publish_state();
    // Received work has its own bounded validation deadline. Heartbeat and
    // cancellation checks remain active; CTC has not received any motion yet.
    try {
      const auto authorized=[this,&req] {
        const auto state=supervisor_.snapshot();
        return !shutdown_.load()&&state.mode==S::MODE_SYSID&&
            state.session_id==req.session_id&&state.sysid_phase==S::SYSID_VALIDATING&&
            !state.waiting_for_settle;
      };
      if(!authorized()) throw std::runtime_error("SysID session is no longer ready");
      std::lock_guard<std::mutex> lock(planning_model_mutex_);
      planner_.verify_trajectory(req.trajectory,true,[&authorized]{return !authorized();});
      for(size_t tick=0;tick<req.trajectory.points.size();++tick) {
        const auto& t=req.trajectory.points[tick].time_from_start;
        if(static_cast<int64_t>(t.sec)*1000000000LL+t.nanosec!=
            static_cast<int64_t>(tick)*1000000LL)
          throw std::runtime_error("SysID references must use an exact 1 ms grid starting at zero");
      }
    } catch(const std::exception& e) {
      supervisor_.end_excitation_validation(req.session_id);
      resp.accepted=false;resp.message=std::string("SysID trajectory rejected: ")+e.what();
      resp.revision=supervisor_.snapshot().revision;return;
    }
    // The Fourier reference must start at the current physical pose; no jump.
    {
      MotionPlanner::InitialState current;
      double dummy_speed=0.0; uint64_t dummy_seq=0; int64_t dummy_ns=0;
      if(!read_shm_joints(current,dummy_speed,dummy_seq,dummy_ns)) {
        std::lock_guard<std::mutex> lock(joints_mutex_);
        current=actual_;
      }
      for(int i=0;i<6;++i) {
        if(std::abs(req.trajectory.points.front().positions[i]-current.q[i])>0.05||
           std::abs(current.dq[i])>0.02) {
          resp.accepted=false;
          resp.message="SysID reference start differs from stationary physical state";
          supervisor_.end_excitation_validation(req.session_id);
          resp.revision=supervisor_.snapshot().revision;
          return;
        }
      }
    }
    if(!supervisor_.reserve_excitation(req.session_id,key,why)) {
      supervisor_.end_excitation_validation(req.session_id);
      resp.accepted=false;resp.message=why;
      resp.revision=supervisor_.snapshot().revision;return;
    }
    resp.accepted=true;
    resp.revision=supervisor_.snapshot().revision;
    resp.message="Trajectory authorized and submitted to CTC; monitor supervisor state for completion";
    dispatch_ctc(key,req.trajectory,nullptr);
  }
  void on_heartbeat() {
    {
      std::lock_guard<std::mutex> transition(transition_mutex_);
      do_effects(supervisor_.watchdog());
    }
    std::string timed_out;
    {
      std::lock_guard<std::mutex> lock(ctc_mutex_);
      if(operation_&&Clock::now()>operation_->deadline) timed_out=operation_->key;
    }
    if(!timed_out.empty()) {
      cancel_ctc(timed_out);
      finish_ctc(timed_out,Motion::Result::ERR_TIMEOUT,"CTC goal/result timeout");
    }
    publish_state();
  }
};
} // namespace lite6_planner

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<lite6_planner::IndustrialPlannerNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),4);
    executor.add_node(node);
    executor.spin();
  } catch(const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("lite6_planner"),"Planner startup failed: %s",e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
