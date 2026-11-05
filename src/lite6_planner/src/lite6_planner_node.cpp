#include "lite6_planner/kinematics_engine.hpp"
#include "lite6_planner/motion_planner.hpp"
#include "lite6_planner/supervisor.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32.hpp>
#include <lite6_interfaces/action/industrial_motion.hpp>
#include <lite6_interfaces/msg/controller_status.hpp>
#include <lite6_interfaces/msg/supervisor_state.hpp>
#include <lite6_interfaces/srv/supervisor_command.hpp>
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
       !kinematics_.init_robot_model(desc+"/urdf/lite6.urdf",pkg+"/config/lite6.srdf")||
       !kinematics_.load_limits(pkg+"/config/kinematic_limits.yaml"))
      throw std::runtime_error("Lite6 planning model / YAML initialization failed");
    io_group_=create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    const auto state_qos=rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    state_pub_=create_publisher<S>("/supervisor/state",state_qos);
    cmd_pub_=create_publisher<std_msgs::msg::Int32>("/ctc_controller/system_cmd",10);
    rclcpp::SubscriptionOptions sensor_options;
    sensor_options.callback_group=io_group_;
    controller_sub_=create_subscription<lite6_interfaces::msg::ControllerStatus>(
      "/ctc_controller/status",rclcpp::QoS(10),
      [this](const lite6_interfaces::msg::ControllerStatus::SharedPtr msg) {
        do_effects(supervisor_.observe_controller(msg->mode));
        publish_state();
      },sensor_options);
    joints_sub_=create_subscription<sensor_msgs::msg::JointState>("/joint_states",rclcpp::QoS(10),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg){on_joints(*msg);},sensor_options);
    jtc_=rclcpp_action::create_client<FJT>(this,"/lite6_arm_controller/follow_joint_trajectory");
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
        auto reply=supervisor_.command(req->command,req->session_id,req->reason);
        resp->accepted=reply.accepted;resp->session_id=reply.session_id;
        resp->revision=reply.revision;resp->message=reply.message;
        do_effects(reply.effects);publish_state();
      });
    sysid_service_=create_service<lite6_interfaces::srv::SubmitSysIdTrajectory>(
      "/supervisor/submit_sysid_trajectory",
      [this](const std::shared_ptr<lite6_interfaces::srv::SubmitSysIdTrajectory::Request> req,
             std::shared_ptr<lite6_interfaces::srv::SubmitSysIdTrajectory::Response> resp){
        on_sysid_trajectory(*req,*resp);
      });
    heartbeat_=create_wall_timer(100ms,[this]{on_heartbeat();},io_group_);
    RCLCPP_INFO(get_logger(),"Supervisor-owned Lite6 planner created (awaiting CTC status)");
  }
  ~IndustrialPlannerNode() override {
    shutdown_.store(true);
    if(planner_thread_.joinable()) planner_thread_.join();
  }

private:
  struct JTCOperation {
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
  std::mutex joints_mutex_,jtc_mutex_;
  MotionPlanner::InitialState actual_{};
  std::string current_accepted_key_;
  std::unique_ptr<JTCOperation> operation_;
  std::thread planner_thread_;
  rclcpp::CallbackGroup::SharedPtr io_group_;
  std::atomic<bool> shutdown_{false};
  rclcpp::Publisher<S>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr cmd_pub_;
  rclcpp::Subscription<lite6_interfaces::msg::ControllerStatus>::SharedPtr controller_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp_action::Client<FJT>::SharedPtr jtc_;
  rclcpp_action::Server<Motion>::SharedPtr action_server_;
  rclcpp::Service<lite6_interfaces::srv::SupervisorCommand>::SharedPtr command_service_;
  rclcpp::Service<lite6_interfaces::srv::SubmitSysIdTrajectory>::SharedPtr sysid_service_;
  rclcpp::TimerBase::SharedPtr heartbeat_;

  void publish_state() {
    auto state=supervisor_.snapshot();
    state.stamp=get_clock()->now();
    state_pub_->publish(state);
  }
  void on_joints(const sensor_msgs::msg::JointState& msg) {
    MotionPlanner::InitialState candidate;
    for(int i=0;i<6;++i) {
      const auto name="joint"+std::to_string(i+1);
      auto it=std::find(msg.name.begin(),msg.name.end(),name);
      if(it==msg.name.end()) return;
      const auto index=std::distance(msg.name.begin(),it);
      if(index<0||static_cast<size_t>(index)>=msg.position.size()||
         static_cast<size_t>(index)>=msg.velocity.size()) return;
      candidate.q[i]=msg.position[index];candidate.dq[i]=msg.velocity[index];
      if(!std::isfinite(candidate.q[i])||!std::isfinite(candidate.dq[i])) return;
    }
    {
      std::lock_guard<std::mutex> lock(joints_mutex_);
      actual_=candidate;
    }
    supervisor_.observe_joint_state();
  }
  void issue_ctc_command(int value) {
    std_msgs::msg::Int32 msg;
    msg.data=value;
    cmd_pub_->publish(msg);
  }
  void do_effects(const Supervisor::Effects& effects) {
    // CTC ESTOP is requested immediately, before slower cancellation paths.
    if(effects.ctc_command>=0) issue_ctc_command(effects.ctc_command);
    if(effects.cancel_active) cancel_jtc(effects.cancel_key);
  }
  void cancel_jtc(const std::string& key) {
    FJTHandle::SharedPtr downstream;
    {
      std::lock_guard<std::mutex> lock(jtc_mutex_);
      if(!operation_||operation_->key!=key) return;
      operation_->cancel_pending=true;
      downstream=operation_->downstream;
    }
    if(downstream) {
      try {jtc_->async_cancel_goal(downstream);}
      catch(const std::exception& e) {RCLCPP_WARN(get_logger(),"JTC cancel: %s",e.what());}
    }
  }
  rclcpp_action::GoalResponse on_goal(const rclcpp_action::GoalUUID& id,
      std::shared_ptr<const Motion::Goal> goal) {
    if(goal->command_type>Motion::Goal::MOVEC||!std::isfinite(goal->velocity_scale)||
       !std::isfinite(goal->acceleration_scale)||goal->velocity_scale<0.01||
       goal->velocity_scale>1.0||goal->acceleration_scale<0.01||
       goal->acceleration_scale>1.0) return rclcpp_action::GoalResponse::REJECT;
    if(goal->context!=Motion::Goal::CONTEXT_NORMAL&&
       goal->context!=Motion::Goal::CONTEXT_SYSID&&
       goal->context!=Motion::Goal::CONTEXT_RECOVERY)
      return rclcpp_action::GoalResponse::REJECT;
    if(goal->context!=Motion::Goal::CONTEXT_NORMAL&&goal->command_type!=Motion::Goal::MOVEJ)
      return rclcpp_action::GoalResponse::REJECT;
    const auto key=hex_uuid(id);
    std::string reason;
    if(!supervisor_.reserve_motion(key,goal->context,goal->session_id,reason)) {
      RCLCPP_WARN(get_logger(),"Goal rejected by Supervisor: %s",reason.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    publish_state();
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse on_cancel(const std::shared_ptr<MotionHandle>& gh) {
    const auto key=hex_uuid(gh->get_goal_id());
    if(!supervisor_.request_cancel(key)) return rclcpp_action::CancelResponse::REJECT;
    cancel_jtc(key);publish_state();
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
      {
        std::lock_guard<std::mutex> lock(joints_mutex_);
        start=actual_;
      }
      if(shutdown_.load()||!supervisor_.permitted(key))
        throw MotionError(Motion::Result::ERR_CANCELLED,"Supervisor withdrew authorization");
      const auto traj=planner_.plan(*gh->get_goal(),start,[this,key]{
        return shutdown_.load()||!supervisor_.permitted(key);
      });
      if(!supervisor_.permitted(key))
        throw MotionError(Motion::Result::ERR_CANCELLED,"Goal cancelled during planning");
      const auto feedback=std::make_shared<Motion::Feedback>();
      feedback->current_state=Motion::Feedback::STATE_WAITING_JTC;
      feedback->percent_complete=0;
      gh->publish_feedback(feedback);
      dispatch_jtc(key,traj,gh);
    } catch(const MotionError& error) {
      finish_upper(key,gh,error.code,error.what(),0);
    } catch(const std::exception& error) {
      finish_upper(key,gh,Motion::Result::ERR_INTERNAL,error.what(),0);
    }
  }
  void dispatch_jtc(const std::string& key,
      const trajectory_msgs::msg::JointTrajectory& trajectory,
      std::shared_ptr<MotionHandle> upper) {
    if(!supervisor_.permitted(key)) {
      finish_upper(key,upper,Motion::Result::ERR_CANCELLED,"Authorization revoked",0);return;
    }
    if(!jtc_->wait_for_action_server(2s)) {
      finish_upper(key,upper,Motion::Result::ERR_EXECUTION_ABORTED,"JTC unavailable",0);return;
    }
    if(!supervisor_.change_motion_phase(key,S::MOTION_WAITING_JTC)) {
      finish_upper(key,upper,Motion::Result::ERR_CANCELLED,"Motion canceled before JTC dispatch",0);return;
    }
    const double duration=sec(trajectory.points.back().time_from_start);
    {
      std::lock_guard<std::mutex> lock(jtc_mutex_);
      operation_=std::make_unique<JTCOperation>();
      operation_->key=key;operation_->upper=upper;
      operation_->duration=duration;
      operation_->dispatch_time=Clock::now();
      operation_->deadline=Clock::now()+std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(duration+12.0));
    }
    // Cancellation may have arrived after the initial authorization check,
    // while the downstream goal slot was being prepared.
    if(!supervisor_.permitted(key)) {
      finish_jtc(key,Motion::Result::ERR_CANCELLED,"Authorization revoked before JTC send");
      return;
    }
    FJT::Goal g;g.trajectory=trajectory;
    rclcpp_action::Client<FJT>::SendGoalOptions options;
    options.goal_response_callback=[this,key](const FJTHandle::SharedPtr& downstream) {
      if(!downstream) {
        finish_jtc(key,Motion::Result::ERR_EXECUTION_ABORTED,"JTC rejected goal");return;
      }
      bool should_cancel=false;
      {
        std::lock_guard<std::mutex> lock(jtc_mutex_);
        if(!operation_||operation_->key!=key) should_cancel=true;
        else {
          operation_->downstream=downstream;
          should_cancel=operation_->cancel_pending;
        }
      }
      if(should_cancel||!supervisor_.permitted(key)) {
        try {jtc_->async_cancel_goal(downstream);}
        catch(const std::exception& e) {RCLCPP_WARN(get_logger(),"JTC cancellation: %s",e.what());}
      }
    };
    options.feedback_callback=[this,key](FJTHandle::SharedPtr,
        const std::shared_ptr<const FJT::Feedback> feedback) {
      std::shared_ptr<MotionHandle> upper_goal;
      double duration=0;
      {
        std::lock_guard<std::mutex> lock(jtc_mutex_);
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
      if(wrapped.code==rclcpp_action::ResultCode::SUCCEEDED&&wrapped.result&&
         wrapped.result->error_code==FJT::Result::SUCCESSFUL)
        finish_jtc(key,Motion::Result::SUCCESS,"JTC trajectory completed");
      else if(wrapped.code==rclcpp_action::ResultCode::CANCELED)
        finish_jtc(key,Motion::Result::ERR_CANCELLED,"JTC trajectory canceled");
      else finish_jtc(key,Motion::Result::ERR_EXECUTION_ABORTED,"JTC execution failed");
    };
    try {jtc_->async_send_goal(g,options);}
    catch(const std::exception& e) {
      finish_jtc(key,Motion::Result::ERR_EXECUTION_ABORTED,e.what());
    }
    publish_state();
  }
  void finish_upper(const std::string& key,const std::shared_ptr<MotionHandle>& gh,
      int32_t code,const std::string& message,double duration) {
    const auto effects=supervisor_.finish_motion(key,code==Motion::Result::SUCCESS,code,message);
    auto result=std::make_shared<Motion::Result>();
    result->error_code=effects.terminal_success?Motion::Result::SUCCESS:
      (code==Motion::Result::SUCCESS?Motion::Result::ERR_EXECUTION_ABORTED:code);
    result->error_string=message;
    result->execution_time=duration;
    do_effects(effects);
    if(gh&&gh->is_active()) {
      if(effects.terminal_success) gh->succeed(result);
      else if(gh->is_canceling()) gh->canceled(result);
      else gh->abort(result);
    }
    publish_state();
  }
  void finish_jtc(const std::string& key,int32_t code,const std::string& message) {
    std::shared_ptr<MotionHandle> upper;
    double elapsed=0;
    {
      std::lock_guard<std::mutex> lock(jtc_mutex_);
      if(!operation_||operation_->key!=key) return; // stale callback / timed out
      upper=operation_->upper;
      elapsed=std::chrono::duration<double>(Clock::now()-operation_->dispatch_time).count();
      operation_.reset();
    }
    if(upper) finish_upper(key,upper,code,message,elapsed);
    else {
      const auto effects=supervisor_.finish_motion(key,code==Motion::Result::SUCCESS,
          code,message);
      do_effects(effects);publish_state();
    }
  }
  void on_sysid_trajectory(const lite6_interfaces::srv::SubmitSysIdTrajectory::Request& req,
      lite6_interfaces::srv::SubmitSysIdTrajectory::Response& resp) {
    // Validate early before occupying the single motion slot.
    try {
      planner_.verify_trajectory(req.trajectory,true,[this]{return shutdown_.load();});
    } catch(const std::exception& e) {
      resp.accepted=false;resp.message=std::string("SysID trajectory rejected: ")+e.what();
      resp.revision=supervisor_.snapshot().revision;return;
    }
    // The Fourier reference must start at the current physical pose; no jump.
    {
      std::lock_guard<std::mutex> lock(joints_mutex_);
      for(int i=0;i<6;++i) {
        if(std::abs(req.trajectory.points.front().positions[i]-actual_.q[i])>0.05||
           std::abs(actual_.dq[i])>0.02) {
          resp.accepted=false;
          resp.message="SysID reference start differs from stationary physical state";
          resp.revision=supervisor_.snapshot().revision;
          return;
        }
      }
    }
    std::string key,why;
    if(!supervisor_.reserve_excitation(req.session_id,key,why)) {
      resp.accepted=false;resp.message=why;
      resp.revision=supervisor_.snapshot().revision;return;
    }
    resp.accepted=true;
    resp.revision=supervisor_.snapshot().revision;
    resp.message="Trajectory authorized and submitted to JTC; monitor supervisor state for completion";
    dispatch_jtc(key,req.trajectory,nullptr);
  }
  void on_heartbeat() {
    do_effects(supervisor_.watchdog());
    std::string timed_out;
    {
      std::lock_guard<std::mutex> lock(jtc_mutex_);
      if(operation_&&Clock::now()>operation_->deadline) timed_out=operation_->key;
    }
    if(!timed_out.empty()) {
      cancel_jtc(timed_out);
      finish_jtc(timed_out,Motion::Result::ERR_TIMEOUT,"JTC goal/result timeout");
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
