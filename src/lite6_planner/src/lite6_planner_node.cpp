#include "lite6_kinematics/kinematics_engine.hpp"
#include "lite6_planner/motion_planner.hpp"
#include "lite6_planner/supervisor.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <lite6_interfaces/action/industrial_motion.hpp>
#include <lite6_interfaces/msg/controller_status.hpp>
#include <lite6_interfaces/msg/supervisor_state.hpp>
#include <lite6_interfaces/msg/motion_rejection.hpp>
#include <lite6_interfaces/srv/controller_command.hpp>
#include <lite6_interfaces/srv/supervisor_command.hpp>
#include <lite6_interfaces/compliance_validation.hpp>
#include <lite6_interfaces/motion_contract.hpp>
#include <lite6_interfaces/shm_registers.hpp>
#include <lite6_interfaces/srv/submit_sys_id_trajectory.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <pthread.h>
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
    const auto kin_pkg=ament_index_cpp::get_package_share_directory("lite6_kinematics");
    if(!kinematics_.load_ik_config(kin_pkg+"/config/ik_config.yaml")||
       !kinematics_.init_robot_model(desc+"/urdf/lite6_arm.urdf",kin_pkg+"/config/lite6.srdf")||
       !kinematics_.load_limits(kin_pkg+"/config/kinematic_limits.yaml"))
      throw std::runtime_error("Robot planning model / YAML initialization failed");
    io_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    validation_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cmd_client_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // Joint freshness must never stall behind status/heartbeat callbacks.
    joint_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    const auto state_qos=rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    state_pub_=create_publisher<S>("/supervisor/state",state_qos);
    rejection_pub_=create_publisher<lite6_interfaces::msg::MotionRejection>(
      "/supervisor/motion_rejections",rclcpp::QoS(32).reliable());
    ctc_cmd_client_=create_client<lite6_interfaces::srv::ControllerCommand>(
      "/ctc_controller/command",rclcpp::ServicesQoS(),cmd_client_group_);
    rclcpp::SubscriptionOptions sensor_options;
    sensor_options.callback_group=io_group_;
    controller_sub_=create_subscription<lite6_interfaces::msg::ControllerStatus>(
      "/ctc_controller/status",rclcpp::QoS(10),
      [this](const lite6_interfaces::msg::ControllerStatus::SharedPtr msg) {
        const auto stamp=rclcpp::Time(msg->stamp).nanoseconds();
        if(msg->controller_epoch!=0||msg->status_seq!=0) {
          if(msg->controller_epoch<last_controller_epoch_) return;
          if(msg->controller_epoch==last_controller_epoch_&&
             msg->status_seq<=last_controller_status_seq_) return;
          last_controller_epoch_=msg->controller_epoch;
          last_controller_status_seq_=msg->status_seq;
          last_controller_stamp_=stamp;
        } else {
          if(stamp<=last_controller_stamp_) return;
          last_controller_stamp_=stamp;
        }
        {
          std::lock_guard<std::mutex> transition(transition_mutex_);
          do_effects(supervisor_.observe_controller(*msg));
        }
        {
          std::lock_guard<std::mutex> lock(ctc_mutex_);
          if(msg->trajectory_start.sec>0) {
            last_traj_start_id_=msg->trajectory_id;
            last_traj_start_time_=msg->trajectory_start;
          }
          if(operation_&&operation_->downstream&&
              operation_->downstream->get_goal_id()==msg->trajectory_id)
            supervisor_.observe_excitation_start(operation_->key,msg->trajectory_start);
        }
        publish_state();
      },sensor_options);
    const auto shm_status=state_shm_.open_status(false);
    if(shm_status==lite6_interfaces::shm::ShmOpenStatus::INCOMPATIBLE||
       shm_status==lite6_interfaces::shm::ShmOpenStatus::OS_ERROR) {
      throw std::runtime_error(
        std::string("Failed to open read-only state shared memory: ")+
        lite6_interfaces::shm::shm_open_status_name(shm_status));
    }
    if(shm_status==lite6_interfaces::shm::ShmOpenStatus::NOT_FOUND) {
      RCLCPP_INFO(get_logger(),
        "State shared memory not created yet; waiting lazily for hardware interface");
    }
    joint_poll_timer_=create_wall_timer(10ms,[this]{poll_joint_state_shm();},joint_group_);
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
        using SC=lite6_interfaces::srv::SupervisorCommand;
        const bool compliance=(req->command>=8&&req->command<=10)||req->command==12;
        if(compliance&&!lite6_interfaces::valid_compliance_settings(req->compliance)) {
          const auto snap=supervisor_.snapshot();
          resp->accepted=false;resp->planner_epoch=snap.planner_epoch;
          resp->session_id=snap.session_id;
          resp->revision=snap.revision;resp->mode=snap.mode;
          resp->message="Invalid compliance gains / sensitivity";return;
        }
        if(req->command==SC::Request::CMD_SET_PAYLOAD&&
           !lite6_interfaces::valid_payload_params(req->payload_mass,req->payload_com_tcp)) {
          const auto snap=supervisor_.snapshot();
          resp->accepted=false;resp->planner_epoch=snap.planner_epoch;
          resp->session_id=snap.session_id;
          resp->revision=snap.revision;resp->mode=snap.mode;
          resp->message="Invalid payload parameters";return;
        }
        Supervisor::Reply reply;
        PendingCtcCommand pending;
        bool sent=true;
        {
          std::lock_guard<std::mutex> transition(transition_mutex_);
          reply=supervisor_.command(*req);
          sent=do_effects(reply.effects,&pending);
        }
        resp->accepted=reply.accepted;resp->planner_epoch=reply.planner_epoch;
        resp->session_id=reply.session_id;
        resp->revision=reply.revision;resp->mode=reply.mode;resp->message=reply.message;
        // Wait for the CTC response outside transition_mutex_ so status/heartbeat
        // callbacks keep refreshing controller and joint freshness meanwhile.
        const bool cmd_ok=sent&&await_ctc_command(pending);
        if(!cmd_ok&&req->command!=SC::Request::CMD_ESTOP&&reply.accepted) {
          const auto snap=supervisor_.snapshot();
          resp->accepted=false;resp->planner_epoch=snap.planner_epoch;
          resp->revision=snap.revision;resp->mode=snap.mode;
          resp->message=snap.detail;
        } else if(cmd_ok&&reply.accepted&&pending.request) {
          // The CTC acknowledgement may have advanced the state (e.g. payload result).
          const auto snap=supervisor_.snapshot();
          resp->planner_epoch=snap.planner_epoch;
          resp->revision=snap.revision;resp->mode=snap.mode;resp->message=snap.detail;
        }
        publish_state();
      });
    sysid_service_=create_service<lite6_interfaces::srv::SubmitSysIdTrajectory>(
      "/supervisor/submit_sysid_trajectory",
      [this](const std::shared_ptr<lite6_interfaces::srv::SubmitSysIdTrajectory::Request> req,
             std::shared_ptr<lite6_interfaces::srv::SubmitSysIdTrajectory::Response> resp){
        on_sysid_trajectory(*req,*resp);
      },rmw_qos_profile_services_default,validation_group_);
    planner_thread_=std::thread([this]{planner_worker_loop();});
    heartbeat_=create_wall_timer(100ms,[this]{on_heartbeat();},io_group_);
    RCLCPP_INFO(get_logger(),"Supervisor-owned Robot planner created (awaiting CTC status)");
  }
  ~IndustrialPlannerNode() override {
    shutdown_.store(true);
    {
      std::lock_guard<std::mutex> lock(planner_queue_mutex_);
      planner_exit_=true;
    }
    planner_cv_.notify_all();
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
  lite6_kinematics::KinematicsEngine kinematics_;
  MotionPlanner planner_;
  Supervisor supervisor_;
  mutable std::mutex joints_mutex_;
  std::mutex ctc_mutex_;
  // Serialize a state transition with its mode-command publication. Otherwise
  // a pending NORMAL command could be published after a concurrent ESTOP.
  std::mutex transition_mutex_;
  int64_t last_controller_stamp_=0;
  uint64_t last_controller_epoch_=0;
  uint64_t last_controller_status_seq_=0;
  std::array<uint8_t,16> last_traj_start_id_{};
  builtin_interfaces::msg::Time last_traj_start_time_{};
  uint64_t last_joint_seq_=0;
  int64_t last_joint_steady_ns_=0;
  MotionPlanner::InitialState actual_{};
  int64_t actual_steady_ns_=0;
  std::unique_ptr<CTCOperation> operation_;
  std::mutex planner_queue_mutex_;
  std::condition_variable planner_cv_;
  std::deque<std::shared_ptr<MotionHandle>> planner_queue_;
  bool planner_exit_=false;
  std::thread planner_thread_;
  rclcpp::CallbackGroup::SharedPtr io_group_;
  rclcpp::CallbackGroup::SharedPtr validation_group_;
  rclcpp::CallbackGroup::SharedPtr cmd_client_group_;
  rclcpp::CallbackGroup::SharedPtr joint_group_;
  std::atomic<bool> shutdown_{false};
  lite6_interfaces::shm::ShmHandle<lite6_interfaces::shm::StateShmSegment> state_shm_;
  rclcpp::TimerBase::SharedPtr joint_poll_timer_;
  rclcpp::Publisher<S>::SharedPtr state_pub_;
  rclcpp::Publisher<lite6_interfaces::msg::MotionRejection>::SharedPtr rejection_pub_;
  rclcpp::Client<lite6_interfaces::srv::ControllerCommand>::SharedPtr ctc_cmd_client_;
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
    if(!lite6_interfaces::shm::is_stamp_fresh(
          pod.steady_stamp_ns,now_ns,lite6_interfaces::contract::kJointStateStaleTimeoutNs)) {
      return false;
    }
    max_speed=0.0;
    for(int i=0;i<6;++i) {
      out.q[i]=pod.position[i];out.dq[i]=pod.velocity[i];
      if(!std::isfinite(out.q[i])||!std::isfinite(out.dq[i])) return false;
      max_speed=std::max(max_speed,std::abs(out.dq[i]));
    }
    out_steady_ns=pod.steady_stamp_ns;
    return true;
  }
  bool read_fresh_cached_joints(MotionPlanner::InitialState& out) const {
    std::lock_guard<std::mutex> lock(joints_mutex_);
    const int64_t now_ns=lite6_interfaces::shm::steady_now_ns();
    if(!lite6_interfaces::shm::is_stamp_fresh(
          actual_steady_ns_,now_ns,lite6_interfaces::contract::kJointStateStaleTimeoutNs)) {
      return false;
    }
    out=actual_;
    return true;
  }
  void poll_joint_state_shm() {
    if(!state_shm_.valid()) {
      const auto status=state_shm_.open_status(false);
      if(status!=lite6_interfaces::shm::ShmOpenStatus::OK) {
        if(status==lite6_interfaces::shm::ShmOpenStatus::INCOMPATIBLE||
           status==lite6_interfaces::shm::ShmOpenStatus::OS_ERROR) {
          RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,
            "State shared memory open failed (%s); refusing to attach",
            lite6_interfaces::shm::shm_open_status_name(status));
        }
        return;
      }
    }
    MotionPlanner::InitialState candidate;
    double max_speed=0.0;
    uint64_t seq=0;
    int64_t steady_ns=0;
    if(!read_shm_joints(candidate,max_speed,seq,steady_ns)) return;
    if(seq==last_joint_seq_||steady_ns<=last_joint_steady_ns_) return;
    {
      std::lock_guard<std::mutex> lock(joints_mutex_);
      actual_=candidate;
      actual_steady_ns_=steady_ns;
    }
    last_joint_seq_=seq;
    last_joint_steady_ns_=steady_ns;
    supervisor_.observe_joint_state(max_speed,steady_ns);
  }
  using CtcCommand=lite6_interfaces::srv::ControllerCommand;
  struct PendingCtcCommand {
    uint8_t cmd=CtcCommand::Request::CMD_NONE;
    std::optional<rclcpp::Client<CtcCommand>::FutureAndRequestId> request;
  };
  // Sends a CTC command without waiting for its response (called under
  // transition_mutex_, which only orders command emission). With `pending`, the
  // caller awaits the response after releasing the mutex; otherwise the response is
  // acknowledged from cmd_client_group_.
  bool issue_ctc_command(const Supervisor::Effects& effects,PendingCtcCommand* pending) {
    if(!ctc_cmd_client_->service_is_ready()&&!ctc_cmd_client_->wait_for_service(20ms)) {
      // Never sent: definitively not applied.
      supervisor_.on_controller_command_ack(
        effects.ctc_cmd,false,CtcCommand::Response::ACK_RT_TIMEOUT,0,
        "CTC command service unavailable");
      return false;
    }
    auto req=std::make_shared<CtcCommand::Request>();
    req->command=effects.ctc_cmd;
    req->expected_epoch=effects.expected_controller_epoch;
    if(effects.has_compliance) req->compliance=effects.compliance;
    if(effects.has_payload) {
      req->payload_mass=effects.payload_mass;
      req->payload_com_tcp=effects.payload_com_tcp;
    }
    req->reason=effects.cmd_reason;
    if(pending) {
      pending->cmd=effects.ctc_cmd;
      pending->request.emplace(ctc_cmd_client_->async_send_request(req));
      return true;
    }
    const uint8_t cmd=effects.ctc_cmd;
    ctc_cmd_client_->async_send_request(req,
      [this,cmd](rclcpp::Client<CtcCommand>::SharedFuture future) {
        const auto resp=future.get();
        const bool unknown=resp->ack_result==CtcCommand::Response::ACK_OUTCOME_UNKNOWN;
        supervisor_.on_controller_command_ack(
          cmd,resp->accepted,resp->ack_result,resp->current_mode,resp->message,
          unknown,resp->cmd_seq);
        publish_state();
      });
    return true;
  }
  // Must be called without holding transition_mutex_.
  bool await_ctc_command(PendingCtcCommand& pending) {
    if(!pending.request) return true;
    if(pending.request->future.wait_for(250ms)!=std::future_status::ready) {
      ctc_cmd_client_->remove_pending_request(*pending.request);
      // No response: the CTC may have applied it, so mark outcome unknown and reconcile.
      supervisor_.on_controller_command_ack(
        pending.cmd,false,CtcCommand::Response::ACK_OUTCOME_UNKNOWN,0,
        "Timed out waiting for /ctc_controller/command response",true,0);
      return false;
    }
    const auto resp=pending.request->future.get();
    const bool unknown=resp->ack_result==CtcCommand::Response::ACK_OUTCOME_UNKNOWN;
    supervisor_.on_controller_command_ack(
      pending.cmd,resp->accepted,resp->ack_result,resp->current_mode,resp->message,
      unknown,resp->cmd_seq);
    return resp->accepted;
  }
  bool do_effects(const Supervisor::Effects& effects,PendingCtcCommand* pending=nullptr) {
    // CTC ESTOP is requested immediately, before slower cancellation paths.
    bool cmd_ok=true;
    if(effects.ctc_cmd!=CtcCommand::Request::CMD_NONE) {
      cmd_ok=issue_ctc_command(effects,pending);
    }
    if(effects.cancel_active) cancel_ctc(effects.cancel_key);
    return cmd_ok;
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
       goal->context!=Motion::Goal::CONTEXT_SYSID)
      return reject(Motion::Result::ERR_INVALID_INPUT,"Unsupported motion context="+std::to_string(goal->context));
    if(goal->context!=Motion::Goal::CONTEXT_NORMAL&&goal->command_type!=Motion::Goal::MOVEJ)
      return reject(Motion::Result::ERR_SUPERVISOR_DENIED,"SYSID context only permits MoveJ reference alignment");
    std::string reason;
    {
      std::lock_guard<std::mutex> transition(transition_mutex_);
      if(!supervisor_.reserve_motion(key,goal->context,goal->session_id,reason)) {
        return reject(Motion::Result::ERR_SUPERVISOR_DENIED,reason);
      }
    }
    publish_state();
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse on_cancel(const std::shared_ptr<MotionHandle>& gh) {
    const auto key=hex_uuid(gh->get_goal_id());
    {
      std::lock_guard<std::mutex> transition(transition_mutex_);
      if(!supervisor_.request_cancel(key)) return rclcpp_action::CancelResponse::REJECT;
    }
    if(gh&&gh->is_active()) {
      auto fb=std::make_shared<Motion::Feedback>();
      fb->current_state=Motion::Feedback::STATE_CANCELING;
      fb->percent_complete=0.0;
      gh->publish_feedback(fb);
    }
    cancel_ctc(key);publish_state();
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void on_accepted(const std::shared_ptr<MotionHandle>& gh) {
    {
      std::lock_guard<std::mutex> lock(planner_queue_mutex_);
      planner_queue_.push_back(gh);
    }
    planner_cv_.notify_one();
  }
  void planner_worker_loop() {
    pthread_setname_np(pthread_self(), "lite6_planner");
    while(true) {
      std::shared_ptr<MotionHandle> gh;
      {
        std::unique_lock<std::mutex> lock(planner_queue_mutex_);
        planner_cv_.wait(lock,[this]{
          return planner_exit_||shutdown_.load()||!planner_queue_.empty();
        });
        if((planner_exit_||shutdown_.load())&&planner_queue_.empty()) return;
        if(planner_queue_.empty()) continue;
        gh=std::move(planner_queue_.front());
        planner_queue_.pop_front();
      }
      if(!shutdown_.load()) {
        compute_and_dispatch(gh);
      } else if(gh) {
        finish_upper(hex_uuid(gh->get_goal_id()),gh,
          Motion::Result::ERR_CANCELLED,"Planner shutting down",0);
      }
    }
  }
  void compute_and_dispatch(const std::shared_ptr<MotionHandle>& gh) {
    const auto key=hex_uuid(gh->get_goal_id());
    try {
      if(gh&&gh->is_active()) {
        auto plan_fb=std::make_shared<Motion::Feedback>();
        plan_fb->current_state=Motion::Feedback::STATE_PLANNING;
        plan_fb->percent_complete=0.0;
        gh->publish_feedback(plan_fb);
      }
      MotionPlanner::InitialState start;
      double dummy_speed=0.0; uint64_t dummy_seq=0; int64_t dummy_ns=0;
      if(!read_shm_joints(start,dummy_speed,dummy_seq,dummy_ns)&&
         !read_fresh_cached_joints(start)) {
        throw MotionError(Motion::Result::ERR_NOT_READY,
          "Fresh joint state feedback unavailable for motion planning");
      }
      if(shutdown_.load()||!supervisor_.permitted(key))
        throw MotionError(Motion::Result::ERR_CANCELLED,"Supervisor withdrew authorization");
      const trajectory_msgs::msg::JointTrajectory traj=planner_.plan(*gh->get_goal(),start,[this,key]{
        return shutdown_.load()||!supervisor_.permitted(key);
      });
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
    bool ctc_ready=false;
    for(int attempt=0;attempt<20;++attempt) {
      if(shutdown_.load()||!supervisor_.permitted(key)) {
        finish_upper(key,upper,Motion::Result::ERR_CANCELLED,
            "Goal canceled while waiting for CTC server",0);
        return;
      }
      if(ctc_->wait_for_action_server(100ms)) {ctc_ready=true;break;}
    }
    if(!ctc_ready) {
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
      bool cached_start=false;
      builtin_interfaces::msg::Time cached_start_time{};
      {
        std::lock_guard<std::mutex> lock(ctc_mutex_);
        if(!operation_||operation_->key!=key) should_cancel=true;
        else {
          operation_->downstream=downstream;
          should_cancel=operation_->cancel_pending;
          if(last_traj_start_time_.sec>0&&downstream->get_goal_id()==last_traj_start_id_) {
            cached_start=true;
            cached_start_time=last_traj_start_time_;
          }
        }
      }
      if(cached_start) {
        supervisor_.observe_excitation_start(key,cached_start_time);
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
      if(!authorized()) throw std::runtime_error("Friction identification session is no longer ready");
      planner_.verify_trajectory(req.trajectory,true,[&authorized]{return !authorized();});
      for(size_t tick=0;tick<req.trajectory.points.size();++tick) {
        const auto& t=req.trajectory.points[tick].time_from_start;
        if(static_cast<int64_t>(t.sec)*1000000000LL+t.nanosec!=
            static_cast<int64_t>(tick)*1000000LL)
          throw std::runtime_error("Friction identification references must use an exact 1 ms grid starting at zero");
      }
    } catch(const std::exception& e) {
      supervisor_.end_excitation_validation(req.session_id);
      resp.accepted=false;resp.message=std::string("Friction identification trajectory rejected: ")+e.what();
      resp.revision=supervisor_.snapshot().revision;return;
    }
    // The Fourier reference must start at the current physical pose; no jump.
    {
      MotionPlanner::InitialState current;
      double dummy_speed=0.0; uint64_t dummy_seq=0; int64_t dummy_ns=0;
      if(!read_shm_joints(current,dummy_speed,dummy_seq,dummy_ns)&&
         !read_fresh_cached_joints(current)) {
        resp.accepted=false;
        resp.message="Fresh joint state feedback unavailable for friction identification validation";
        supervisor_.end_excitation_validation(req.session_id);
        resp.revision=supervisor_.snapshot().revision;
        return;
      }
      for(int i=0;i<6;++i) {
        if(std::abs(req.trajectory.points.front().positions[i]-current.q[i])>0.05||
           std::abs(current.dq[i])>0.02) {
          resp.accepted=false;
          resp.message="Friction identification reference start differs from stationary physical state";
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
    // Drop bookkeeping for callback-style CTC requests that never got a response.
    ctc_cmd_client_->prune_requests_older_than(
      std::chrono::system_clock::now()-std::chrono::seconds(5));
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
