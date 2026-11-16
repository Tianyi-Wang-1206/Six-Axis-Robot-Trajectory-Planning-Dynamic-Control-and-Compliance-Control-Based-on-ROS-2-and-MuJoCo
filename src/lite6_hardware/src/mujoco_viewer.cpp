// A deliberately small native MuJoCo viewer. GLFW, ROS callbacks and all display
// mjData live on the main thread; the hardware process owns the physics state.
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "lite6_interfaces/shm_registers.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
using Clock = std::chrono::steady_clock;
int64_t now_ns()
{
  return lite6_interfaces::shm::steady_now_ns();
}

class Viewer
{
public:
  explicit Viewer(const rclcpp::Node::SharedPtr & node) : node_(node)
  {
    fps_limit_ = node_->declare_parameter("fps", 60);
    if (fps_limit_ < 15 || fps_limit_ > 240) {
      throw std::runtime_error("fps must be in [15, 240]");
    }
    show_shadow_ = node_->declare_parameter("shadow", false);
    // Cast shadows amplify overlapping STL surfaces in close-up views.
    const bool shadows = node_->declare_parameter("shadows", false);
    const auto path = ament_index_cpp::get_package_share_directory("lite6_description") +
      "/mujoco/scene.xml";
    char error[1024]{};
    model_.reset(mj_loadXML(path.c_str(), nullptr, error, sizeof(error)));
    if (!model_) {throw std::runtime_error(error);}
    data_.reset(mj_makeData(model_.get()));
    shadow_.reset(mj_makeData(model_.get()));
    if (!data_ || !shadow_) {throw std::runtime_error("Cannot allocate display state");}
    for (size_t i = 0; i < 6; ++i) {
      const int joint = mj_name2id(model_.get(), mjOBJ_JOINT,
        ("joint" + std::to_string(i + 1)).c_str());
      if (joint < 0) {throw std::runtime_error("Missing display joint");}
      qpos_[i] = model_->jnt_qposadr[joint];
      qvel_[i] = model_->jnt_dofadr[joint];
    }
    for (size_t i = 0; i < 2; ++i) {
      const int id = mj_name2id(model_.get(), mjOBJ_JOINT,
        i == 0 ? "gripper_left_finger" : "gripper_right_finger");
      if (id < 0) {throw std::runtime_error("Missing gripper display joint");}
      gripper_qpos_[i] = model_->jnt_qposadr[id];
      gripper_qvel_[i] = model_->jnt_dofadr[id];
      data_->qpos[gripper_qpos_[i]] = shadow_->qpos[gripper_qpos_[i]] = i == 0 ? -0.0081 : 0.0081;
    }
    const std::array<const char *, 2> object_joints{
      "grasp_cylinder_free", "grasp_box_free"};
    for (size_t i = 0; i < object_joints.size(); ++i) {
      const int joint = mj_name2id(model_.get(), mjOBJ_JOINT, object_joints[i]);
      if (joint < 0 || model_->jnt_type[joint] != mjJNT_FREE) {
        throw std::runtime_error("Missing scene object free joint");
      }
      object_qpos_[i] = model_->jnt_qposadr[joint];
    }
    mj_forward(model_.get(), data_.get());
    mj_forward(model_.get(), shadow_.get());
    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&options_);
    mjv_defaultPerturb(&perturb_);
    mjv_defaultScene(&scene_);
    mjr_defaultContext(&context_);
    options_.geomgroup[4] = 1;  // Movable scene objects.
    options_.geomgroup[3] = 0;  // Collision meshes must not cover the visual meshes.
    options_.flags[mjVIS_CONVEXHULL] = 0;  // Render the full visual meshes.
    options_.sitegroup[4] = 0;
    reset_camera();

    if (!state_shm_.open(false)) {
      throw std::runtime_error("Cannot open read-only state shared memory");
    }
    if (!perturb_shm_.open(true)) {
      throw std::runtime_error("Cannot open perturbation shared memory");
    }

    glfwWindowHint(GLFW_SAMPLES, 8);
    glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
    glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE);
    int width = 1400, height = 1400;
    float content_xscale = 1.0f, content_yscale = 1.0f;
    if (auto * monitor = glfwGetPrimaryMonitor()) {
      glfwGetMonitorContentScale(monitor, &content_xscale, &content_yscale);
      // On unscaled desktops (1.0x), clamp to the work area; on HiDPI X11
      // fractional scaling, GLFW reports CRTC work area smaller than the X11
      // virtual screen while GLFW_SCALE_TO_MONITOR multiplies width/height.
      if (content_xscale <= 1.05f && content_yscale <= 1.05f) {
        int work_x, work_y, work_width, work_height;
        glfwGetMonitorWorkarea(monitor, &work_x, &work_y, &work_width, &work_height);
        if (work_width > 80 && work_height > 100) {
          width = std::min(width, work_width - 80);
          height = std::min(height, work_height - 100);
        }
      }
    }
    window_.reset(glfwCreateWindow(width, height, "Robot | MuJoCo Studio", nullptr, nullptr));
    if (!window_) {throw std::runtime_error("Cannot create an OpenGL window (check DISPLAY)");}
    glfwMakeContextCurrent(window_.get());
    glfwSwapInterval(1);
    mjv_makeScene(model_.get(), &scene_, 512);
    const int font_scale = (content_xscale >= 1.75f || content_yscale >= 1.75f) ?
      mjFONTSCALE_200 : mjFONTSCALE_150;
    mjr_makeContext(model_.get(), &context_, font_scale);
    scene_.flags[mjRND_REFLECTION] = 0;
    scene_.flags[mjRND_SHADOW] = shadows;
    glfwSetWindowUserPointer(window_.get(), this);
    glfwSetMouseButtonCallback(window_.get(), [](GLFWwindow * w, int b, int a, int mods) {
      self(w).button(b, a, mods);
    });
    glfwSetCursorPosCallback(window_.get(), [](GLFWwindow * w, double x, double y) {
      self(w).motion(x, y);
    });
    glfwSetScrollCallback(window_.get(), [](GLFWwindow * w, double, double y) {
      auto & v = self(w);
      mjv_moveCamera(v.model_.get(), mjMOUSE_ZOOM, 0, -0.05 * y, &v.scene_, &v.camera_);
    });
    glfwSetWindowFocusCallback(window_.get(), [](GLFWwindow * w, int focused) {
      if (!focused) {self(w).release();}
    });
    glfwSetKeyCallback(window_.get(), [](GLFWwindow * w, int key, int, int action, int) {
      auto & v = self(w);
      if ((key == GLFW_KEY_LEFT_CONTROL || key == GLFW_KEY_RIGHT_CONTROL) &&
        action == GLFW_RELEASE) {v.release();}
      if (action != GLFW_PRESS) {return;}
      if (key == GLFW_KEY_ESCAPE) {v.release(); glfwSetWindowShouldClose(w, 1);}
      if (key == GLFW_KEY_HOME) {v.reset_camera();}
      if (key == GLFW_KEY_H) {v.help_ = !v.help_;}
      if (key == GLFW_KEY_S) {v.show_shadow_ = !v.show_shadow_;}
    });
  }

  ~Viewer()
  {
    // All GL resources are freed while the owning context is still current.
    if (window_) {glfwMakeContextCurrent(window_.get());}
    mjr_freeContext(&context_);
    mjv_freeScene(&scene_);
  }

  void run()
  {
    const auto period = std::chrono::nanoseconds(1000000000 / fps_limit_);
    auto deadline = Clock::now();
    auto stats_start = deadline;
    int frames = 0;
    double fps = 0;
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_);
    RCLCPP_INFO(node_->get_logger(), "Native MuJoCo viewer: %d FPS target; H for controls", fps_limit_);
    while (rclcpp::ok() && !glfwWindowShouldClose(window_.get())) {
      executor.spin_some(std::chrono::milliseconds(0));
      poll_shm_state();
      if (dirty_) {mj_forward(model_.get(), data_.get()); dirty_ = false;}
      glfwPollEvents();
      const bool fresh = Clock::now() - last_state_ < std::chrono::milliseconds(250);
      int width, height;
      glfwGetFramebufferSize(window_.get(), &width, &height);
      if (!fresh || !active_ || width <= 0 || height <= 0 ||
        glfwGetWindowAttrib(window_.get(), GLFW_ICONIFIED)) {release();}
      if (width > 0 && height > 0 && !glfwGetWindowAttrib(window_.get(), GLFW_ICONIFIED)) {
        send_target();  // Heartbeat while dragging, including when the mouse is still.
        mjv_updateScene(model_.get(), data_.get(), &options_, &perturb_,
          &camera_, mjCAT_ALL, &scene_);
        if (show_shadow_ && Clock::now() - last_shadow_ < std::chrono::milliseconds(500)) {
          if (shadow_dirty_) {mj_forward(model_.get(), shadow_.get()); shadow_dirty_ = false;}
          const int first = scene_.ngeom;
          auto shadow_options = options_;
          shadow_options.geomgroup[4] = 0;  // Scene objects have no target/ghost copy.
          mjv_addGeoms(model_.get(), shadow_.get(), &shadow_options, nullptr, mjCAT_DYNAMIC, &scene_);
          for (int i = first; i < scene_.ngeom; ++i) {
            auto & g = scene_.geoms[i];
            g.rgba[0] = 0.12f; g.rgba[1] = 0.8f; g.rgba[2] = 0.95f; g.rgba[3] = 0.22f;
            g.matid = -1;
          }
        }
        const mjrRect viewport{0, 0, width, height};
        mjr_render(viewport, &scene_, &context_);
        char status[160];
        std::snprintf(status, sizeof(status), "%s  |  %.0f FPS  |  %.2f s%s",
          !fresh ? "CONNECTING" : (active_ ? "LIVE" : "PHYSICS STOPPED"),
          fps, data_->time, perturb_.active ? "  |  MOUSE FORCE" : "");
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport,
          "ROBOT / MUJOCO STUDIO", status, &context_);
        if (help_) {
          mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport,
            "Orbit / pan / zoom\nSelect body\nApply force / torque\nView / shadow / help",
            "Left drag / right drag / wheel\nDouble-click robot\nCtrl + right / left drag (Shift: horizontal)\nHome / S / H", &context_);
        }
        glfwSwapBuffers(window_.get());
        ++frames;
      }
      const auto now = Clock::now();
      const double elapsed = std::chrono::duration<double>(now - stats_start).count();
      if (elapsed >= 2.0) {
        fps = frames / elapsed;
        RCLCPP_DEBUG(node_->get_logger(), "Rendered %.1f FPS", fps);
        frames = 0; stats_start = now;
      }
      // Absolute deadlines avoid adding a full sleep after vsync. Skip missed
      // render deadlines instead of competing with physics to catch them up.
      deadline += period;
      if (deadline < now) {deadline = now;}
      std::this_thread::sleep_until(deadline);
    }
    if (rclcpp::ok()) {release();}
  }

private:
  static Viewer & self(GLFWwindow * w)
  {
    return *static_cast<Viewer *>(glfwGetWindowUserPointer(w));
  }
  void reset_camera()
  {
    camera_.type = mjCAMERA_FREE;
    camera_.lookat[0] = 0.15; camera_.lookat[1] = 0; camera_.lookat[2] = 0.08;
    camera_.distance = 2.65;
    camera_.azimuth = 135; camera_.elevation = -22;
  }
  void poll_shm_state()
  {
    if (!state_shm_.valid()) {return;}
    const int64_t steady_now = now_ns();
    const int64_t alive_ns = state_shm_->visual_alive_ns.load(std::memory_order_acquire);

    lite6_interfaces::shm::VisualStatePod vis{};
    uint64_t vis_seq = 0;
    if (state_shm_->visual_state.read(vis, vis_seq) && vis.valid != 0) {
      const int64_t latest_stamp = std::max(vis.published_steady_ns, alive_ns);
      if (latest_stamp > 0 && steady_now - latest_stamp < 250000000LL) {
        bool ok = std::isfinite(vis.simulation_time);
        for (size_t i = 0; i < 6 && ok; ++i) {
          ok = std::isfinite(vis.position[i]) && std::isfinite(vis.velocity[i]);
        }
        for (size_t i = 0; i < 2 && ok; ++i) {
          ok = std::isfinite(vis.gripper_position[i]) && std::isfinite(vis.gripper_velocity[i]);
        }
        for (size_t i = 0; i < object_qpos_.size() && ok; ++i) {
          const double * first = vis.object_qpos + 7 * i;
          ok = std::all_of(first, first + 7, [](double v) {return std::isfinite(v);});
          if (ok) {
            const double norm = std::hypot(std::hypot(first[3], first[4]),
              std::hypot(first[5], first[6]));
            ok = norm >= 1e-6;
          }
        }
        if (ok) {
          if (vis_seq != last_visual_seq_) {
            for (size_t i = 0; i < 6; ++i) {
              data_->qpos[qpos_[i]] = vis.position[i];
              data_->qvel[qvel_[i]] = vis.velocity[i];
            }
            for (size_t i = 0; i < 2; ++i) {
              data_->qpos[gripper_qpos_[i]] = vis.gripper_position[i];
              data_->qvel[gripper_qvel_[i]] = vis.gripper_velocity[i];
              shadow_->qpos[gripper_qpos_[i]] = vis.gripper_position[i];
            }
            for (size_t i = 0; i < object_qpos_.size(); ++i) {
              const double * first = vis.object_qpos + 7 * i;
              std::copy_n(first, 7, data_->qpos + object_qpos_[i]);
              mju_normalize4(data_->qpos + object_qpos_[i] + 3);
            }
            data_->time = vis.simulation_time;
            dirty_ = true;
            shadow_dirty_ = true;
            last_visual_seq_ = vis_seq;
          }
          active_ = (vis.active != 0);
          last_state_ = Clock::now();
        }
      }
    } else if (alive_ns > 0 && steady_now - alive_ns < 250000000LL) {
      active_ = false;
      last_state_ = Clock::now();
    }

    lite6_interfaces::shm::ShadowStatePod shadow{};
    uint64_t shadow_seq = 0;
    if (state_shm_->shadow_state.read(shadow, shadow_seq) && shadow.valid != 0 &&
      shadow.stamp_ns > 0 && steady_now - shadow.stamp_ns < 500000000LL)
    {
      bool ok = true;
      for (size_t i = 0; i < 6; ++i) {
        if (!std::isfinite(shadow.position[i])) {ok = false; break;}
      }
      if (ok) {
        if (shadow_seq != last_shadow_seq_) {
          for (size_t i = 0; i < 6; ++i) {shadow_->qpos[qpos_[i]] = shadow.position[i];}
          shadow_dirty_ = true;
          last_shadow_seq_ = shadow_seq;
        }
        last_shadow_ = Clock::now();
      }
    }
  }
  void send_target()
  {
    if (!perturb_.active || !perturb_shm_.valid()) {return;}
    lite6_interfaces::shm::MousePerturbationPod pod{};
    pod.issued_ns = now_ns();
    pod.body_id = perturb_.select;
    pod.active = perturb_.active;
    std::copy_n(perturb_.refpos, 3, pod.refpos);
    std::copy_n(perturb_.refselpos, 3, pod.refselpos);
    std::copy_n(perturb_.refquat, 4, pod.refquat);
    std::copy_n(perturb_.localpos, 3, pod.localpos);
    pod.localmass = perturb_.localmass;
    perturb_shm_->mouse_perturbation.write(pod);
  }
  void release()
  {
    if (perturb_.active && perturb_shm_.valid()) {
      perturb_shm_->mouse_perturbation.write(lite6_interfaces::shm::MousePerturbationPod{});
    }
    perturb_.active = 0;
  }
  void button(int button, int action, int mods)
  {
    glfwGetCursorPos(window_.get(), &last_x_, &last_y_);
    if (action == GLFW_RELEASE) {release(); return;}
    if (action != GLFW_PRESS) {return;}
    const double now = glfwGetTime();
    if (button == GLFW_MOUSE_BUTTON_LEFT && !(mods & GLFW_MOD_CONTROL)) {
      if (now - last_click_ < 0.3 && std::hypot(last_x_ - click_x_, last_y_ - click_y_) < 5) {
        release();
        int width, height, geom, flex, skin;
        glfwGetWindowSize(window_.get(), &width, &height);
        if (width <= 0 || height <= 0) {return;}
        mjtNum point[3], delta[3];
        const int body = mjv_select(model_.get(), data_.get(), &options_,
          static_cast<double>(width) / height, last_x_ / width, 1 - last_y_ / height,
          &scene_, point, &geom, &flex, &skin);
        perturb_.select = body > 0 && model_->body_dofnum[body] > 0 ? body : 0;
        if (perturb_.select) {
          mju_sub3(delta, point, data_->xpos + 3 * body);
          mju_mulMatTVec(perturb_.localpos, data_->xmat + 9 * body, delta, 3, 3);
        }
      }
      last_click_ = now; click_x_ = last_x_; click_y_ = last_y_;
    }
    if ((mods & GLFW_MOD_CONTROL) && perturb_.select && active_ &&
      Clock::now() - last_state_ < std::chrono::milliseconds(250))
    {
      if (button != GLFW_MOUSE_BUTTON_RIGHT && button != GLFW_MOUSE_BUTTON_LEFT) {return;}
      mjv_initPerturb(model_.get(), data_.get(), &scene_, &perturb_);
      perturb_.active = button == GLFW_MOUSE_BUTTON_RIGHT ? mjPERT_TRANSLATE : mjPERT_ROTATE;
      send_target();
    }
  }
  void motion(double x, double y)
  {
    const double dx = x - last_x_, dy = y - last_y_;
    last_x_ = x; last_y_ = y;
    const bool left = glfwGetMouseButton(window_.get(), GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    const bool right = glfwGetMouseButton(window_.get(), GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    const bool middle = glfwGetMouseButton(window_.get(), GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    if (!left && !right && !middle) {return;}
    const bool shift = glfwGetKey(window_.get(), GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
      glfwGetKey(window_.get(), GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
    int width, height;
    glfwGetWindowSize(window_.get(), &width, &height);
    if (height <= 0) {return;}
    const int action = right ? (shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V) :
      (left ? (shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V) : mjMOUSE_ZOOM);
    if (perturb_.active) {
      mjv_movePerturb(model_.get(), data_.get(), action, dx / height, dy / height,
        &scene_, &perturb_);
    } else {
      mjv_moveCamera(model_.get(), action, dx / height, dy / height, &scene_, &camera_);
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_{nullptr, mj_deleteModel};
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data_{nullptr, mj_deleteData};
  std::unique_ptr<mjData, decltype(&mj_deleteData)> shadow_{nullptr, mj_deleteData};
  std::unique_ptr<GLFWwindow, decltype(&glfwDestroyWindow)> window_{nullptr, glfwDestroyWindow};
  mjvCamera camera_{};
  mjvOption options_{};
  mjvPerturb perturb_{};
  mjvScene scene_{};
  mjrContext context_{};
  std::array<int, 2> object_qpos_{};
  std::array<int, 6> qpos_{}, qvel_{};
  std::array<int, 2> gripper_qpos_{}, gripper_qvel_{};
  lite6_interfaces::shm::ShmHandle<lite6_interfaces::shm::StateShmSegment> state_shm_;
  lite6_interfaces::shm::ShmHandle<lite6_interfaces::shm::PerturbShmSegment> perturb_shm_;
  uint64_t last_visual_seq_ = 0;
  uint64_t last_shadow_seq_ = 0;
  Clock::time_point last_state_{}, last_shadow_{};
  bool active_ = false, dirty_ = false, shadow_dirty_ = false;
  bool show_shadow_ = false, help_ = true;
  int fps_limit_ = 60;
  double last_x_ = 0, last_y_ = 0, last_click_ = -1, click_x_ = 0, click_y_ = 0;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  glfwSetErrorCallback([](int code, const char * message) {
    std::fprintf(stderr, "GLFW %d: %s\n", code, message);
  });
  if (!glfwInit()) {
    RCLCPP_ERROR(rclcpp::get_logger("mujoco_viewer"), "GLFW initialization failed; check DISPLAY/OpenGL");
    rclcpp::shutdown();
    return 1;
  }
  try {
    Viewer viewer(std::make_shared<rclcpp::Node>("mujoco_viewer"));
    viewer.run();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("mujoco_viewer"), "%s", error.what());
    result = 1;
  }
  glfwTerminate();
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return result;
}
