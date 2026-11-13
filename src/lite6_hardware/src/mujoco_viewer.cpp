// A deliberately small native MuJoCo viewer. GLFW, ROS callbacks and all display
// mjData live on the main thread; the hardware process owns the physics state.
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "lite6_interfaces/msg/simulation_state.hpp"
#include "lite6_interfaces/msg/mouse_perturbation.hpp"

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
using State = lite6_interfaces::msg::SimulationState;
using Target = lite6_interfaces::msg::MousePerturbation;
int64_t now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    Clock::now().time_since_epoch()).count();
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
    mj_forward(model_.get(), data_.get());
    mj_forward(model_.get(), shadow_.get());
    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&options_);
    mjv_defaultPerturb(&perturb_);
    mjv_defaultScene(&scene_);
    mjr_defaultContext(&context_);
    options_.geomgroup[3] = 0;  // Collision meshes must not cover the visual meshes.
    options_.flags[mjVIS_CONVEXHULL] = 0;  // Render the full visual meshes.
    options_.sitegroup[4] = 0;
    reset_camera();

    publisher_ = node_->create_publisher<Target>(
      "/simulation/mouse_perturbation", rclcpp::QoS(1).best_effort());
    state_subscription_ = node_->create_subscription<State>(
      "/simulation/visual_state", rclcpp::QoS(1).best_effort(),
      [this](const State & state) {
        for (size_t i = 0; i < 6; ++i) {
          if (!std::isfinite(state.position[i]) || !std::isfinite(state.velocity[i])) {return;}
        }
        for (size_t i = 0; i < 6; ++i) {
          data_->qpos[qpos_[i]] = state.position[i];
          data_->qvel[qvel_[i]] = state.velocity[i];
        }
        data_->time = state.simulation_time;
        active_ = state.active;
        last_state_ = Clock::now();
        dirty_ = true;
      });
    shadow_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/shadow/joint_states", rclcpp::QoS(1).best_effort(),
      [this](const sensor_msgs::msg::JointState & state) {
        std::array<double, 6> positions{};
        for (size_t i = 0; i < 6; ++i) {
          const auto name = "joint" + std::to_string(i + 1);
          const auto it = std::find(state.name.begin(), state.name.end(), name);
          const auto index = static_cast<size_t>(it - state.name.begin());
          if (it == state.name.end() || index >= state.position.size() ||
            !std::isfinite(state.position[index])) {return;}
          positions[i] = state.position[index];
        }
        for (size_t i = 0; i < 6; ++i) {shadow_->qpos[qpos_[i]] = positions[i];}
        last_shadow_ = Clock::now();
        shadow_dirty_ = true;
      });

    glfwWindowHint(GLFW_SAMPLES, 8);
    glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
    int width = 1920, height = 1080;
    if (auto * monitor = glfwGetPrimaryMonitor()) {
      int work_x, work_y, work_width, work_height;
      glfwGetMonitorWorkarea(monitor, &work_x, &work_y, &work_width, &work_height);
      // Leave room for window decorations, including on scaled desktops.
      if (work_width > 80 && work_height > 100) {
        width = std::min(width, work_width - 80);
        height = std::min(height, work_height - 100);
      }
    }
    window_.reset(glfwCreateWindow(width, height, "Lite6 | MuJoCo Studio", nullptr, nullptr));
    if (!window_) {throw std::runtime_error("Cannot create an OpenGL window (check DISPLAY)");}
    glfwMakeContextCurrent(window_.get());
    glfwSwapInterval(1);
    mjv_makeScene(model_.get(), &scene_, 512);
    mjr_makeContext(model_.get(), &context_, mjFONTSCALE_150);
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
      executor.spin_some(std::chrono::milliseconds(2));
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
          mjv_addGeoms(model_.get(), shadow_.get(), &options_, nullptr, mjCAT_DYNAMIC, &scene_);
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
          "LITE6 / MUJOCO STUDIO", status, &context_);
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
  void send_target()
  {
    if (!perturb_.active) {return;}
    Target message;
    message.issued_ns = now_ns();
    message.body_id = perturb_.select;
    message.active = perturb_.active;
    std::copy_n(perturb_.refpos, 3, message.refpos.begin());
    std::copy_n(perturb_.refselpos, 3, message.refselpos.begin());
    std::copy_n(perturb_.refquat, 4, message.refquat.begin());
    std::copy_n(perturb_.localpos, 3, message.localpos.begin());
    message.localmass = perturb_.localmass;
    publisher_->publish(message);
  }
  void release()
  {
    if (perturb_.active && rclcpp::ok()) {publisher_->publish(Target{});}
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
  std::array<int, 6> qpos_{}, qvel_{};
  rclcpp::Publisher<Target>::SharedPtr publisher_;
  rclcpp::Subscription<State>::SharedPtr state_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr shadow_subscription_;
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
