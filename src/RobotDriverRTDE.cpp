#include <rtde_driver/RobotDriverRTDE.h>

#include <ur_client_library/comm/control_mode.h>
#include <ur_client_library/control/reverse_interface.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fmt/core.h>
#include <iostream>
#include <stdexcept>
#include <variant>

namespace
{

std::string findUrScript()
{
  if(const char * env = std::getenv("UR_SCRIPT_FILE")) return env;

  const char * candidates[] = {
      "/opt/ros/humble/share/ur_client_library/resources/external_control.urscript",
      "/opt/ros/iron/share/ur_client_library/resources/external_control.urscript",
      "/opt/ros/jazzy/share/ur_client_library/resources/external_control.urscript",
      "/usr/share/ur_client_library/resources/external_control.urscript",
  };

  for(const char * p : candidates)
  {
    if(std::filesystem::exists(p)) return p;
  }

  throw std::runtime_error("[RobotDriverRTDE] Could not find external_control.urscript. "
                           "Set UR_SCRIPT_FILE to its path or source a ROS Humble/Iron/Jazzy workspace.");
}

inline std::vector<double> toVec(const urcl::vector6d_t & a)
{
  return {a.begin(), a.end()};
}

inline urcl::vector6d_t toArr(const std::vector<double> & v)
{
  urcl::vector6d_t a;
  std::copy_n(v.data(), a.size(), a.data());
  return a;
}

// A number from the data package. getData() needs the field's exact type (std::get on a variant), so the RTDE integer
// types are tried in turn.
bool readNumber(const urcl::rtde_interface::DataPackage & pkg, const std::string & name, double & out)
{
  try
  {
    int32_t v;
    if(pkg.getData(name, v))
    {
      out = v;
      return true;
    }
    return false;
  }
  catch(const std::bad_variant_access &)
  {
  }
  try
  {
    uint32_t v;
    if(pkg.getData(name, v))
    {
      out = v;
      return true;
    }
  }
  catch(const std::bad_variant_access &)
  {
  }
  return false;
}

} // namespace

namespace rtde_driver
{

RobotDriverRTDE::RobotDriverRTDE(const std::string & ip,
                                 uint16_t /*port*/,
                                 const std::string & config_path,
                                 const std::vector<robot_interface::GripperInfo> & grippers)
: ip_(ip)
{
  // Read before connecting, so a bad file fails at once
  const auto gripper_config = GripperConfig::load(config_path);
  for(const auto & gripper : grippers)
  {
    if(gripper.joints.size() != 1 || gripper_joint_)
    {
      throw std::invalid_argument("[RobotDriverRTDE] One gripper with one joint at most: the joint after the arm's, "
                                  "from the gripper's position");
    }
    gripper_joint_ = true;
    fmt::print("[RobotDriverRTDE] Gripper '{}': its joint '{}' comes after the arm's\n", gripper.name,
               gripper.joints.front());
  }
  if(gripper_joint_ && !gripper_config.enabled)
  {
    fmt::print("[RobotDriverRTDE] Warning: a gripper joint but no gripper section in the driver config: the joint "
               "stays at 0\n");
  }

  fmt::print("[RobotDriverRTDE] Connecting to UR robot at {}\n", ip_);

  // robot_mode, safety_mode and runtime_state go to the controller through getStatus(), and so do the sample time, the
  // TCP force and the TCP pose, for its state history and log
  output_recipe_ = {"actual_q",      "actual_qd", "target_moment",    "robot_mode",     "safety_mode",
                    "runtime_state", "timestamp", "actual_TCP_force", "actual_TCP_pose"};
  // Minimal input recipe (speed override controls are useful but not strictly required).
  const std::vector<std::string> input_recipe = {"speed_slider_mask", "speed_slider_fraction"};

  urcl::UrDriverConfiguration config;
  config.robot_ip = ip_;
  config.script_file = findUrScript();
  config.output_recipe = output_recipe_;
  config.input_recipe = input_recipe;
  config.headless_mode = true;
  config.handle_program_state = [this](bool running)
  {
    program_running_ = running;
    fmt::print("[RobotDriverRTDE] Program state: {}\n", running ? "running" : "stopped");
    std::fflush(stdout);
  };
  // Give up after 3 socket attempts (default 0 = infinite) so the process
  // remains interruptible when the robot is unreachable.
  config.socket_reconnect_attempts = 3;
  config.socket_reconnection_timeout = std::chrono::seconds(2);
  config.rtde_initialization_attempts = 3;

  driver_ = std::make_unique<urcl::UrDriver>(config);

  // Start RTDE communication without a background reader — sync() drives the loop via blocking reads.
  driver_->startRTDECommunication(false);

  // The headless driver has just sent the program: give it time to connect back before sending it again
  next_send_ = std::chrono::steady_clock::now() + kStartupGrace;

  // Pre-allocate the data package with the exact recipe to avoid per-cycle allocation.
  data_pkg_ = std::make_unique<urcl::rtde_interface::DataPackage>(driver_->getRTDEOutputRecipe());

  fmt::print("[RobotDriverRTDE] Connected to {}\n", ip_);

  if(gripper_config.enabled) gripper_ = std::make_unique<Gripper>(gripper_config, ip_);
}

RobotDriverRTDE::~RobotDriverRTDE()
{
  gripper_.reset();
  if(driver_) driver_->stopControl();
}

void RobotDriverRTDE::sync()
{
  // Block until the robot delivers a new RTDE package — this paces the control loop.
  if(!driver_->getDataPackageBlocking(data_pkg_))
    fmt::print("[RobotDriverRTDE] Warning: failed to receive RTDE data package\n");
  // Stamped on this side of the read, as callm_controller's state ring stamps robot_receive_timestamp
  read_time_ = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();

  double robot_mode = 0.0;
  double safety_mode = 0.0;
  double runtime_state = 0.0;
  normal_ = data_pkg_ && readNumber(*data_pkg_, "robot_mode", robot_mode)
            && readNumber(*data_pkg_, "safety_mode", safety_mode) && robot_mode == 7.0
            && (safety_mode == 1.0 || safety_mode == 2.0);
  const bool playing = data_pkg_ && readNumber(*data_pkg_, "runtime_state", runtime_state)
                       && runtime_state == static_cast<double>(urcl::rtde_interface::RUNTIME_STATE::PLAYING);
  running_ = program_running_ && playing;
  // A stop (protective stop, e-stop, the program stopping) closes the gate: the next motion waits for the controller
  // to catch up with the arm, whether the program stopped or only paused
  if(!running_ || !normal_) gate_closed_ = true;
  const auto now = std::chrono::steady_clock::now();
  // A program that just stopped is sent again kResendPeriod later at the earliest
  if(was_running_ && !running_) next_send_ = std::max(next_send_, now + kResendPeriod);
  was_running_ = running_;
  if(running_ || !normal_) return;

  // The program stopped or paused and the operator cleared the stop: send it again. A program sent now replaces a
  // paused one, as when uri manager restarted after an e-stop in Remote Control (2026-10-28)
  if(now >= next_send_)
  {
    next_send_ = now + kResendPeriod;
    const bool sent = driver_->sendRobotProgram();
    fmt::print("[RobotDriverRTDE] Robot running again: {} the external-control program\n",
               sent ? "sent" : "could not send");
    std::fflush(stdout);
  }
}

std::vector<double> RobotDriverRTDE::withGripperJoint(std::vector<double> arm, double gripper) const
{
  if(gripper_joint_ && !arm.empty()) arm.push_back(gripper);
  return arm;
}

std::vector<double> RobotDriverRTDE::getActualQ()
{
  urcl::vector6d_t q;
  if(data_pkg_ && data_pkg_->getData("actual_q", q))
    return withGripperJoint(toVec(q), gripper_ ? gripper_->angle() : 0.0);
  return {};
}

std::vector<double> RobotDriverRTDE::getActualQd()
{
  urcl::vector6d_t qd;
  if(data_pkg_ && data_pkg_->getData("actual_qd", qd)) return withGripperJoint(toVec(qd), 0.0);
  return {};
}

std::vector<double> RobotDriverRTDE::getJointTorques()
{
  urcl::vector6d_t tau;
  if(data_pkg_ && data_pkg_->getData("target_moment", tau)) return withGripperJoint(toVec(tau), 0.0);
  return {};
}

std::map<std::string, double> RobotDriverRTDE::getStatus()
{
  std::map<std::string, double> status{{"program_running", running_ ? 1.0 : 0.0}};
  if(!data_pkg_) return status;
  for(const char * name : {"robot_mode", "safety_mode", "runtime_state"})
  {
    double value = 0.0;
    if(readNumber(*data_pkg_, name, value)) status[name] = value;
  }
  double timestamp = 0.0;
  if(data_pkg_->getData("timestamp", timestamp)) status["timestamp"] = timestamp;
  status["read_time"] = read_time_;
  urcl::vector6d_t v;
  if(data_pkg_->getData("actual_TCP_force", v))
  {
    for(size_t i = 0; i < v.size(); ++i) status[fmt::format("tcp_force_{}", i)] = v[i];
  }
  if(data_pkg_->getData("actual_TCP_pose", v))
  {
    for(size_t i = 0; i < v.size(); ++i) status[fmt::format("tcp_pose_{}", i)] = v[i];
  }
  if(gripper_) gripper_->addStatus(status);
  return status;
}

void RobotDriverRTDE::setCommandValues(const std::map<std::string, double> & values)
{
  if(gripper_) gripper_->setCommandValues(values);
}

void RobotDriverRTDE::servoJ(const std::vector<double> & q)
{
  // Commands go to the program only while it plays. A paused program (an e-stop in Remote Control pauses it) keeps its
  // socket open but reads nothing: the commands would pile up until the write blocks this loop, RTDE reads included
  if(!running_) return;
  auto command = toArr(q);
  if(gate_closed_)
  {
    // Hold the actual joints until the controller's command is close to them
    urcl::vector6d_t actual;
    if(!data_pkg_ || !data_pkg_->getData("actual_q", actual)) return;
    double gap = 0.0;
    for(size_t i = 0; i < actual.size(); ++i) gap = std::max(gap, std::abs(command[i] - actual[i]));
    if(normal_ && gap < kGateTolerance)
    {
      gate_closed_ = false;
      fmt::print("[RobotDriverRTDE] Following the controller's commands (within {:.4f} rad)\n", gap);
      std::fflush(stdout);
    }
    else
    {
      command = actual;
    }
  }
  const bool ok = driver_->writeJointCommand(command, urcl::comm::ControlMode::MODE_SERVOJ,
                                             urcl::RobotReceiveTimeout::millisec(20));
  // Fails when the program stopped since the last package: said once per stop
  if(!ok && !logged_write_failure_) std::cout << "[RobotDriverRTDE] servoJ: writeJointCommand returned false\n";
  logged_write_failure_ = !ok;
}

void RobotDriverRTDE::speedJ(const std::vector<double> & alpha)
{
  // Only while the program plays, as in servoJ()
  if(!running_) return;
  driver_->writeJointCommand(toArr(alpha), urcl::comm::ControlMode::MODE_SPEEDJ,
                             urcl::RobotReceiveTimeout::millisec(20));
}

void RobotDriverRTDE::tauJ(const std::vector<double> & tau)
{
  // Only while the program plays, as in servoJ()
  if(!running_) return;
  driver_->writeJointCommand(toArr(tau), urcl::comm::ControlMode::MODE_TORQUE, urcl::RobotReceiveTimeout::millisec(20));
}

bool RobotDriverRTDE::freeDrive(bool enable)
{
  const auto action = enable ? urcl::control::FreedriveControlMessage::FREEDRIVE_START
                             : urcl::control::FreedriveControlMessage::FREEDRIVE_STOP;
  return driver_->writeFreedriveControlMessage(action);
}

} // namespace rtde_driver

// Lets robot_interface refuse this plugin (instead of crashing) once the
// RobotDriver interface changes and the plugin needs a rebuild.
ROBOT_DRIVER_EXPORT_ABI_VERSION()

extern "C"
{
  void ROBOT_DRIVER_PLUGIN(std::vector<std::string> & classes)
  {
    classes.push_back("RobotDriverRTDE");
  }

  robot_interface::RobotDriver * create(const std::string & /*name*/,
                                        const std::string & ip,
                                        const uint16_t & port,
                                        const std::string & config_path,
                                        const std::vector<robot_interface::GripperInfo> & grippers)
  {
    return new rtde_driver::RobotDriverRTDE(ip, port, config_path, grippers);
  }

  void destroy(robot_interface::RobotDriver * ptr)
  {
    delete ptr;
  }
}
