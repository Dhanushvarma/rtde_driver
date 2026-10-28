#pragma once

#include <robot_interface/RobotDriverTemplate.h>
#include <robot_interface/driver/GripperInfo.h>

#include <rtde_driver/Gripper.h>

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <ur_client_library/rtde/data_package.h>
#include <ur_client_library/ur/ur_driver.h>

namespace rtde_driver
{

class RobotDriverRTDE : public robot_interface::RobotDriver
{
public:
  // config_path: optional driver config file (robot_interface.config_path); its `gripper` section drives a Robotiq
  // gripper through the UR's URCap socket (see Gripper). grippers: the robot's grippers in the URI config (`grippers:`,
  // name and joints); a declared gripper joint is reported after the six arm joints, from the gripper's position.
  RobotDriverRTDE(const std::string & ip,
                  uint16_t port = 0,
                  const std::string & config_path = "",
                  const std::vector<robot_interface::GripperInfo> & grippers = {});

  ~RobotDriverRTDE() override;

  void sync() override;

  void setDataRead() override {}

  std::vector<double> getActualQ() override;
  std::vector<double> getActualQd() override;
  std::vector<double> getJointTorques() override;

  void servoJ(const std::vector<double> & q) override;
  void speedJ(const std::vector<double> & alpha) override;
  void tauJ(const std::vector<double> & tau) override;

  // robot_mode, safety_mode and runtime_state from RTDE, and program_running: 1 while the external-control program
  // runs (urcl's program state, and RTDE's runtime_state PLAYING). A controller stops when the robot leaves normal mode
  // or the program stops. Also, for the controller's state history: timestamp (the UR's own sample time [s]),
  // read_time (this host's CLOCK_REALTIME when the package arrived [s]), tcp_force_0..5 and tcp_pose_0..5 (RTDE's
  // actual_TCP_force and actual_TCP_pose); and the gripper's values (Gripper::addStatus) when there is one.
  std::map<std::string, double> getStatus() override;

  // The gripper's command values (Gripper::setCommandValues); ignored without a gripper
  void setCommandValues(const std::map<std::string, double> & values) override;

  // Enable / disable freedrive mode.
  bool freeDrive(bool enable);

private:
  /// The six arm joints, then the gripper joint when there is one
  std::vector<double> withGripperJoint(std::vector<double> arm, double gripper) const;

  std::string ip_;
  std::vector<std::string> output_recipe_;
  std::unique_ptr<Gripper> gripper_;
  // A gripper joint is declared in the URI config: one more value after the arm's
  bool gripper_joint_ = false;
  // This host's CLOCK_REALTIME when the last package arrived [s]
  double read_time_ = 0.0;
  std::unique_ptr<urcl::UrDriver> driver_;
  std::unique_ptr<urcl::rtde_interface::DataPackage> data_pkg_;
  // Set from urcl's thread when the external-control program starts or stops
  std::atomic<bool> program_running_{false};

  // After a stop, the program is sent again once the robot is back in normal mode (see sync()). Until the controller's
  // command comes within kGateTolerance of the actual joints, the arm holds where it stands: the controller's command
  // may have moved while the arm was stopped. The gate acts in position mode only (servoJ), not on speedJ or tauJ.
  static constexpr double kGateTolerance = 0.01; // rad
  static constexpr std::chrono::seconds kResendPeriod{2};
  // The program sent at startup takes a moment to connect back: no re-send before this
  static constexpr std::chrono::seconds kStartupGrace{5};
  bool gate_closed_ = true;
  // robot_mode RUNNING and safety_mode NORMAL or REDUCED in the last package
  bool normal_ = false;
  // The external-control program runs: urcl's program state, and RTDE's runtime_state PLAYING. A power cycle can stop
  // the program while its socket stays connected, so the socket alone is not enough
  bool running_ = false;
  // The program ran at the last cycle
  bool was_running_ = false;
  bool logged_write_failure_ = false;
  // No re-send before this time: a startup grace, then kResendPeriod after each send
  std::chrono::steady_clock::time_point next_send_{};
};

} // namespace rtde_driver

// ── robot_interface plugin symbols ────────────────────────────────────────────
#include <robot_interface/driver/api.h>

extern "C"
{
  ROBOT_DRIVER_DLLAPI void ROBOT_DRIVER_PLUGIN(std::vector<std::string> & classes);

  // grippers: the URI config's grippers of this robot, see RobotDriverRTDE's constructor
  ROBOT_DRIVER_DLLAPI robot_interface::RobotDriver * create(const std::string & name,
                                                            const std::string & ip,
                                                            const uint16_t & port,
                                                            const std::string & config_path,
                                                            const std::vector<robot_interface::GripperInfo> & grippers);

  ROBOT_DRIVER_DLLAPI void destroy(robot_interface::RobotDriver * ptr);
}
