#pragma once

#include <robot_interface/RobotDriverTemplate.h>
#include <robot_interface/driver/GripperInfo.h>

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
  // config_path: unused by this driver (RTDE is configured entirely via ip/port),
  // but present so every RobotDriver's create() shares the same signature —
  // see robot_interface/PluginLoader.h and RobotInterface::loadDriver().
  RobotDriverRTDE(const std::string & ip, uint16_t port = 0, const std::string & config_path = "");

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
  // runs (urcl's program state). A controller stops when the robot leaves normal mode or the program stops.
  std::map<std::string, double> getStatus() override;

  // Enable / disable freedrive mode.
  bool freeDrive(bool enable);

private:
  std::string ip_;
  std::vector<std::string> output_recipe_;
  std::unique_ptr<urcl::UrDriver> driver_;
  std::unique_ptr<urcl::rtde_interface::DataPackage> data_pkg_;
  // Set from urcl's thread when the external-control program starts or stops
  std::atomic<bool> program_running_{false};

  // After a stop, the program is sent again once the robot is back in normal mode (see sync()). Until the controller's
  // command comes within kGateTolerance of the actual joints, the arm holds where it stands: the controller's command
  // may have moved while the arm was stopped.
  static constexpr double kGateTolerance = 0.01; // rad
  static constexpr std::chrono::seconds kResendPeriod{2};
  bool gate_closed_ = true;
  // robot_mode RUNNING and safety_mode NORMAL or REDUCED in the last package
  bool normal_ = false;
  bool logged_write_failure_ = false;
  std::chrono::steady_clock::time_point last_resend_{};
};

} // namespace rtde_driver

// ── robot_interface plugin symbols ────────────────────────────────────────────
#include <robot_interface/driver/api.h>

extern "C"
{
  ROBOT_DRIVER_DLLAPI void ROBOT_DRIVER_PLUGIN(std::vector<std::string> & classes);

  // grippers: unused by this driver (RTDE robots have no grippers), but
  // present so every RobotDriver's create() shares the same signature —
  // see robot_interface/PluginLoader.h and RobotInterface::loadDriver().
  ROBOT_DRIVER_DLLAPI robot_interface::RobotDriver * create(const std::string & name,
                                                            const std::string & ip,
                                                            const uint16_t & port,
                                                            const std::string & config_path,
                                                            const std::vector<robot_interface::GripperInfo> & grippers);

  ROBOT_DRIVER_DLLAPI void destroy(robot_interface::RobotDriver * ptr);
}
