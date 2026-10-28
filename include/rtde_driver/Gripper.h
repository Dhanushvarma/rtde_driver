#pragma once

#include <rtde_driver/RobotiqSocket.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

namespace rtde_driver
{

/// The `gripper` section of the driver config file (`robot_interface.config_path`). Without one there is no gripper.
struct GripperConfig
{
  bool enabled = false;
  /// Host of the gripper socket; empty: the robot's address
  std::string host;
  int port = 63352;
  /// Speed and force of a target that gives none [0..255]
  int speed = 150;
  int force = 10;
  /// Position polls per second
  double pollRate = 100.0;
  /// Command loop rate, so at most this many writes per second
  double commandRate = 200.0;
  /// Socket timeout [s]
  double timeout = 1.0;
  /// Angle of the finger joint at 255 counts [rad], for the gripper joint the driver reports when URI declares one
  double closedAngle = 0.8;

  /// The section of the file at `path`; a disabled config when the path is empty or the file has no section
  static GripperConfig load(const std::string & path);
};

/// A Robotiq 2F gripper behind the UR's URCap socket, run on its own thread so that the RTDE loop never waits on it.
/// Port of the gripper part of callm_controller's arm.py (callm_controller ca6e567), with its rules:
/// - activation only on request: reset if not active, activate, then calibrate the travel (open, close, open);
/// - a target is clipped to the calibrated travel and never sent again while it is the live command; a step under
///   3 counts is skipped, except onto either end of the travel, where the grip force builds;
/// - one request at a time on the socket, each waiting for its reply; a write whose ack is lost is not remembered, so
/// the
///   next tick sends it again;
/// - a stop is rGTO = 0; the next target goes out even if it repeats the last one.
///
/// The controller drives it through RobotDriver::setCommandValues, every command:
/// - `gripper_position` [counts], the target, NaN for none; `gripper_speed`, `gripper_force`;
/// - `gripper_stop`, `gripper_activate`: the NUC time of the newest stop or activation request. A value larger than the
///   last one asks for it; the first command only sets the reference, so a restart does not replay an old request.
/// Whether a stop is needed (moving fingers, or at rest) is the controller's call, as stop_gripper's was; the status
/// says which stop request was sent last (`gripper_stop_handled`) and whether the gripper acknowledged it.
class Gripper
{
public:
  enum class State
  {
    Disconnected = 1,
    Inactive = 2,
    Activating = 3,
    Active = 4,
    Fault = 5
  };

  Gripper(GripperConfig config, const std::string & robotHost);
  ~Gripper();
  Gripper(const Gripper &) = delete;
  Gripper & operator=(const Gripper &) = delete;

  /// The controller's values, from the RTDE loop; returns at once
  void setCommandValues(const std::map<std::string, double> & values);
  /// Adds the gripper_* status values
  void addStatus(std::map<std::string, double> & status) const;
  /// The finger joint's angle [rad] at the newest polled position, 0 before the first poll
  double angle() const;

private:
  struct Command
  {
    int position;
    int speed;
    int force;
  };

  void run();
  bool connect();
  void activate();
  void calibrate();
  /// As robotiq_gripper.py's move_and_wait_for_pos: the final position and object status
  std::pair<int, int> moveAndWait(int position, int speed, int force);
  void write(double position, double speed, double force);
  /// rGTO = 0, for the stop request of time `requested`
  void stopFingers(double requested);
  void poll(bool full);
  /// Sleeps `seconds`, or less if the driver stops; false once it stops
  bool pause(double seconds);
  /// Waits until `done` holds, polling every `period` [s], at most `timeout` [s]; throws std::runtime_error after it
  template<typename Done>
  void waitFor(Done && done, double period, double timeout, const std::string & what);
  void setState(State state);

  const GripperConfig config_;
  const std::string host_;
  RobotiqSocket socket_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  std::mutex wakeMutex_;
  std::condition_variable wake_;

  // Set by setCommandValues, taken by the thread
  mutable std::mutex commandMutex_;
  bool seen_ = false;
  double lastActivate_ = 0.0;
  double lastStop_ = 0.0;
  bool activatePending_ = false;
  bool stopPending_ = false;
  double stopRequested_ = 0.0;
  double target_ = std::numeric_limits<double>::quiet_NaN();
  double targetSpeed_ = 0.0;
  double targetForce_ = 0.0;

  // The thread's own
  std::optional<Command> lastCommand_;
  bool loggedDisconnect_ = false;
  // Until the next activation
  bool activationFailed_ = false;

  // Status, written by the thread
  mutable std::mutex statusMutex_;
  State state_ = State::Disconnected;
  // The calibrated travel [counts]; read by the thread without the lock
  int min_ = 0;
  int max_ = 255;
  double position_ = std::numeric_limits<double>::quiet_NaN();
  double pollTime_ = std::numeric_limits<double>::quiet_NaN();
  int object_ = -1;
  int status_ = -1;
  int fault_ = -1;
  double sent_ = std::numeric_limits<double>::quiet_NaN();
  double lastMove_ = std::numeric_limits<double>::quiet_NaN();
  bool stopped_ = false;
  // The request time of the last stop sent, and whether the gripper acknowledged it
  double stopHandled_ = 0.0;
  bool stopAcked_ = false;
};

} // namespace rtde_driver
