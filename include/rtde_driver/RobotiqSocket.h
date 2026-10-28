#pragma once

#include <string>
#include <utility>
#include <vector>

namespace rtde_driver
{

/// The Robotiq URCap's gripper socket on the UR control box (port 63352): one line per request, "SET <VAR> <n> ..."
/// answered "ack", "GET <VAR>" answered "<VAR> <n>". Variables: ACT, GTO, ATR, ADR, FOR, SPE, POS (write and read),
/// STA, PRE, OBJ, FLT (read). Port of callm_controller's robot/robotiq_gripper.py (callm_controller ca6e567), which is
/// taken from https://github.com/githubuser0xFFFF/py_robotiq_gripper.
///
/// Blocking, for one thread: every request waits for its reply, at most `timeout`. A socket error or a reply that does
/// not parse throws std::runtime_error and closes the socket.
class RobotiqSocket
{
public:
  RobotiqSocket() = default;
  ~RobotiqSocket();
  RobotiqSocket(const RobotiqSocket &) = delete;
  RobotiqSocket & operator=(const RobotiqSocket &) = delete;

  /// Throws std::runtime_error if the gripper socket does not accept within `timeout` [s]
  void open(const std::string & host, int port, double timeout);
  void close();
  bool isOpen() const
  {
    return fd_ >= 0;
  }

  /// Sets the variables in one request. False when the reply is not "ack": the set may not have been taken
  bool set(const std::vector<std::pair<std::string, int>> & vars);
  bool set(const std::string & var, int value)
  {
    return set({{var, value}});
  }

  /// The variable's value
  int get(const std::string & var);

private:
  /// Sends `line` and returns the reply, read until it holds `tokens` words or `timeout_` passes
  std::string request(const std::string & line, size_t tokens);
  [[noreturn]] void fail(const std::string & what);

  int fd_ = -1;
  double timeout_ = 1.0;
};

} // namespace rtde_driver
