#include <rtde_driver/Gripper.h>

#include <mc_rtc/Configuration.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fmt/core.h>
#include <stdexcept>

namespace rtde_driver
{

namespace
{

// Robotiq variables (robotiq_gripper.py)
const std::string ACT = "ACT"; // activate (1 while activated; 0 resets and clears faults)
const std::string GTO = "GTO"; // go to the position, speed and force set
const std::string ATR = "ATR"; // auto-release
const std::string FOR = "FOR"; // force 0..255
const std::string SPE = "SPE"; // speed 0..255
const std::string POS = "POS"; // position 0..255, 0 = open
const std::string STA = "STA"; // status: 0 reset, 1 activating, 3 active
const std::string PRE = "PRE"; // echo of the commanded position
const std::string OBJ = "OBJ"; // 0 moving, 1 stopped on an outer object, 2 on an inner object, 3 at the target
const std::string FLT = "FLT"; // fault, 0 = none

constexpr int STA_ACTIVE = 3;
constexpr int OBJ_MOVING = 0;
constexpr int OBJ_AT_TARGET = 3;

// 3 counts is about 1 mm of finger travel on a 2F-85, under what the hardware resolves (arm.py)
constexpr int DEADBAND_COUNTS = 3;
// Every full poll also reads STA and FLT
constexpr int FULL_POLL_EVERY = 10;
// Bounds of each wait during activation and calibration [s]
constexpr double RESET_TIMEOUT = 5.0;
constexpr double ACTIVATION_TIMEOUT = 10.0;
constexpr double MOVE_TIMEOUT = 10.0;

double nowReal()
{
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

int clampCount(double x, int lo, int hi)
{
  return std::clamp(static_cast<int>(std::lround(x)), lo, hi);
}

} // namespace

GripperConfig GripperConfig::load(const std::string & path)
{
  GripperConfig out;
  if(path.empty()) return out;
  if(!std::filesystem::exists(path)) throw std::invalid_argument("RTDE driver config " + path + " does not exist");
  const mc_rtc::Configuration config(path);
  if(!config.has("gripper")) return out;
  const auto gripper = config("gripper");
  out.enabled = true;
  gripper("host", out.host);
  gripper("port", out.port);
  gripper("speed", out.speed);
  gripper("force", out.force);
  gripper("poll_rate", out.pollRate);
  gripper("command_rate", out.commandRate);
  gripper("timeout", out.timeout);
  gripper("closed_angle", out.closedAngle);
  if(out.speed < 0 || out.speed > 255 || out.force < 0 || out.force > 255)
  {
    throw std::invalid_argument("RTDE driver config " + path + ": gripper speed and force must be within 0..255");
  }
  if(out.pollRate <= 0 || out.commandRate <= 0 || out.timeout <= 0)
  {
    throw std::invalid_argument("RTDE driver config " + path + ": gripper rates and timeout must be positive");
  }
  return out;
}

Gripper::Gripper(GripperConfig config, const std::string & robotHost)
: config_(std::move(config)), host_(config_.host.empty() ? robotHost : config_.host)
{
  targetSpeed_ = config_.speed;
  targetForce_ = config_.force;
  fmt::print("[RobotDriverRTDE] Gripper on {}:{}, speed {}, force {}\n", host_, config_.port, config_.speed,
             config_.force);
  std::fflush(stdout);
  thread_ = std::thread([this]() { run(); });
}

Gripper::~Gripper()
{
  stop_ = true;
  wake_.notify_all();
  if(thread_.joinable()) thread_.join();
}

void Gripper::setCommandValues(const std::map<std::string, double> & values)
{
  auto value = [&values](const char * key, double fallback)
  {
    const auto it = values.find(key);
    return it == values.end() ? fallback : it->second;
  };
  const double activate = value("gripper_activate", 0.0);
  const double stop = value("gripper_stop", 0.0);
  std::lock_guard<std::mutex> lock(commandMutex_);
  if(seen_)
  {
    if(activate > lastActivate_)
    {
      activatePending_ = true;
      activateRequested_ = activate;
    }
    if(stop > lastStop_)
    {
      stopPending_ = true;
      stopRequested_ = stop;
    }
  }
  seen_ = true;
  lastActivate_ = activate;
  lastStop_ = stop;
  target_ = value("gripper_position", std::numeric_limits<double>::quiet_NaN());
  targetSpeed_ = value("gripper_speed", config_.speed);
  targetForce_ = value("gripper_force", config_.force);
}

void Gripper::addStatus(std::map<std::string, double> & status) const
{
  std::lock_guard<std::mutex> lock(statusMutex_);
  status["gripper_state"] = static_cast<double>(state_);
  status["gripper_position"] = position_;
  status["gripper_poll_time"] = pollTime_;
  status["gripper_object"] = object_;
  status["gripper_status"] = status_;
  status["gripper_fault"] = fault_;
  status["gripper_min"] = min_;
  status["gripper_max"] = max_;
  status["gripper_target"] = sent_;
  status["gripper_last_move"] = lastMove_;
  status["gripper_stopped"] = stopped_ ? 1.0 : 0.0;
  status["gripper_stop_handled"] = stopHandled_;
  status["gripper_stop_acked"] = stopAcked_ ? 1.0 : 0.0;
  status["gripper_activate_handled"] = activateHandled_;
  status["gripper_speed"] = config_.speed;
  status["gripper_force"] = config_.force;
}

double Gripper::angle() const
{
  std::lock_guard<std::mutex> lock(statusMutex_);
  if(!std::isfinite(position_)) return 0.0;
  return std::clamp(position_, 0.0, 255.0) / 255.0 * config_.closedAngle;
}

void Gripper::run()
{
  using Clock = std::chrono::steady_clock;
  const auto period =
      std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / config_.commandRate));
  const auto pollPeriod =
      std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / config_.pollRate));
  auto next = Clock::now();
  auto nextPoll = Clock::now();
  int polls = 0;
  while(!stop_)
  {
    if(!socket_.isOpen() && !connect())
    {
      pause(1.0);
      next = nextPoll = Clock::now();
      continue;
    }
    try
    {
      bool activateNow = false;
      bool stopNow = false;
      double stopTime = 0.0;
      double activateTime = 0.0;
      double target = 0.0;
      double speed = 0.0;
      double force = 0.0;
      {
        std::lock_guard<std::mutex> lock(commandMutex_);
        std::swap(activateNow, activatePending_);
        std::swap(stopNow, stopPending_);
        stopTime = stopRequested_;
        activateTime = activateRequested_;
        target = target_;
        speed = targetSpeed_;
        force = targetForce_;
      }
      if(activateNow) activate(activateTime);
      if(stopNow) stopFingers(stopTime);
      if(std::isfinite(target)) write(target, speed, force);
      if(Clock::now() >= nextPoll)
      {
        poll(polls++ % FULL_POLL_EVERY == 0);
        nextPoll += pollPeriod;
        if(nextPoll < Clock::now()) nextPoll = Clock::now() + pollPeriod;
      }
    }
    catch(const std::exception & e)
    {
      if(!loggedDisconnect_)
      {
        fmt::print("[RobotDriverRTDE] Gripper: {}; connecting again\n", e.what());
        std::fflush(stdout);
        loggedDisconnect_ = true;
      }
      socket_.close();
      lastCommand_.reset();
      setState(State::Disconnected);
      continue;
    }
    next += period;
    const auto now = Clock::now();
    if(next < now) next = now + period;
    std::unique_lock<std::mutex> lock(wakeMutex_);
    wake_.wait_until(lock, next, [this]() { return stop_.load(); });
  }
}

bool Gripper::connect()
{
  try
  {
    socket_.open(host_, config_.port, config_.timeout);
    // A new connection may find a power-cycled gripper: a failed activation no longer holds
    activationFailed_ = false;
    const int sta = socket_.get(STA);
    const int pos = socket_.get(POS);
    {
      std::lock_guard<std::mutex> lock(statusMutex_);
      status_ = sta;
      position_ = pos;
      pollTime_ = nowReal();
    }
    setState(sta == STA_ACTIVE ? State::Active : State::Inactive);
    fmt::print("[RobotDriverRTDE] Gripper connected: {}, position {}\n", sta == STA_ACTIVE ? "active" : "not active",
               pos);
    std::fflush(stdout);
    loggedDisconnect_ = false;
    return true;
  }
  catch(const std::exception & e)
  {
    if(!loggedDisconnect_)
    {
      fmt::print("[RobotDriverRTDE] Gripper: {}; trying again every second\n", e.what());
      std::fflush(stdout);
      loggedDisconnect_ = true;
    }
    socket_.close();
    setState(State::Disconnected);
    return false;
  }
}

void Gripper::activate(double requested)
{
  activationFailed_ = false;
  {
    std::lock_guard<std::mutex> lock(statusMutex_);
    state_ = State::Activating;
    activateHandled_ = requested;
  }
  fmt::print("[RobotDriverRTDE] Gripper: activating\n");
  std::fflush(stdout);
  try
  {
    // As rq_activate_and_wait: reset, activate and wait, unless already active
    if(socket_.get(STA) != STA_ACTIVE)
    {
      socket_.set(ACT, 0);
      socket_.set(ATR, 0);
      waitFor(
          [this]()
          {
            if(socket_.get(ACT) == 0 && socket_.get(STA) == 0) return true;
            socket_.set(ACT, 0);
            socket_.set(ATR, 0);
            return false;
          },
          0.01, RESET_TIMEOUT, "the reset");
      pause(0.5);
      socket_.set(ACT, 1);
      pause(1.0);
      waitFor([this]() { return socket_.get(ACT) == 1 && socket_.get(STA) == STA_ACTIVE; }, 0.01, ACTIVATION_TIMEOUT,
              "the activation");
    }
    calibrate();
    lastCommand_.reset();
    {
      std::lock_guard<std::mutex> lock(statusMutex_);
      stopped_ = false;
      sent_ = std::numeric_limits<double>::quiet_NaN();
      lastMove_ = std::numeric_limits<double>::quiet_NaN();
    }
    setState(State::Active);
    fmt::print("[RobotDriverRTDE] Gripper active, travel {}..{}\n", min_, max_);
    std::fflush(stdout);
  }
  catch(const std::runtime_error & e)
  {
    // A socket error goes on to the reconnect; a timeout or a failed calibration leaves the gripper in fault
    if(!socket_.isOpen()) throw;
    fmt::print("[RobotDriverRTDE] Gripper activation failed: {}\n", e.what());
    std::fflush(stdout);
    activationFailed_ = true;
    setState(State::Fault);
  }
}

void Gripper::calibrate()
{
  // As auto_calibrate: open in case an object is held, close as far as it goes, open as far as it goes
  moveAndWait(min_, 64, 1);
  const int closed = moveAndWait(max_, 32, 1).first;
  {
    std::lock_guard<std::mutex> lock(statusMutex_);
    max_ = std::min(closed, max_);
  }
  const auto [open, openObject] = moveAndWait(min_, 64, 1);
  if(openObject != OBJ_AT_TARGET)
  {
    throw std::runtime_error(fmt::format("calibration stopped on an object (OBJ {})", openObject));
  }
  std::lock_guard<std::mutex> lock(statusMutex_);
  min_ = std::max(open, min_);
}

std::pair<int, int> Gripper::moveAndWait(int position, int speed, int force)
{
  const int command = std::clamp(position, min_, max_);
  if(!socket_.set({{POS, command}, {SPE, speed}, {FOR, force}, {GTO, 1}}))
  {
    throw std::runtime_error("the gripper did not acknowledge a move");
  }
  // The gripper takes the target, then moves until it arrives or meets an object
  waitFor([this, command]() { return socket_.get(PRE) == command; }, 0.001, MOVE_TIMEOUT, "the move to start");
  int object = OBJ_MOVING;
  waitFor(
      [this, &object]()
      {
        object = socket_.get(OBJ);
        return object != OBJ_MOVING;
      },
      0.001, MOVE_TIMEOUT, "the move to end");
  const int final = socket_.get(POS);
  {
    std::lock_guard<std::mutex> lock(statusMutex_);
    position_ = final;
    pollTime_ = nowReal();
    object_ = object;
  }
  return {final, object};
}

void Gripper::write(double position, double speed, double force)
{
  const Command request{clampCount(position, min_, max_), clampCount(speed, 0, 255), clampCount(force, 0, 255)};
  if(lastCommand_ && request.speed == lastCommand_->speed && request.force == lastCommand_->force)
  {
    // Already latched in the gripper, or too small a step to send, except onto either end of the travel
    const int step = std::abs(request.position - lastCommand_->position);
    const bool atEnd = request.position == min_ || request.position == max_;
    if(step == 0 || (step < DEADBAND_COUNTS && !atEnd)) return;
  }
  const bool acked = socket_.set({{POS, request.position}, {SPE, request.speed}, {FOR, request.force}, {GTO, 1}});
  // A write whose ack was lost is not remembered, so the next tick sends it again
  if(acked)
    lastCommand_ = request;
  else
    lastCommand_.reset();
  std::lock_guard<std::mutex> lock(statusMutex_);
  lastMove_ = nowReal();
  stopped_ = false;
  sent_ = request.position;
}

void Gripper::stopFingers(double requested)
{
  const bool acked = socket_.set(GTO, 0);
  // The next target goes out even if it repeats the last one
  lastCommand_.reset();
  if(!acked)
  {
    fmt::print("[RobotDriverRTDE] Gripper: the stop was not acknowledged\n");
    std::fflush(stdout);
  }
  std::lock_guard<std::mutex> lock(statusMutex_);
  stopped_ = acked;
  stopHandled_ = requested;
  stopAcked_ = acked;
}

void Gripper::poll(bool full)
{
  const int position = socket_.get(POS);
  // Stamped on this side of the read, as the arm's state is
  const double t = nowReal();
  const int object = socket_.get(OBJ);
  int sta = -1;
  int flt = -1;
  if(full)
  {
    sta = socket_.get(STA);
    flt = socket_.get(FLT);
  }
  std::lock_guard<std::mutex> lock(statusMutex_);
  position_ = position;
  pollTime_ = t;
  object_ = object;
  if(!full) return;
  status_ = sta;
  if(flt != fault_ && flt > 0)
  {
    fmt::print("[RobotDriverRTDE] Gripper fault: FLT {}\n", flt);
    std::fflush(stdout);
  }
  fault_ = flt;
  if(state_ == State::Activating) return;
  if(activationFailed_)
    state_ = State::Fault;
  else if(status_ == STA_ACTIVE)
    state_ = fault_ > 0 ? State::Fault : State::Active;
  else
    state_ = State::Inactive;
}

bool Gripper::pause(double seconds)
{
  std::unique_lock<std::mutex> lock(wakeMutex_);
  return !wake_.wait_for(lock, std::chrono::duration<double>(seconds), [this]() { return stop_.load(); });
}

template<typename Done>
void Gripper::waitFor(Done && done, double period, double timeout, const std::string & what)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
  while(!done())
  {
    if(stop_) throw std::runtime_error("the driver stopped during " + what);
    if(std::chrono::steady_clock::now() > deadline)
    {
      throw std::runtime_error(fmt::format("no end to {} within {} s", what, timeout));
    }
    pause(period);
  }
}

void Gripper::setState(State state)
{
  std::lock_guard<std::mutex> lock(statusMutex_);
  state_ = state;
}

} // namespace rtde_driver
