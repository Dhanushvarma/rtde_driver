// Gripper against a fake Robotiq URCap socket on localhost: activation and calibration, the write rules (deadband,
// no resend, the ends of the travel, a lost ack), stop, polling, the request values' first-command rule, and a
// reconnect after the socket drops.

#include <rtde_driver/Gripper.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace
{

int failures = 0;

#define CHECK(cond)                                                 \
  do                                                                \
  {                                                                 \
    if(!(cond))                                                     \
    {                                                               \
      std::printf("FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      ++failures;                                                   \
    }                                                               \
  } while(0)

double now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool waitUntil(const std::function<bool()> & done, double timeout)
{
  const double end = now() + timeout;
  while(now() < end)
  {
    if(done()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return done();
}

/// A 2F-85 behind the URCap socket: travel 3..227, 1000 counts/s, activation 0.2 s after ACT 1
class FakeRobotiq
{
public:
  FakeRobotiq()
  {
    listen_ = socket(AF_INET, SOCK_STREAM, 0);
    const int one = 1;
    setsockopt(listen_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    bind(listen_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
    socklen_t len = sizeof(addr);
    getsockname(listen_, reinterpret_cast<sockaddr *>(&addr), &len);
    port_ = ntohs(addr.sin_port);
    ::listen(listen_, 4);
    thread_ = std::thread([this]() { serve(); });
  }

  ~FakeRobotiq()
  {
    stop_ = true;
    thread_.join();
    ::close(listen_);
  }

  int port() const
  {
    return port_;
  }

  std::vector<std::string> sets()
  {
    std::lock_guard<std::mutex> lock(m_);
    return sets_;
  }
  size_t countSets(const std::string & part)
  {
    std::lock_guard<std::mutex> lock(m_);
    size_t n = 0;
    for(const auto & s : sets_) n += s.find(part) != std::string::npos;
    return n;
  }
  void clearSets()
  {
    std::lock_guard<std::mutex> lock(m_);
    sets_.clear();
  }
  int position()
  {
    std::lock_guard<std::mutex> lock(m_);
    return static_cast<int>(std::lround(pos_));
  }
  void setActive()
  {
    std::lock_guard<std::mutex> lock(m_);
    act_ = 1;
    sta_ = 3;
  }
  void nakNextSet()
  {
    std::lock_guard<std::mutex> lock(m_);
    nak_ = true;
  }
  void dropClient()
  {
    std::lock_guard<std::mutex> lock(m_);
    drop_ = true;
  }
  int accepted()
  {
    std::lock_guard<std::mutex> lock(m_);
    return accepted_;
  }

private:
  void serve()
  {
    while(!stop_)
    {
      pollfd p{listen_, POLLIN, 0};
      if(poll(&p, 1, 20) <= 0) continue;
      const int client = accept(listen_, nullptr, nullptr);
      if(client < 0) continue;
      {
        std::lock_guard<std::mutex> lock(m_);
        ++accepted_;
        drop_ = false;
      }
      std::string buffer;
      while(!stop_)
      {
        {
          std::lock_guard<std::mutex> lock(m_);
          if(drop_) break;
        }
        pollfd c{client, POLLIN, 0};
        if(poll(&c, 1, 5) <= 0) continue;
        char chunk[256];
        const ssize_t n = recv(client, chunk, sizeof(chunk), 0);
        if(n <= 0) break;
        buffer.append(chunk, static_cast<size_t>(n));
        size_t eol;
        while((eol = buffer.find('\n')) != std::string::npos)
        {
          const std::string reply = handle(buffer.substr(0, eol));
          buffer.erase(0, eol + 1);
          send(client, reply.data(), reply.size(), MSG_NOSIGNAL);
        }
      }
      ::close(client);
      // A dropped client finds nobody answering for a while, as when the URCap restarts
      bool dropped;
      {
        std::lock_guard<std::mutex> lock(m_);
        dropped = drop_;
      }
      if(dropped) std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
  }

  void simulate()
  {
    const double t = now();
    const double dt = t - last_;
    last_ = t;
    if(act_ == 1 && sta_ == 1 && t - activateTime_ > 0.2) sta_ = 3;
    if(sta_ != 3 || gto_ != 1) return;
    const double goal = std::clamp(static_cast<double>(pre_), 3.0, 227.0);
    const double step = 1000.0 * dt;
    if(std::abs(goal - pos_) <= step)
    {
      pos_ = goal;
      obj_ = 3;
    }
    else
    {
      pos_ += goal > pos_ ? step : -step;
      obj_ = 0;
    }
  }

  std::string handle(const std::string & line)
  {
    std::lock_guard<std::mutex> lock(m_);
    simulate();
    std::istringstream in(line);
    std::string verb;
    in >> verb;
    if(verb == "GET")
    {
      std::string var;
      in >> var;
      int value = -1;
      if(var == "POS")
        value = static_cast<int>(std::lround(pos_));
      else if(var == "PRE")
        value = pre_;
      else if(var == "OBJ")
        value = obj_;
      else if(var == "STA")
        value = sta_;
      else if(var == "ACT")
        value = act_;
      else if(var == "FLT")
        value = 0;
      return var + " " + std::to_string(value);
    }
    if(verb == "SET")
    {
      sets_.push_back(line);
      if(nak_)
      {
        nak_ = false;
        return "nak";
      }
      std::string var;
      int value;
      while(in >> var >> value)
      {
        if(var == "ACT")
        {
          if(value == 1 && act_ == 0)
          {
            sta_ = 1;
            activateTime_ = now();
          }
          if(value == 0)
          {
            sta_ = 0;
            gto_ = 0;
          }
          act_ = value;
        }
        else if(var == "POS")
          pre_ = value;
        else if(var == "GTO")
        {
          gto_ = value;
          if(value == 1 && sta_ == 3) obj_ = 0;
        }
      }
      return "ack";
    }
    return "?";
  }

  int listen_ = -1;
  int port_ = 0;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  std::mutex m_;
  std::vector<std::string> sets_;
  double last_ = now();
  double activateTime_ = 0.0;
  int act_ = 0;
  int sta_ = 0;
  int gto_ = 0;
  int pre_ = 0;
  int obj_ = 3;
  double pos_ = 120.0;
  bool nak_ = false;
  bool drop_ = false;
  int accepted_ = 0;
};

std::map<std::string, double> statusOf(const rtde_driver::Gripper & gripper)
{
  std::map<std::string, double> s;
  gripper.addStatus(s);
  return s;
}

using State = rtde_driver::Gripper::State;

double stateOf(const rtde_driver::Gripper & gripper)
{
  return statusOf(gripper)["gripper_state"];
}

struct Values
{
  double position = std::numeric_limits<double>::quiet_NaN();
  double speed = 150;
  double force = 10;
  double stop = 0;
  double activate = 0;
  std::map<std::string, double> map() const
  {
    return {{"gripper_position", position},
            {"gripper_speed", speed},
            {"gripper_force", force},
            {"gripper_stop", stop},
            {"gripper_activate", activate}};
  }
};

} // namespace

int main()
{
  FakeRobotiq fake;
  rtde_driver::GripperConfig config;
  config.enabled = true;
  config.host = "127.0.0.1";
  config.port = fake.port();
  config.timeout = 0.5;
  rtde_driver::Gripper gripper(config, "unused");
  Values v;
  // An old activation time in the first command must not activate
  v.activate = 100.0;
  gripper.setCommandValues(v.map());

  CHECK(waitUntil([&]() { return stateOf(gripper) == static_cast<double>(State::Inactive); }, 2.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  CHECK(fake.countSets("ACT 1") == 0);

  // Activation and calibration, on request
  v.activate = 200.0;
  gripper.setCommandValues(v.map());
  CHECK(waitUntil([&]() { return stateOf(gripper) == static_cast<double>(State::Activating); }, 1.0));
  CHECK(waitUntil([&]() { return stateOf(gripper) == static_cast<double>(State::Active); }, 10.0));
  auto s = statusOf(gripper);
  CHECK(s["gripper_min"] == 3);
  CHECK(s["gripper_max"] == 227);
  CHECK(fake.countSets("ACT 1") == 1);
  // The same activation value again does nothing
  gripper.setCommandValues(v.map());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  CHECK(fake.countSets("ACT 1") == 1);

  // Polling: the position and its stamps follow the fingers
  CHECK(waitUntil([&]() { return statusOf(gripper)["gripper_position"] == fake.position(); }, 1.0));
  const double t0 = statusOf(gripper)["gripper_poll_time"];
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  const double t1 = statusOf(gripper)["gripper_poll_time"];
  CHECK(t1 - t0 > 0.15 && t1 - t0 < 0.25);

  // One write per new target; a repeat and a sub-deadband step are not sent
  fake.clearSets();
  v.position = 100;
  gripper.setCommandValues(v.map());
  CHECK(waitUntil([&]() { return fake.countSets("SET POS 100 SPE 150 FOR 10 GTO 1") == 1; }, 1.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  CHECK(fake.sets().size() == 1);
  v.position = 101.6; // rounds to 102: a step of 2
  gripper.setCommandValues(v.map());
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  CHECK(fake.sets().size() == 1);
  v.position = 104;
  gripper.setCommandValues(v.map());
  CHECK(waitUntil([&]() { return fake.countSets("POS 104") == 1; }, 1.0));
  CHECK(waitUntil([&]() { return std::abs(fake.position() - 104) <= 1; }, 2.0));

  // Clipped to the travel; a sub-deadband step onto its end is sent
  v.position = 255;
  gripper.setCommandValues(v.map());
  CHECK(waitUntil([&]() { return fake.countSets("POS 227") == 1; }, 1.0));
  v.position = 225;
  gripper.setCommandValues(v.map());
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  CHECK(fake.countSets("POS 225") == 0);
  v.position = 226;
  gripper.setCommandValues(v.map());
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  CHECK(fake.countSets("POS 226") == 0);
  v.position = 3;
  gripper.setCommandValues(v.map());
  CHECK(waitUntil([&]() { return fake.countSets("POS 3 ") == 1; }, 1.0));
  v.position = 4;
  gripper.setCommandValues(v.map());
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  CHECK(fake.countSets("POS 4 ") == 0);
  // Another speed is a new command even at the same position
  v.speed = 80;
  gripper.setCommandValues(v.map());
  CHECK(waitUntil([&]() { return fake.countSets("POS 4 SPE 80") == 1; }, 1.0));
  v.speed = 150;

  // A stop: rGTO = 0, then the same target goes out again
  fake.clearSets();
  v.stop = 300.0;
  v.position = std::numeric_limits<double>::quiet_NaN();
  gripper.setCommandValues(v.map());
  CHECK(waitUntil([&]() { return fake.countSets("SET GTO 0") == 1; }, 1.0));
  CHECK(waitUntil([&]() { return statusOf(gripper)["gripper_stopped"] == 1.0; }, 1.0));
  CHECK(statusOf(gripper)["gripper_stop_handled"] == 300.0);
  CHECK(statusOf(gripper)["gripper_stop_acked"] == 1.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  CHECK(fake.sets().size() == 1);
  v.position = 4;
  gripper.setCommandValues(v.map());
  CHECK(waitUntil([&]() { return fake.countSets("POS 4 SPE 150") == 1; }, 1.0));
  CHECK(waitUntil([&]() { return statusOf(gripper)["gripper_stopped"] == 0.0; }, 1.0));

  // A lost ack is not remembered: the next tick sends the target again
  fake.clearSets();
  fake.nakNextSet();
  v.position = 150;
  gripper.setCommandValues(v.map());
  CHECK(waitUntil([&]() { return fake.countSets("POS 150") == 2; }, 1.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  CHECK(fake.countSets("POS 150") == 2);

  // The socket drops: disconnected, then connected again within about a second
  const int before = fake.accepted();
  fake.dropClient();
  CHECK(waitUntil([&]() { return stateOf(gripper) == static_cast<double>(State::Disconnected); }, 2.0));
  CHECK(waitUntil([&]() { return fake.accepted() == before + 1; }, 3.0));
  CHECK(waitUntil([&]() { return stateOf(gripper) == static_cast<double>(State::Active); }, 2.0));
  // After a reconnect the live target goes out once more
  CHECK(waitUntil([&]() { return fake.countSets("POS 150") == 3; }, 1.0));

  // The gripper joint: the polled counts in radians, 0.8 rad at 255
  CHECK(waitUntil(
      [&]() { return std::abs(gripper.angle() - 0.8 * statusOf(gripper)["gripper_position"] / 255.0) < 1e-9; }, 1.0));
  CHECK(gripper.angle() > 0.0);

  std::printf("%s (%d failed)\n", failures ? "FAILED" : "passed", failures);
  return failures ? 1 : 0;
}
