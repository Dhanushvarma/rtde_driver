#include <rtde_driver/RobotiqSocket.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <sstream>
#include <stdexcept>

namespace rtde_driver
{

namespace
{

size_t countTokens(const std::string & s)
{
  std::istringstream in(s);
  size_t n = 0;
  std::string word;
  while(in >> word) ++n;
  return n;
}

} // namespace

RobotiqSocket::~RobotiqSocket()
{
  close();
}

void RobotiqSocket::open(const std::string & host, int port, double timeout)
{
  close();
  timeout_ = timeout;

  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo * found = nullptr;
  const std::string service = std::to_string(port);
  if(getaddrinfo(host.c_str(), service.c_str(), &hints, &found) != 0 || found == nullptr)
  {
    throw std::runtime_error("[RobotiqSocket] Cannot resolve " + host);
  }

  fd_ = socket(found->ai_family, found->ai_socktype, found->ai_protocol);
  if(fd_ < 0)
  {
    freeaddrinfo(found);
    fail("socket()");
  }
  // A non-blocking connect, so an unreachable host costs `timeout`, not the kernel's minutes
  const int flags = fcntl(fd_, F_GETFL, 0);
  fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
  const int rc = connect(fd_, found->ai_addr, found->ai_addrlen);
  freeaddrinfo(found);
  if(rc < 0 && errno != EINPROGRESS) fail("connect to " + host + ":" + service);
  if(rc < 0)
  {
    pollfd p{fd_, POLLOUT, 0};
    if(poll(&p, 1, static_cast<int>(timeout_ * 1000)) <= 0)
    {
      close();
      throw std::runtime_error("[RobotiqSocket] No connection to " + host + ":" + service + " within "
                               + std::to_string(timeout_) + " s");
    }
    int error = 0;
    socklen_t len = sizeof(error);
    getsockopt(fd_, SOL_SOCKET, SO_ERROR, &error, &len);
    if(error != 0)
    {
      errno = error;
      fail("connect to " + host + ":" + service);
    }
  }
  fcntl(fd_, F_SETFL, flags);
  // One short request at a time: send it now rather than wait to fill a segment
  const int one = 1;
  setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
}

void RobotiqSocket::close()
{
  if(fd_ >= 0) ::close(fd_);
  fd_ = -1;
}

bool RobotiqSocket::set(const std::vector<std::pair<std::string, int>> & vars)
{
  std::string line = "SET";
  for(const auto & [var, value] : vars) line += " " + var + " " + std::to_string(value);
  // "ack" carries no newline: the reply is complete once it holds one word
  std::string reply = request(line + "\n", 1);
  while(!reply.empty() && std::isspace(static_cast<unsigned char>(reply.back()))) reply.pop_back();
  return reply == "ack";
}

int RobotiqSocket::get(const std::string & var)
{
  const std::string reply = request("GET " + var + "\n", 2);
  std::istringstream in(reply);
  std::string name;
  std::string value;
  in >> name >> value;
  errno = 0;
  if(name != var) fail("unexpected reply '" + reply + "' to GET " + var);
  try
  {
    return std::stoi(value);
  }
  catch(const std::exception &)
  {
    fail("unexpected reply '" + reply + "' to GET " + var);
  }
}

std::string RobotiqSocket::request(const std::string & line, size_t tokens)
{
  if(fd_ < 0) throw std::runtime_error("[RobotiqSocket] Not connected");
  size_t sent = 0;
  while(sent < line.size())
  {
    const ssize_t n = send(fd_, line.data() + sent, line.size() - sent, MSG_NOSIGNAL);
    if(n < 0)
    {
      if(errno == EINTR) continue;
      fail("send");
    }
    sent += static_cast<size_t>(n);
  }

  std::string reply;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_);
  char buffer[256];
  while(countTokens(reply) < tokens)
  {
    const auto left =
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
    if(left.count() <= 0)
    {
      errno = 0;
      fail("no reply to '" + line.substr(0, line.size() - 1) + "'");
    }
    pollfd p{fd_, POLLIN, 0};
    const int ready = poll(&p, 1, static_cast<int>(left.count()));
    if(ready < 0 && errno == EINTR) continue;
    if(ready <= 0) continue;
    const ssize_t n = recv(fd_, buffer, sizeof(buffer), 0);
    if(n == 0)
    {
      errno = ECONNRESET;
      fail("recv");
    }
    if(n < 0)
    {
      if(errno == EINTR) continue;
      fail("recv");
    }
    reply.append(buffer, static_cast<size_t>(n));
  }
  return reply;
}

void RobotiqSocket::fail(const std::string & what)
{
  const std::string reason = errno != 0 ? std::string(": ") + std::strerror(errno) : std::string{};
  close();
  throw std::runtime_error("[RobotiqSocket] " + what + reason);
}

} // namespace rtde_driver
