#ifndef KEY_EVENT_HPP
#define KEY_EVENT_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ncurses.h>
#include <thread>
#include <atomic>
#include <string>

namespace key_event {

class KeyEvent : public rclcpp::Node {
public:
  KeyEvent();
  ~KeyEvent();

private:
  void run_key_listener();
  void publish_key();

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string last_key_;
  std::atomic<bool> running_;
  std::thread key_thread_;
};

} // namespace key_event

#endif // KEY_EVENT_HPP
