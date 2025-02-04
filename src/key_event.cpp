#include "../include/key_event/key_event.hpp"

namespace key_event {

KeyEvent::KeyEvent() : Node("key_event"), last_key_(""), running_(true) {
  // 퍼블리셔 설정 (String 타입)
  publisher_ = this->create_publisher<std_msgs::msg::String>("/key_event", 10);

  // 타이머 설정 (100ms 간격으로 마지막 키 입력값 퍼블리쉬)
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                   std::bind(&KeyEvent::publish_key, this));

  // 키보드 입력 감지 스레드 실행
  run_key_listener();
}

KeyEvent::~KeyEvent() {
  running_ = false;
  if (key_thread_.joinable()) {
    key_thread_.join();
  }
  endwin(); // ncurses 종료
}

void KeyEvent::publish_key() {
  if (!last_key_.empty()) {
    auto message = std_msgs::msg::String();
    message.data = last_key_;
    publisher_->publish(message);
  }
}

void KeyEvent::run_key_listener() {
  key_thread_ = std::thread([this]() {
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);

    while (running_) {
      int ch = getch();
      if (ch != ERR) { // 유효한 키 입력이 있을 경우
        last_key_ = std::string(1, static_cast<char>(ch));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    endwin(); // ncurses 종료
  });
}

} // namespace key_event

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<key_event::KeyEvent>());
  rclcpp::shutdown();
  return 0;
}
