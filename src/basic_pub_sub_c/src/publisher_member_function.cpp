#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>  // C언어 표준 입출력 (printf, snprintf 등)

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// ==========================================
// [순수 C언어 영역]
// 클래스와 상관없는 일반 C 함수입니다.
// ==========================================
int c_style_add(int a, int b) {
    return a + b;
}

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();

      // ---------------------------------------------------------
      // [C언어 스타일 구현]
      // std::string 대신 char 배열과 snprintf 사용
      // ---------------------------------------------------------
      char buffer[100]; // C언어 문자열 버퍼

      // C 함수 호출 (현재 카운트에 1을 더함)
      int next_count = c_style_add(count_, 1);

      // C언어 문자열 포맷팅
      // 수정: count_ 대신 next_count를 사용하여 변수를 활용함
      snprintf(buffer, sizeof(buffer), "Hello World (C Style): %d", next_count);

      // C++ string에 C 문자열 대입
      message.data = buffer;

      // ROS 2 로깅 (C++ 스타일)
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

      // C언어 표준 출력 (printf)도 사용 가능합니다.
      printf("[C-printf] Sent message number: %d\n", next_count);

      publisher_->publish(message);
      count_++;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    int count_; // size_t 대신 int 사용 (C 스타일)
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
