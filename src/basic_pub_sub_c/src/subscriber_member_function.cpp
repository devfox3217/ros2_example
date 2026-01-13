#include <memory> // 스마트 포인터 사용을 위한 헤더

#include "rclcpp/rclcpp.hpp" // ROS 2 C++ 클라이언트 라이브러리
#include "std_msgs/msg/string.hpp" // 문자열 메시지 타입 헤더

// std::bind에서 매개변수 자리를 표시하기 위해 사용 (_1, _2 ...)
using std::placeholders::_1;

// rclcpp::Node를 상속받아 MinimalSubscriber 클래스를 정의합니다.
class MinimalSubscriber : public rclcpp::Node
{
  public:
    // 생성자
    MinimalSubscriber()
    : Node("minimal_subscriber") // 노드 이름을 'minimal_subscriber'로 초기화
    {
      // 서브스크라이버(Subscriber) 생성
      // 템플릿 타입: <std_msgs::msg::String> (수신할 메시지 타입)
      // 매개변수 1: "topic" (구독할 토픽 이름)
      // 매개변수 2: 10 (큐 사이즈 - 메시지 버퍼 크기)
      // 매개변수 3: 콜백 함수 등록
      // std::bind를 사용하여 멤버 함수(topic_callback)를 콜백으로 등록합니다.
      // this: 현재 객체의 포인터
      // _1: 콜백 함수가 받을 첫 번째 인자(수신된 메시지)를 의미
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    // 메시지를 수신했을 때 호출되는 콜백 함수
    // msg: 수신된 메시지를 가리키는 스마트 포인터 (SharedPtr)
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      // 수신된 메시지 내용을 터미널에 로그로 출력합니다.
      // msg->data.c_str(): std::string을 C 스타일 문자열(const char*)로 변환
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    // 서브스크라이버 객체를 저장할 멤버 변수
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  // rclcpp 라이브러리 초기화
  rclcpp::init(argc, argv);

  // MinimalSubscriber 노드 객체를 생성하고 Spin(대기) 상태로 만듭니다.
  // std::make_shared를 사용하여 노드를 스마트 포인터로 생성합니다.
  rclcpp::spin(std::make_shared<MinimalSubscriber>());

  // rclcpp 라이브러리 종료 (Ctrl+C 등으로 종료 시 실행됨)
  rclcpp::shutdown();
  return 0;
}
