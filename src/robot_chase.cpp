#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/timer.hpp"
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

class RobotChaseNode : public rclcpp::Node {
public:
  RobotChaseNode();

private:
  static constexpr char kNodeName[]{"robot_chase"};
  static constexpr char kSourceFrame[]{"rick_base_link"};
  static constexpr char kDestinationFrame[]{"morty_base_link"};
  static constexpr char kCmdTopicName[]{"rick/cmd_vel"};
  static constexpr double kKpYaw{1.0};
  static constexpr double kKpDdistance{0.5};
  static constexpr std::chrono::milliseconds kTimerPeriod{100ms};

  void timer_cb();

  std::shared_ptr<tf2_ros::Buffer> buffer_{};
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{};
  rclcpp::TimerBase::SharedPtr timer_{};
};

RobotChaseNode::RobotChaseNode()
    : Node{kNodeName}, buffer_{std::make_unique<tf2_ros::Buffer>(
                           this->get_clock())},
      transform_listener_{
          std::make_shared<tf2_ros::TransformListener>(*this->buffer_)},
      publisher_{
          this->create_publisher<geometry_msgs::msg::Twist>(kCmdTopicName, 1)},
      timer_{this->create_wall_timer(kTimerPeriod,
                                     [this] { return timer_cb(); })} {}

void RobotChaseNode::timer_cb() {
  geometry_msgs::msg::TransformStamped t{};
  try {
    t = buffer_->lookupTransform(kSourceFrame, kDestinationFrame,
                                 tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                kSourceFrame, kDestinationFrame, ex.what());
    return;
  }
  const auto error_distance{std::sqrt(std::pow(t.transform.translation.x, 2) +
                                      std::pow(t.transform.translation.y, 2))};
  const auto error_yaw{
      std::atan2(t.transform.translation.y, t.transform.translation.x)};

  geometry_msgs::msg::Twist msg{};

  msg.angular.z = kKpYaw * error_yaw;
  msg.linear.x = kKpDdistance * error_distance;

  publisher_->publish(msg);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<RobotChaseNode>());

  rclcpp::shutdown();
}