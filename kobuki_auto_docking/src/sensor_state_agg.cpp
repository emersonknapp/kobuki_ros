#include <string>

#include "gazebo_msgs/msg/contacts_state.hpp"
#include "kobuki_ros_interfaces/msg/sensor_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace kobuki
{

class SensorStateAggregator : public rclcpp::Node
{
private:
  using SensorState = kobuki_ros_interfaces::msg::SensorState;

  tf2::Vector3 heading_{1, 0, 0};
  double front_contact_threshold_ = 35.0;
  rclcpp::Publisher<SensorState>::SharedPtr state_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr bumper_sub_;
  SensorState msg_;
  rclcpp::TimerBase::SharedPtr pub_timer_;

public:
  explicit
  SensorStateAggregator(const rclcpp::NodeOptions & options)
    : rclcpp::Node("sensor_states", options)
  {
    using namespace std::placeholders;

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&SensorStateAggregator::odom_cb, this, _1));
    bumper_sub_ = create_subscription<gazebo_msgs::msg::ContactsState>(
      "/bumper_states",
      10,
      std::bind(&SensorStateAggregator::bumper_cb, this, _1));
    state_pub_ = create_publisher<SensorState>(
      "core", 10);
    pub_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&SensorStateAggregator::publish_state, this));
  }

private:
  static tf2::Vector3 fromMsg(const geometry_msgs::msg::Vector3 & v)
  {
    return tf2::Vector3(v.x, v.y, v.z);
  }

  void publish_state()
  {
    state_pub_->publish(msg_);
  }

  void odom_cb(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
  {
    // Convert the orientation from odom into a tf type
    const auto & q = msg->pose.pose.orientation;
    tf2::Quaternion r{q.x, q.y, q.z, q.w};

    // Calculate the heading by rotating a X direction vector by the given quaternion
    const tf2::Vector3 d{1, 0, 0};
    heading_ = tf2::quatRotate(r, d);
  }

  void bumper_cb(const std::shared_ptr<gazebo_msgs::msg::ContactsState> contacts)
  {
    const tf2::Vector3 planeNormal{0, 0, 1};
    msg_.bumper = 0;
    for (const auto & state : contacts->states) {
      // RCLCPP_INFO(get_logger(), "  Contact %zu", state.contact_normals.size());
      for (const auto & normal : state.contact_normals) {
        auto n = fromMsg(normal);
        // Contact points are on a cylindrical collision body.
        // Therefore, the contact normals are all pointed inwards to the robot
        // Invert the normal (point outwards from origin)
        n *= -1;

        // Check if the contact comes from the front of the robot
        auto dot = heading_.dot(n);
        if (dot >= 0.0) {
          auto angle = heading_.angle(n);
          if (planeNormal.dot(heading_.cross(n)) < 0) {
            angle = -angle;
          }
          angle = tf2Degrees(angle);
          if (fabs(angle) <= front_contact_threshold_) {
            msg_.bumper |= SensorState::BUMPER_CENTRE;
          } else if (angle > 0.0) {
            // left
            msg_.bumper |= SensorState::BUMPER_LEFT;
          } else {
            // right
            msg_.bumper |= SensorState::BUMPER_RIGHT;
          }
        }
      }
    }
  }
};

} //namespace kobuki

RCLCPP_COMPONENTS_REGISTER_NODE(kobuki::SensorStateAggregator)
