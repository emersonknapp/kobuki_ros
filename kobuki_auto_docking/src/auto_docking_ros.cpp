#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "kdl/frames.hpp"
#include "kobuki_core/dock_drive.hpp"
#include "kobuki_ros_interfaces/msg/dock_infra_red.hpp"
#include "kobuki_ros_interfaces/msg/sensor_state.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_kdl/tf2_kdl.h"

namespace kobuki
{

typedef message_filters::sync_policies::ApproximateTime<
  nav_msgs::msg::Odometry,
  // kobuki_ros_interfaces::msg::SensorState,
  kobuki_ros_interfaces::msg::DockInfraRed
> SyncPolicy;

class AutoDockingROS : public rclcpp::Node
{
public:
  explicit
  AutoDockingROS(const rclcpp::NodeOptions & options)
    : rclcpp::Node("auto_docking_ros", options)
  {
    RCLCPP_WARN(get_logger(), "Started autodock node");
    double min_abs_v = 0.01;
    double min_abs_w = 0.1;
    // if (nh.getParam("min_abs_v", min_abs_v) == true)
    dock_.setMinAbsV(min_abs_v);
    // if (nh.getParam("min_abs_w", min_abs_w) == true)
    dock_.setMinAbsW(min_abs_w);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/simulation/cmd_vel", 10);

    odom_sub_ = std::make_unique<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
      this, "/simulation/odom", rmw_qos_profile_sensor_data);
    ir_sub_ = std::make_unique<
      message_filters::Subscriber<kobuki_ros_interfaces::msg::DockInfraRed>>(this, "/dock_ir", rmw_qos_profile_sensor_data);
    core_sub_ = std::make_unique<
      message_filters::Subscriber<kobuki_ros_interfaces::msg::SensorState>>(this, "core");

    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10),
      *odom_sub_,
      // *core_sub_,
      *ir_sub_
    );

    sync_->registerCallback(std::bind(
      &AutoDockingROS::sync_callback, this
      , std::placeholders::_1
      , std::placeholders::_2
      // , std::placeholders::_3
    ));
    if (!dock_.init()) {
      RCLCPP_ERROR(get_logger(), "Fuck");
    }
    dock_.enable();
  }


  virtual ~AutoDockingROS()
  {
  }

  void sync_callback(
    std::shared_ptr<nav_msgs::msg::Odometry const> odom,
    // std::shared_ptr<kobuki_ros_interfaces::msg::SensorState const> core,
    std::shared_ptr<kobuki_ros_interfaces::msg::DockInfraRed const> ir)
  {
    RCLCPP_INFO(get_logger(), "Get MESAGE");
    if (!dock_.isEnabled()) {
      return;
    }

    //conversions
    KDL::Frame frame;
    tf2::fromMsg(odom->pose.pose, frame);
    const KDL::Rotation & rot = frame.M;

    double r, p, y;
    rot.GetRPY(r, p, y);

    ecl::linear_algebra::Vector3d pose;  // x, y, heading
    pose[0] = odom->pose.pose.position.x;
    pose[1] = odom->pose.pose.position.y;
    pose[2] = y;

    //update
    dock_.update(ir->data, 0, 0, pose);
    // dock_.update(ir->data, core->bumper, core->charger, pose);
    RCLCPP_INFO(get_logger(), dock_.getDebugStream());

    //publish command velocity
    if (dock_.canRun()) {
      auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
      cmd_vel->linear.x = dock_.getVX();
      cmd_vel->angular.z = dock_.getWZ();
      cmd_vel_pub_->publish(std::move(cmd_vel));
    }

    if (dock_.getState() == RobotDockingState::DONE) {
      RCLCPP_INFO(get_logger(), "Arrived on docking station successfully.");
      dock_.disable();
    }

  }

private:


  DockDrive dock_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::unique_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
  std::unique_ptr<message_filters::Subscriber<kobuki_ros_interfaces::msg::DockInfraRed>> ir_sub_;
  std::unique_ptr<message_filters::Subscriber<kobuki_ros_interfaces::msg::SensorState>> core_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

} //namespace kobuki

RCLCPP_COMPONENTS_REGISTER_NODE(kobuki::AutoDockingROS)
