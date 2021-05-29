#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "kdl/frames.hpp"
#include "kobuki_core/dock_drive.hpp"
#include "kobuki_ros_interfaces/action/auto_docking.hpp"
#include "kobuki_ros_interfaces/msg/dock_infra_red.hpp"
#include "kobuki_ros_interfaces/msg/sensor_state.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
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
    using namespace std::placeholders;
    RCLCPP_DEBUG(get_logger(), "Started autodock node");

    declare_parameter<double>("min_abs_v", 0.01);
    declare_parameter<double>("min_abs_w", 0.1);

    {
      double min_abs_v;
      double min_abs_w;
      get_parameter("min_abs_v", min_abs_v);
      get_parameter("min_abs_w", min_abs_w);
      dock_.setMinAbsV(min_abs_v);
      dock_.setMinAbsW(min_abs_w);
    }

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);

    odom_sub_ = std::make_unique<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
      this, "/odom", rmw_qos_profile_sensor_data);
    ir_sub_ = std::make_unique<
      message_filters::Subscriber<kobuki_ros_interfaces::msg::DockInfraRed>>(
        this, "/dock_ir", rmw_qos_profile_sensor_data);
    core_sub_ = std::make_unique<
      message_filters::Subscriber<kobuki_ros_interfaces::msg::SensorState>>(
        this, "core");

    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10),
      *odom_sub_,
      // *core_sub_,
      *ir_sub_
    );

    sync_->registerCallback(std::bind(&AutoDockingROS::sync_callback, this, _1, _2));
    // , _3


    this->action_server_ = rclcpp_action::create_server<
      kobuki_ros_interfaces::action::AutoDocking>(
        this,
        "auto_dock",
        std::bind(&AutoDockingROS::handle_goal, this, _1, _2),
        std::bind(&AutoDockingROS::handle_cancel, this, _1),
        std::bind(&AutoDockingROS::handle_accepted, this, _1));

    if (!dock_.init()) {
      RCLCPP_ERROR(get_logger(), "Unable to initialize docking algorithm.");
    }
  }


  virtual ~AutoDockingROS()
  {
  }

  void sync_callback(
    std::shared_ptr<nav_msgs::msg::Odometry const> odom,
    // std::shared_ptr<kobuki_ros_interfaces::msg::SensorState const> core,
    std::shared_ptr<kobuki_ros_interfaces::msg::DockInfraRed const> ir)
  {
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

  using AutoDocking = kobuki_ros_interfaces::action::AutoDocking;
  using GoalHandleAutoDocking = rclcpp_action::ServerGoalHandle<AutoDocking>;

private:

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const AutoDocking::Goal> goal)
  {
    (void)uuid;
    (void)goal;

    if (dock_.isEnabled()) {
      RCLCPP_INFO(this->get_logger(), "Rejected: dock drive is already enabled");
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "Received New goal received and accepted.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleAutoDocking> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    dock_.disable();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleAutoDocking> goal_handle)
  {
    (void)goal_handle;
    dock_.enable();
  }

  DockDrive dock_;

  rclcpp_action::Server<kobuki_ros_interfaces::action::AutoDocking>::SharedPtr action_server_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::unique_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
  std::unique_ptr<message_filters::Subscriber<kobuki_ros_interfaces::msg::DockInfraRed>> ir_sub_;
  std::unique_ptr<message_filters::Subscriber<kobuki_ros_interfaces::msg::SensorState>> core_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

} //namespace kobuki

RCLCPP_COMPONENTS_REGISTER_NODE(kobuki::AutoDockingROS)
