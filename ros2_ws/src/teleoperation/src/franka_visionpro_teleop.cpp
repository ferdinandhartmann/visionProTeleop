#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "franka_msgs/msg/franka_robot_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace
{
using Vector3 = Eigen::Vector3d;
using Quaternion = Eigen::Quaterniond;

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
}

Quaternion normalized(const Quaternion & quaternion)
{
  if (!std::isfinite(quaternion.norm()) || quaternion.norm() < 1.0e-9) {
    return Quaternion::Identity();
  }
  return quaternion.normalized();
}

Quaternion poseQuaternion(const geometry_msgs::msg::Pose & pose)
{
  return normalized(Quaternion(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z));
}

geometry_msgs::msg::Quaternion quaternionMsg(const Quaternion & quaternion)
{
  const Quaternion q = normalized(quaternion);
  geometry_msgs::msg::Quaternion message;
  message.w = q.w();
  message.x = q.x();
  message.y = q.y();
  message.z = q.z();
  return message;
}

double lowpassAlpha(double cutoff_hz, double dt)
{
  if (cutoff_hz <= 0.0) {
    return 1.0;
  }
  const double tau = 1.0 / (2.0 * M_PI * cutoff_hz);
  return clamp(dt / (tau + dt), 0.0, 1.0);
}
}  // namespace

class FrankaVisionProTeleop : public rclcpp::Node
{
public:
  FrankaVisionProTeleop()
  : Node("franka_visionpro_teleop"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(this)
  {
    loadParameters();

    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      pose_command_topic_, 10);
    gripper_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      gripper_command_topic_, 10);
    enabled_publisher_ = create_publisher<std_msgs::msg::Bool>(
      enabled_topic_, rclcpp::QoS(1).reliable().transient_local());
    robot_state_subscription_ = create_subscription<franka_msgs::msg::FrankaRobotState>(
      robot_state_topic_, 10,
      std::bind(&FrankaVisionProTeleop::robotStateCallback, this, std::placeholders::_1));

    tracking_timer_ = create_wall_timer(
      rateToPeriod(tracking_rate_hz_),
      std::bind(&FrankaVisionProTeleop::trackingCallback, this));
    pose_timer_ = create_wall_timer(
      rateToPeriod(pose_publish_rate_hz_),
      std::bind(&FrankaVisionProTeleop::posePublishCallback, this));
    gripper_timer_ = create_wall_timer(
      rateToPeriod(gripper_publish_rate_hz_),
      std::bind(&FrankaVisionProTeleop::gripperPublishCallback, this));

    publishEnabled(false);
    RCLCPP_INFO(
      get_logger(),
      "Vision Pro Franka teleop ready: pose=%s (%.1f Hz), gripper=%s (%.1f Hz)",
      pose_command_topic_.c_str(), pose_publish_rate_hz_,
      gripper_command_topic_.c_str(), gripper_publish_rate_hz_);
  }

private:
  static std::chrono::nanoseconds rateToPeriod(double rate_hz)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate_hz));
  }

  void loadParameters()
  {
    robot_state_topic_ = declare_parameter<std::string>(
      "robot_state_topic", "/franka_robot_state_broadcaster/robot_state");
    pose_command_topic_ = declare_parameter<std::string>(
      "pose_command_topic", "/cartesian_impedance_controller/pose_command");
    gripper_command_topic_ = declare_parameter<std::string>(
      "gripper_command_topic", "/factr_teleop/gripper_pos_cmd");
    enabled_topic_ = declare_parameter<std::string>(
      "enabled_topic", "/teleop/teleop_enabled");
    base_frame_ = declare_parameter<std::string>("base_frame", "fr3_link0");
    hand_frame_ = declare_parameter<std::string>(
      "hand_frame", "visionpro/right/wrist");
    left_thumb_frame_ = declare_parameter<std::string>(
      "left_thumb_frame", "visionpro/left/thumb_4");
    left_index_frame_ = declare_parameter<std::string>(
      "left_index_frame", "visionpro/left/index_4");
    right_thumb_frame_ = declare_parameter<std::string>(
      "right_thumb_frame", "visionpro/right/thumb_4");
    right_index_frame_ = declare_parameter<std::string>(
      "right_index_frame", "visionpro/right/index_4");
    target_tf_frame_ = declare_parameter<std::string>(
      "target_tf_frame", "franka_visionpro_target");
    measured_ee_tf_frame_ = declare_parameter<std::string>(
      "measured_ee_tf_frame", "franka_measured_ee");

    tracking_rate_hz_ = declare_parameter<double>("tracking_rate_hz", 100.0);
    pose_publish_rate_hz_ = declare_parameter<double>("pose_publish_rate_hz", 250.0);
    gripper_publish_rate_hz_ = declare_parameter<double>("gripper_publish_rate_hz", 25.0);
    activation_distance_m_ = declare_parameter<double>("activation_distance_m", 0.02);
    activation_release_distance_m_ = declare_parameter<double>(
      "activation_release_distance_m", 0.025);
    tracking_timeout_sec_ = declare_parameter<double>("tracking_timeout_sec", 0.25);
    translation_scale_ = declare_parameter<double>("translation_scale", 0.95);
    rotation_scale_ = declare_parameter<double>("rotation_scale", 1.0);
    position_filter_cutoff_hz_ = declare_parameter<double>(
      "position_filter_cutoff_hz", 12.0);
    orientation_filter_cutoff_hz_ = declare_parameter<double>(
      "orientation_filter_cutoff_hz", 12.0);
    gripper_filter_cutoff_hz_ = declare_parameter<double>(
      "gripper_filter_cutoff_hz", 2.0);
    right_pinch_min_m_ = declare_parameter<double>("right_pinch_min_m", 0.015);
    right_pinch_max_m_ = declare_parameter<double>("right_pinch_max_m", 0.150);
    gripper_open_command_ = declare_parameter<double>("gripper_open_command", 0.0);
    gripper_closed_command_ = declare_parameter<double>("gripper_closed_command", 0.95);

    const auto axis_map = declare_parameter<std::vector<int64_t>>(
      "axis_map", {0, 1, 2});
    const auto axis_signs = declare_parameter<std::vector<double>>(
      "axis_signs", {1.0, 1.0, 1.0});
    const auto workspace_min = declare_parameter<std::vector<double>>(
      "workspace_min_xyz", {0.0, -0.5, 0.02});
    const auto workspace_max = declare_parameter<std::vector<double>>(
      "workspace_max_xyz", {0.75, 0.3, 0.70});

    if (tracking_rate_hz_ <= 0.0 || pose_publish_rate_hz_ <= 0.0 ||
      gripper_publish_rate_hz_ <= 0.0)
    {
      throw std::invalid_argument("All teleop rates must be positive");
    }
    if (activation_release_distance_m_ <= activation_distance_m_) {
      throw std::invalid_argument(
              "activation_release_distance_m must exceed activation_distance_m");
    }
    if (right_pinch_max_m_ <= right_pinch_min_m_) {
      throw std::invalid_argument("right_pinch_max_m must exceed right_pinch_min_m");
    }
    if (axis_map.size() != 3 || axis_signs.size() != 3 ||
      workspace_min.size() != 3 || workspace_max.size() != 3)
    {
      throw std::invalid_argument("Axis and workspace parameters must contain three values");
    }

    mapping_.setZero();
    std::array<bool, 3> used_axes{false, false, false};
    for (std::size_t row = 0; row < 3; ++row) {
      const int axis = static_cast<int>(axis_map[row]);
      if (axis < 0 || axis > 2 || used_axes[axis]) {
        throw std::invalid_argument("axis_map must be a permutation of [0, 1, 2]");
      }
      used_axes[axis] = true;
      mapping_(row, axis) = axis_signs[row];
      workspace_min_[row] = workspace_min[row];
      workspace_max_[row] = workspace_max[row];
      if (workspace_max_[row] <= workspace_min_[row]) {
        throw std::invalid_argument("Each workspace maximum must exceed its minimum");
      }
    }
  }

  std::optional<geometry_msgs::msg::TransformStamped> lookup(
    const std::string & child_frame)
  {
    try {
      return tf_buffer_.lookupTransform(base_frame_, child_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  static Vector3 translation(const geometry_msgs::msg::TransformStamped & transform)
  {
    return {
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z};
  }

  static Quaternion rotation(const geometry_msgs::msg::TransformStamped & transform)
  {
    const auto & q = transform.transform.rotation;
    return normalized(Quaternion(q.w, q.x, q.y, q.z));
  }

  void robotStateCallback(
    const franka_msgs::msg::FrankaRobotState::ConstSharedPtr message)
  {
    const Vector3 position{
      message->o_t_ee.pose.position.x,
      message->o_t_ee.pose.position.y,
      message->o_t_ee.pose.position.z};
    const Quaternion orientation = poseQuaternion(message->o_t_ee.pose);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_robot_position_ = position;
      current_robot_orientation_ = orientation;
      have_robot_pose_ = true;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = base_frame_;
    transform.child_frame_id = measured_ee_tf_frame_;
    transform.transform.translation.x = position.x();
    transform.transform.translation.y = position.y();
    transform.transform.translation.z = position.z();
    transform.transform.rotation = quaternionMsg(orientation);
    tf_broadcaster_.sendTransform(transform);
  }

  void trackingCallback()
  {
    const auto left_thumb = lookup(left_thumb_frame_);
    const auto left_index = lookup(left_index_frame_);
    const auto hand = lookup(hand_frame_);
    const auto right_thumb = lookup(right_thumb_frame_);
    const auto right_index = lookup(right_index_frame_);
    const rclcpp::Time stamp = now();

    std::lock_guard<std::mutex> lock(mutex_);
    if (!left_thumb || !left_index || !hand || !right_thumb || !right_index) {
      if (enabled_ && (stamp - last_tracking_stamp_).seconds() > tracking_timeout_sec_) {
        disableLocked("hand tracking timeout");
      }
      return;
    }

    last_tracking_stamp_ = stamp;
    const double left_pinch = (translation(*left_thumb) - translation(*left_index)).norm();

    if (!enabled_) {
      if (left_pinch > activation_distance_m_ || !have_robot_pose_) {
        return;
      }
      hand_anchor_position_ = translation(*hand);
      hand_anchor_orientation_ = rotation(*hand);
      robot_anchor_position_ = current_robot_position_;
      robot_anchor_orientation_ = current_robot_orientation_;
      goal_position_ = robot_anchor_position_;
      goal_orientation_ = robot_anchor_orientation_;
      filtered_position_ = goal_position_;
      filtered_orientation_ = goal_orientation_;
      filtered_gripper_ = gripper_open_command_;
      filter_initialized_ = true;
      gripper_filter_initialized_ = false;
      enabled_ = true;
      publishEnabled(true);
      RCLCPP_INFO(get_logger(), "Teleoperation enabled");
    } else if (left_pinch >= activation_release_distance_m_) {
      disableLocked("left clutch released");
      return;
    }

    const Vector3 hand_delta = translation(*hand) - hand_anchor_position_;
    goal_position_ = robot_anchor_position_ + mapping_ * hand_delta * translation_scale_;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      goal_position_[axis] = clamp(
        goal_position_[axis], workspace_min_[axis], workspace_max_[axis]);
    }

    const Quaternion raw_delta = normalized(
      hand_anchor_orientation_.conjugate() * rotation(*hand));
    Eigen::Matrix3d mapped_delta =
      mapping_ * raw_delta.toRotationMatrix() * mapping_.inverse();
    Quaternion delta_orientation(mapped_delta);
    delta_orientation = normalized(delta_orientation);
    if (rotation_scale_ != 1.0) {
      Eigen::AngleAxisd angle_axis(delta_orientation);
      delta_orientation = Quaternion(
        Eigen::AngleAxisd(angle_axis.angle() * rotation_scale_, angle_axis.axis()));
    }
    goal_orientation_ = normalized(robot_anchor_orientation_ * delta_orientation);

    const double right_pinch =
      (translation(*right_thumb) - translation(*right_index)).norm();
    const double open_fraction = clamp(
      (right_pinch - right_pinch_min_m_) /
      (right_pinch_max_m_ - right_pinch_min_m_), 0.0, 1.0);
    gripper_goal_ =
      gripper_closed_command_ +
      open_fraction * (gripper_open_command_ - gripper_closed_command_);
    have_goal_ = true;
  }

  void posePublishCallback()
  {
    geometry_msgs::msg::PoseStamped command;
    geometry_msgs::msg::TransformStamped target_transform;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!enabled_ || !have_goal_) {
        last_pose_publish_stamp_ = now();
        return;
      }

      const rclcpp::Time stamp = now();
      const double dt = last_pose_publish_stamp_.nanoseconds() == 0 ?
        1.0 / pose_publish_rate_hz_ :
        std::max(1.0e-4, (stamp - last_pose_publish_stamp_).seconds());
      last_pose_publish_stamp_ = stamp;

      if (!filter_initialized_) {
        filtered_position_ = goal_position_;
        filtered_orientation_ = goal_orientation_;
        filter_initialized_ = true;
      } else {
        filtered_position_ +=
          lowpassAlpha(position_filter_cutoff_hz_, dt) *
          (goal_position_ - filtered_position_);
        Quaternion orientation_target = goal_orientation_;
        if (filtered_orientation_.dot(orientation_target) < 0.0) {
          orientation_target.coeffs() *= -1.0;
        }
        filtered_orientation_ = normalized(filtered_orientation_.slerp(
          lowpassAlpha(orientation_filter_cutoff_hz_, dt), orientation_target));
      }

      command.header.stamp = stamp;
      command.header.frame_id = base_frame_;
      command.pose.position.x = filtered_position_.x();
      command.pose.position.y = filtered_position_.y();
      command.pose.position.z = filtered_position_.z();
      command.pose.orientation = quaternionMsg(filtered_orientation_);

      target_transform.header = command.header;
      target_transform.child_frame_id = target_tf_frame_;
      target_transform.transform.translation.x = filtered_position_.x();
      target_transform.transform.translation.y = filtered_position_.y();
      target_transform.transform.translation.z = filtered_position_.z();
      target_transform.transform.rotation = command.pose.orientation;
    }
    pose_publisher_->publish(command);
    tf_broadcaster_.sendTransform(target_transform);
  }

  void gripperPublishCallback()
  {
    sensor_msgs::msg::JointState command;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!enabled_ || !have_goal_) {
        last_gripper_publish_stamp_ = now();
        return;
      }
      const rclcpp::Time stamp = now();
      const double dt = last_gripper_publish_stamp_.nanoseconds() == 0 ?
        1.0 / gripper_publish_rate_hz_ :
        std::max(1.0e-4, (stamp - last_gripper_publish_stamp_).seconds());
      last_gripper_publish_stamp_ = stamp;
      if (!gripper_filter_initialized_) {
        filtered_gripper_ = gripper_goal_;
        gripper_filter_initialized_ = true;
      } else {
        filtered_gripper_ += lowpassAlpha(gripper_filter_cutoff_hz_, dt) *
          (gripper_goal_ - filtered_gripper_);
      }
      filtered_gripper_ = clamp(
        filtered_gripper_, std::min(gripper_open_command_, gripper_closed_command_),
        std::max(gripper_open_command_, gripper_closed_command_));
      command.header.stamp = stamp;
      command.name = {"fr3_finger_joint"};
      command.position = {filtered_gripper_};
    }
    gripper_publisher_->publish(command);
  }

  void disableLocked(const char * reason)
  {
    if (!enabled_) {
      return;
    }
    enabled_ = false;
    have_goal_ = false;
    filter_initialized_ = false;
    gripper_filter_initialized_ = false;
    publishEnabled(false);
    RCLCPP_INFO(get_logger(), "Teleoperation disabled: %s", reason);
  }

  void publishEnabled(bool enabled)
  {
    std_msgs::msg::Bool message;
    message.data = enabled;
    enabled_publisher_->publish(message);
  }

  std::mutex mutex_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gripper_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_publisher_;
  rclcpp::Subscription<franka_msgs::msg::FrankaRobotState>::SharedPtr
    robot_state_subscription_;
  rclcpp::TimerBase::SharedPtr tracking_timer_;
  rclcpp::TimerBase::SharedPtr pose_timer_;
  rclcpp::TimerBase::SharedPtr gripper_timer_;

  std::string robot_state_topic_;
  std::string pose_command_topic_;
  std::string gripper_command_topic_;
  std::string enabled_topic_;
  std::string base_frame_;
  std::string hand_frame_;
  std::string left_thumb_frame_;
  std::string left_index_frame_;
  std::string right_thumb_frame_;
  std::string right_index_frame_;
  std::string target_tf_frame_;
  std::string measured_ee_tf_frame_;

  double tracking_rate_hz_{100.0};
  double pose_publish_rate_hz_{250.0};
  double gripper_publish_rate_hz_{25.0};
  double activation_distance_m_{0.02};
  double activation_release_distance_m_{0.025};
  double tracking_timeout_sec_{0.25};
  double translation_scale_{0.95};
  double rotation_scale_{1.0};
  double position_filter_cutoff_hz_{12.0};
  double orientation_filter_cutoff_hz_{12.0};
  double gripper_filter_cutoff_hz_{2.0};
  double right_pinch_min_m_{0.015};
  double right_pinch_max_m_{0.150};
  double gripper_open_command_{0.0};
  double gripper_closed_command_{0.95};
  Eigen::Matrix3d mapping_{Eigen::Matrix3d::Identity()};
  Vector3 workspace_min_{0.0, -0.5, 0.02};
  Vector3 workspace_max_{0.75, 0.3, 0.70};

  bool enabled_{false};
  bool have_robot_pose_{false};
  bool have_goal_{false};
  bool filter_initialized_{false};
  bool gripper_filter_initialized_{false};
  Vector3 current_robot_position_{Vector3::Zero()};
  Quaternion current_robot_orientation_{Quaternion::Identity()};
  Vector3 hand_anchor_position_{Vector3::Zero()};
  Quaternion hand_anchor_orientation_{Quaternion::Identity()};
  Vector3 robot_anchor_position_{Vector3::Zero()};
  Quaternion robot_anchor_orientation_{Quaternion::Identity()};
  Vector3 goal_position_{Vector3::Zero()};
  Quaternion goal_orientation_{Quaternion::Identity()};
  Vector3 filtered_position_{Vector3::Zero()};
  Quaternion filtered_orientation_{Quaternion::Identity()};
  double gripper_goal_{0.0};
  double filtered_gripper_{0.0};
  rclcpp::Time last_tracking_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pose_publish_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_gripper_publish_stamp_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrankaVisionProTeleop>());
  rclcpp::shutdown();
  return 0;
}
