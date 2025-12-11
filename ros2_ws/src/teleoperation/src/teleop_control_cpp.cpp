#include <cmath>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <Eigen/Dense>

namespace
{
geometry_msgs::msg::Quaternion eigenToMsg(const Eigen::Quaterniond & q)
{
  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

Eigen::Matrix4d transformToMatrix(const geometry_msgs::msg::TransformStamped & tf)
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 3) = tf.transform.translation.x;
  T(1, 3) = tf.transform.translation.y;
  T(2, 3) = tf.transform.translation.z;

  tf2::Quaternion q;
  tf2::fromMsg(tf.transform.rotation, q);
  Eigen::Quaterniond eigen_q(q.w(), q.x(), q.y(), q.z());
  T.block<3, 3>(0, 0) = eigen_q.normalized().toRotationMatrix();
  return T;
}

Eigen::Quaterniond multiplyQuat(const Eigen::Quaterniond & q1, const Eigen::Quaterniond & q2)
{
  Eigen::Quaterniond q = q1 * q2;
  q.normalize();
  return q;
}

Eigen::Quaterniond quaternionFromEulerZ(double degrees)
{
  const double radians = degrees * M_PI / 180.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, radians);
  Eigen::Quaterniond eigen_q(q.w(), q.x(), q.y(), q.z());
  eigen_q.normalize();
  return eigen_q;
}
}  // namespace

class TeleopControl : public rclcpp::Node
{
public:
  TeleopControl()
  : Node("teleop_control")
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    ee_target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/teleop/ee_target", 10);

    this->declare_parameter<std::string>("vp_base_frame", "vp_base");
    this->declare_parameter<std::string>("gripper_base_frame", "gripper_ee");
    this->declare_parameter<double>("update_period", 0.1);
    this->declare_parameter<double>("pinch_threshold", 0.02);
    this->declare_parameter<double>("right_pinch_min", 0.015);
    this->declare_parameter<double>("right_pinch_max", 0.150);

    vp_base_frame_ = this->get_parameter("vp_base_frame").as_string();
    gripper_base_frame_ = this->get_parameter("gripper_base_frame").as_string();
    update_period_ = this->get_parameter("update_period").as_double();
    pinch_threshold_ = this->get_parameter("pinch_threshold").as_double();
    right_pinch_min_ = this->get_parameter("right_pinch_min").as_double();
    right_pinch_max_ = this->get_parameter("right_pinch_max").as_double();

    publishVpBaseCalibration(Eigen::Matrix4d::Identity());

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(update_period_),
      std::bind(&TeleopControl::update, this));

    RCLCPP_INFO(this->get_logger(), "Teleop Control node initialized.");
  }

private:
  std::optional<Eigen::Vector3d> getPos(const std::string & child_frame)
  {
    try {
      auto tf = tf_buffer_->lookupTransform(vp_base_frame_, child_frame, tf2::TimePointZero);
      return Eigen::Vector3d(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  std::pair<std::optional<Eigen::Vector3d>, std::optional<Eigen::Quaterniond>> getPose(
    const std::string & child_frame)
  {
    try {
      auto tf = tf_buffer_->lookupTransform(vp_base_frame_, child_frame, tf2::TimePointZero);
      Eigen::Vector3d pos(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
      tf2::Quaternion q;
      tf2::fromMsg(tf.transform.rotation, q);
      Eigen::Quaterniond ori(q.w(), q.x(), q.y(), q.z());
      ori.normalize();
      return {pos, ori};
    } catch (const tf2::TransformException &) {
      return {std::nullopt, std::nullopt};
    }
  }

  double distance(const Eigen::Vector3d & a, const Eigen::Vector3d & b) const
  {
    return (a - b).norm();
  }

  double clamp(double x, double lo, double hi) const
  {
    return std::max(lo, std::min(hi, x));
  }

  void publishVpBaseCalibration(const Eigen::Matrix4d & T)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = "vp_base_origin";
    tf_msg.child_frame_id = "vp_base";

    tf_msg.transform.translation.x = T(0, 3);
    tf_msg.transform.translation.y = T(1, 3);
    tf_msg.transform.translation.z = T(2, 3);

    Eigen::Quaterniond q(T.block<3, 3>(0, 0));
    q.normalize();
    tf_msg.transform.rotation = eigenToMsg(q);

    static_tf_broadcaster_->sendTransform(tf_msg);
    RCLCPP_INFO(this->get_logger(), "Published vp_base calibration transform");
  }

  void publishEeTargetTf(
    const Eigen::Vector3d & ee_pos, const Eigen::Quaterniond & ee_ori,
    const std::string & child_frame, const std::string & header_frame)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = header_frame;
    t.child_frame_id = child_frame;
    t.transform.translation.x = ee_pos.x();
    t.transform.translation.y = ee_pos.y();
    t.transform.translation.z = ee_pos.z();
    t.transform.rotation = eigenToMsg(ee_ori);

    tf_broadcaster_->sendTransform(t);
  }

  void update()
  {
    auto left_thumb = getPos("visionpro/left/thumb_4");
    auto left_index = getPos("visionpro/left/index_4");

    if (!left_thumb || !left_index) {
      teleop_enabled_ = false;
      return;
    }

    const double left_pinch = distance(*left_thumb, *left_index);

    bool just_enabled = false;
    bool just_disabled = false;

    if (!teleop_enabled_ && left_pinch < pinch_threshold_) {
      teleop_enabled_ = true;
      just_enabled = true;
      RCLCPP_INFO(this->get_logger(), "Teleop ENABLED (left pinch)");
    } else if (teleop_enabled_ && left_pinch > pinch_threshold_) {
      teleop_enabled_ = false;
      just_disabled = true;
      publishVpBaseCalibration(Eigen::Matrix4d::Identity());
      RCLCPP_INFO(this->get_logger(), "Teleop DISABLED (left pinch)");
    }

    if (!teleop_enabled_) {
      return;
    }

    auto right_thumb = getPos("visionpro/right/thumb_4");
    auto right_index = getPos("visionpro/right/index_4");
    auto wrist_pose = getPose("visionpro/right/wrist");

    if (!right_thumb || !right_index || !wrist_pose.first || !wrist_pose.second) {
      return;
    }

    Eigen::Vector3d ee_pos = (*right_thumb + *right_index) / 2.0;
    Eigen::Quaterniond wrist_ori = *wrist_pose.second;

    Eigen::Quaterniond q_flip = quaternionFromEulerZ(220.0);
    Eigen::Quaterniond ee_ori = multiplyQuat(wrist_ori, q_flip);

    publishEeTargetTf(ee_pos, ee_ori, "ee_target", vp_base_frame_);

    const double right_pinch = distance(*right_thumb, *right_index);
    const double d_clamped = clamp(right_pinch, right_pinch_min_, right_pinch_max_);
    (void)d_clamped;  // Currently unused; placeholder for gripper logic.

    geometry_msgs::msg::TransformStamped tf_h;
    try {
      tf_h = tf_buffer_->lookupTransform("map", "ee_target", tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;
    }
    Eigen::Matrix4d T_hand_map = transformToMatrix(tf_h);

    if (just_enabled) {
      try {
        auto tf_g = tf_buffer_->lookupTransform("map", gripper_base_frame_, tf2::TimePointZero);
        Eigen::Matrix4d T_gripper = transformToMatrix(tf_g);

        Eigen::Matrix4d T_hand_map_inv = T_hand_map.inverse();
        offset_T_ = T_hand_map_inv * T_gripper;

        offset_pos_ = T_gripper.block<3, 1>(0, 3) - T_hand_map.block<3, 1>(0, 3);
        Eigen::Quaterniond R_hand(T_hand_map.block<3, 3>(0, 0));
        Eigen::Quaterniond R_grip(T_gripper.block<3, 3>(0, 0));
        offset_rot_ = R_hand.conjugate() * R_grip;
        offset_rot_.normalize();

        offset_available_ = true;
        RCLCPP_INFO(this->get_logger(), "Teleop offset captured (gripper frame: %s)", gripper_base_frame_.c_str());
      } catch (const tf2::TransformException & ex) {
        offset_available_ = false;
        RCLCPP_WARN(this->get_logger(), "Offset capture failed for frame '%s': %s", gripper_base_frame_.c_str(), ex.what());
      }
    }

    if (!last_5hz_time_.nanoseconds()) {
      last_5hz_time_ = this->get_clock()->now();
    }

    if (teleop_enabled_ && offset_available_) {
      Eigen::Vector3d T_target_pos = T_hand_map.block<3, 1>(0, 3) + offset_pos_;
      Eigen::Quaterniond T_target_rot(T_hand_map.block<3, 3>(0, 0));
      T_target_rot = T_target_rot * offset_rot_;
      T_target_rot.normalize();

      Eigen::Matrix4d T_target = Eigen::Matrix4d::Identity();
      T_target.block<3, 1>(0, 3) = T_target_pos;
      T_target.block<3, 3>(0, 0) = T_target_rot.toRotationMatrix();

      Eigen::Vector3d ee_pos_offset = T_target.block<3, 1>(0, 3);
      Eigen::Quaterniond ee_ori_offset(T_target.block<3, 3>(0, 0));
      ee_ori_offset.normalize();

      try {
        auto tf_mycobot_base = tf_buffer_->lookupTransform("mycobot_base", "map", tf2::TimePointZero);
        Eigen::Matrix4d T_map_to_mycobot_base = transformToMatrix(tf_mycobot_base);

        Eigen::Matrix4d T_ee_target_offset_map = Eigen::Matrix4d::Identity();
        T_ee_target_offset_map.block<3, 3>(0, 0) = ee_ori_offset.toRotationMatrix();
        T_ee_target_offset_map.block<3, 1>(0, 3) = ee_pos_offset;

        Eigen::Matrix4d T_ee_target_offset_mycobot_base = T_map_to_mycobot_base * T_ee_target_offset_map;
        Eigen::Vector3d ee_pos_mycobot_base = T_ee_target_offset_mycobot_base.block<3, 1>(0, 3);
        Eigen::Quaterniond ee_ori_mycobot_base(T_ee_target_offset_mycobot_base.block<3, 3>(0, 0));
        ee_ori_mycobot_base.normalize();

        publishEeTargetTf(
          ee_pos_mycobot_base, ee_ori_mycobot_base,
          "ee_target_offset_mycobot_base", "mycobot_base");

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->get_clock()->now();
        pose_msg.header.frame_id = "mycobot_base";
        pose_msg.pose.position.x = ee_pos_mycobot_base.x();
        pose_msg.pose.position.y = ee_pos_mycobot_base.y();
        pose_msg.pose.position.z = ee_pos_mycobot_base.z();
        pose_msg.pose.orientation = eigenToMsg(ee_ori_mycobot_base);
        ee_target_pub_->publish(pose_msg);

        RCLCPP_INFO(this->get_logger(), "Published ee_target_offset in mycobot_base frame");
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Failed to transform ee_target_offset to mycobot_base: %s", ex.what());
      }

      if (just_disabled) {
        offset_available_ = false;
        RCLCPP_INFO(this->get_logger(), "Teleop offset cleared");
      }

      last_5hz_time_ = this->get_clock()->now();
    }
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string vp_base_frame_;
  std::string gripper_base_frame_;
  double update_period_{};
  double pinch_threshold_{};
  double right_pinch_min_{};
  double right_pinch_max_{};

  bool teleop_enabled_{false};
  bool offset_available_{false};
  Eigen::Matrix4d offset_T_ = Eigen::Matrix4d::Identity();
  Eigen::Vector3d offset_pos_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond offset_rot_{0.0, 0.0, 0.0, 0.0};
  rclcpp::Time last_5hz_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleopControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
