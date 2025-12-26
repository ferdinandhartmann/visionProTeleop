#include <cmath>
#include <optional>
#include <string>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "teleoperation/msg/teleop_target.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <Eigen/Dense>
#include "std_msgs/msg/bool.hpp"

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
// Flip rotation about the X and Y axes (roll and pitch) while keeping yaw.
Eigen::Quaterniond flipPitchAndRoll(const Eigen::Quaterniond & q)
{
  tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  roll = -roll;
  pitch = -pitch;
  tf2::Quaternion out;
  out.setRPY(roll, pitch, yaw);
  Eigen::Quaterniond eigen_out(out.w(), out.x(), out.y(), out.z());
  eigen_out.normalize();
  return eigen_out;
}
}  // namespace

class TeleopControl : public rclcpp::Node
{
public:
  TeleopControl(): Node("teleop_control"){
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    ee_target_pub_ = this->create_publisher<teleoperation::msg::TeleopTarget>("/teleop/ee_target", 10);

    this->declare_parameter<double>("update_period", 0.2);
    this->declare_parameter<double>("update_period_vis", 0.001);
    this->declare_parameter<double>("pinch_threshold", 0.02);
    this->declare_parameter<double>("right_pinch_min", 0.015);
    this->declare_parameter<double>("right_pinch_max", 0.150);
    this->declare_parameter<double>("smoothing_factor", 3.0);
    this->declare_parameter<int>("smoothing_window", 3);
    this->declare_parameter<double>("translation_scale", 1.0);
    this->declare_parameter<double>("rotation_scale", 1.0);

    // Publisher for teleop trigger (enabled/disabled)
    teleop_trigger_pub_ = this->create_publisher<std_msgs::msg::Bool>("/teleop/teleop_enabled", 10);

    update_period_ = this->get_parameter("update_period").as_double();
		update_period_vis_ = this->get_parameter("update_period_vis").as_double();
    pinch_threshold_ = this->get_parameter("pinch_threshold").as_double();
    right_pinch_min_ = this->get_parameter("right_pinch_min").as_double();
    right_pinch_max_ = this->get_parameter("right_pinch_max").as_double();

    smoothing_factor_ = this->get_parameter("smoothing_factor").as_double();
    if (smoothing_factor_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "smoothing_factor must be > 0. Using 3.0.");
      smoothing_factor_ = 3.0;
    }
    int smoothing_window_param = this->get_parameter("smoothing_window").as_int();
    if (smoothing_window_param <= 0) {
      RCLCPP_WARN(this->get_logger(), "smoothing_window must be > 0. Using 3.");
      smoothing_window_param = 3;
    }
    smoothing_window_ = static_cast<std::size_t>(smoothing_window_param);

    translation_scale_ = this->get_parameter("translation_scale").as_double();
    if (translation_scale_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "translation_scale must be > 0. Using 1.0.");
      translation_scale_ = 1.0;
    }

    rotation_scale_ = this->get_parameter("rotation_scale").as_double();
    if (rotation_scale_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "rotation_scale must be > 0. Using 1.0.");
      rotation_scale_ = 1.0;
    }

    //publishVpBaseCalibration(Eigen::Matrix4d::Identity());
    // rot matrix -90 axis Z
    Eigen::Matrix4d calibration_T = createZRotationMatrix(-90.0); 
    calibration_T(0, 3) = 0.0; // Traslación en X
    calibration_T(1, 3) = 0.0; // Traslación en Y
    calibration_T(2, 3) = 0.0; // Traslación en Z
    publishVpBaseCalibration(calibration_T);
    const double sample_period = update_period_ / smoothing_factor_;
    auto sample_duration = std::chrono::duration<double>(sample_period);

    sampling_timer_ = this->create_wall_timer(
      sample_duration,
      std::bind(&TeleopControl::sample, this));

    publish_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(update_period_),
      std::bind(&TeleopControl::publishSmoothed, this));

		publish_timer_vis_ = this->create_wall_timer(
      std::chrono::duration<double>(update_period_vis_),
      std::bind(&TeleopControl::publishSmoothedVis, this));

    RCLCPP_INFO(this->get_logger(), "Teleop Control node initialized.");
  }

private:
  std::optional<Eigen::Vector3d> getPos(const std::string & child_frame)
  {
    try {
      auto tf = tf_buffer_->lookupTransform("vp_base", child_frame, tf2::TimePointZero);
      return Eigen::Vector3d(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  std::pair<std::optional<Eigen::Vector3d>, std::optional<Eigen::Quaterniond>> getPose(
    const std::string & child_frame)
  {
    try {
      auto tf = tf_buffer_->lookupTransform("vp_base", child_frame, tf2::TimePointZero);
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
    // RCLCPP_INFO(this->get_logger(), "Published vp_base calibration transform");
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

  void sample()
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
      publishTeleopTrigger(true);
      RCLCPP_INFO(this->get_logger(), "Teleop ENABLED");
    } else if (teleop_enabled_ && left_pinch > pinch_threshold_) {
      teleop_enabled_ = false;
      just_disabled = true;
      publishTeleopTrigger(false);
      //publishVpBaseCalibration(Eigen::Matrix4d::Identity());
      // back rot matrix -90 axis Z
      Eigen::Matrix4d calibration_T = createZRotationMatrix(-90.0);
      calibration_T(0, 3) = 0.0;
      calibration_T(1, 3) = 0.0;
      calibration_T(2, 3) = 0.0;
      publishVpBaseCalibration(calibration_T);
      RCLCPP_INFO(this->get_logger(), "Teleop DISABLED");
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
    // Calculate end-effector pose of the hand tracking 
    // Eigen::Vector3d ee_pos = (*right_thumb + *right_index) / 2.0;
    Eigen::Vector3d ee_pos = wrist_pose.first.value();
    Eigen::Quaterniond wrist_ori = *wrist_pose.second;

    // Create a quaternion for -20 degrees about X axis
    tf2::Quaternion q_x;
    q_x.setRPY(-35.0 * M_PI / 180.0, 0.0, 0.0);
    Eigen::Quaterniond q_x_eigen(q_x.w(), q_x.x(), q_x.y(), q_x.z());
    q_x_eigen.normalize();

    Eigen::Quaterniond q_flip = quaternionFromEulerZ(220.0);
    // Apply the -20deg X rotation before the Z rotation
    Eigen::Quaterniond ee_ori = multiplyQuat(wrist_ori, q_x_eigen);
    ee_ori = multiplyQuat(ee_ori, q_flip);

    ee_pos.x() -= 0.06;
    ee_pos.y() += 0.10;
    ee_pos.z() += 0.03;

    publishEeTargetTf(ee_pos, ee_ori, "ee_target", "vp_base");

    const double right_pinch = distance(*right_thumb, *right_index);
    const double d_clamped = clamp(right_pinch, right_pinch_min_, right_pinch_max_);

    geometry_msgs::msg::TransformStamped tf_h;
    try {
      tf_h = tf_buffer_->lookupTransform("map", "ee_target", tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;
    }
    Eigen::Matrix4d T_hand_map = transformToMatrix(tf_h);

    if (just_enabled) {
      try {
        auto tf_g = tf_buffer_->lookupTransform("map", "gripper_ee", tf2::TimePointZero);
        Eigen::Matrix4d T_gripper = transformToMatrix(tf_g);

        Eigen::Matrix4d T_hand_map_inv = T_hand_map.inverse();
        offset_T_ = T_hand_map_inv * T_gripper;

        offset_pos_ = T_gripper.block<3, 1>(0, 3) - T_hand_map.block<3, 1>(0, 3);
        Eigen::Quaterniond R_hand(T_hand_map.block<3, 3>(0, 0));
        Eigen::Quaterniond R_grip(T_gripper.block<3, 3>(0, 0));
        offset_rot_ = R_hand.conjugate() * R_grip;
        offset_rot_.normalize();

        offset_available_ = true;
        // RCLCPP_INFO(this->get_logger(), "Teleop offset captured (gripper frame: %s)", "gripper_ee");
      } catch (const tf2::TransformException & ex) {
        offset_available_ = false;
        RCLCPP_WARN(this->get_logger(), "Offset capture failed for frame '%s': %s", "gripper_ee", ex.what());
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
        // ee_ori_mycobot_base = flipPitchAndRoll(ee_ori_mycobot_base);

        // Apply translation scaling relative to the reference (offset) pose
        if (!ref_pose_set_) {
          ref_pos_mycobot_base_ = ee_pos_mycobot_base;
          ref_pose_set_ = true;
        }
        Eigen::Vector3d delta_pos = ee_pos_mycobot_base - ref_pos_mycobot_base_;
        ee_pos_mycobot_base = ref_pos_mycobot_base_ + translation_scale_ * delta_pos;

        // Apply rotation scaling by scaling the rotation angle
        if (rotation_scale_ != 1.0) {
          Eigen::AngleAxisd aa(ee_ori_mycobot_base);
          double angle = aa.angle();
          if (angle > 1e-6) {
            double scaled_angle = angle * rotation_scale_;
            aa = Eigen::AngleAxisd(scaled_angle, aa.axis());
            ee_ori_mycobot_base = Eigen::Quaterniond(aa);
            ee_ori_mycobot_base.normalize();
          }
        }

        const double normalized = (d_clamped - right_pinch_min_) /
          (right_pinch_max_ - right_pinch_min_);
        const double percent = clamp(normalized, 0.0, 1.0) * 100.0;
        const int32_t gripper_value = static_cast<int32_t>(std::round(percent));

        addSample(ee_pos_mycobot_base, ee_ori_mycobot_base, gripper_value);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Failed to transform ee_target_offset to mycobot_base: %s", ex.what());
      }

      if (just_disabled) {
        offset_available_ = false;
        clearSamples();
        ref_pose_set_ = false;
        RCLCPP_INFO(this->get_logger(), "Teleop offset cleared");
      }

      last_5hz_time_ = this->get_clock()->now();
    }
  }

  void publishSmoothedVis()
  {
    if (!teleop_enabled_ || !offset_available_ || samples_.empty()) {
      return;
    }

    Eigen::Vector3d avg_pos = Eigen::Vector3d::Zero();
    double gripper_sum = 0.0;

    bool first = true;
    Eigen::Quaterniond ref_q;
    Eigen::Vector4d accum_q(0.0, 0.0, 0.0, 0.0);  // (x, y, z, w)

    for (const auto & s : samples_) {
      avg_pos += s.pos_mycobot_base;
      gripper_sum += static_cast<double>(s.gripper);

      Eigen::Quaterniond q = s.ori_mycobot_base;
      if (first) {
        ref_q = q;
        first = false;
      } else {
        if (ref_q.dot(q) < 0.0) {
          q.coeffs() *= -1.0;
        }
      }
      accum_q[0] += q.x();
      accum_q[1] += q.y();
      accum_q[2] += q.z();
      accum_q[3] += q.w();
    }

    const double inv_n = 1.0 / static_cast<double>(samples_.size());
    avg_pos *= inv_n;
    accum_q *= inv_n;

		Eigen::Quaterniond avg_q(accum_q[3], accum_q[0], accum_q[1], accum_q[2]);
		const double n = avg_q.norm();
		if (!std::isfinite(n) || n < 1e-9) {
			avg_q = Eigen::Quaterniond::Identity();
		} else {
			avg_q.normalize();
		}

		ee_target_pos_ = avg_pos;
		ee_target_ori_ = avg_q;

    const double gripper_avg = gripper_sum * inv_n;
    gripper_smoothed_ = static_cast<int32_t>(std::round(gripper_avg));


    publishEeTargetTf(ee_target_pos_, ee_target_ori_, "ee_target_offset_mycobot_base_vis", "mycobot_base");
  }

  void publishSmoothed(){

    publishEeTargetTf(ee_target_pos_, ee_target_ori_, "ee_target_offset_mycobot_base", "mycobot_base");

		teleoperation::msg::TeleopTarget target_msg;
    target_msg.pose.header.stamp = this->get_clock()->now();
    target_msg.pose.header.frame_id = "mycobot_base";
    target_msg.pose.pose.position.x = ee_target_pos_.x();
    target_msg.pose.pose.position.y = ee_target_pos_.y();
    target_msg.pose.pose.position.z = ee_target_pos_.z();
    target_msg.pose.pose.orientation = eigenToMsg(ee_target_ori_);
    target_msg.gripper = gripper_smoothed_;

    ee_target_pub_->publish(target_msg);
  }



  struct Sample
  {
    Eigen::Vector3d pos_mycobot_base;
    Eigen::Quaterniond ori_mycobot_base;
    int32_t gripper;
  };

  void addSample(
    const Eigen::Vector3d & pos_mycobot_base,
    const Eigen::Quaterniond & ori_mycobot_base,
    int32_t gripper)
  {
    if (samples_.size() >= smoothing_window_) {
      samples_.pop_front();
    }
    samples_.push_back(Sample{pos_mycobot_base, ori_mycobot_base, gripper});
  }

  void clearSamples()
  {
    samples_.clear();
  }

  // Añade esto en el namespace anónimo (después de las otras funciones auxiliares)
Eigen::Matrix4d createZRotationMatrix(double degrees)
{
    const double radians = degrees * M_PI / 180.0;
    Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
    R(0, 0) = std::cos(radians);
    R(0, 1) = -std::sin(radians);
    R(1, 0) = std::sin(radians);
    R(1, 1) = std::cos(radians);
    return R;
}

  // Publishes true/false to indicate teleop enabled/disabled
  void publishTeleopTrigger(bool enabled) {
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    teleop_trigger_pub_->publish(msg);
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::Publisher<teleoperation::msg::TeleopTarget>::SharedPtr ee_target_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr teleop_trigger_pub_;
  rclcpp::TimerBase::SharedPtr sampling_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_vis_;

  double update_period_{};
	double update_period_vis_{};
  double pinch_threshold_{};
  double right_pinch_min_{};
  double right_pinch_max_{};
  double translation_scale_{1.0};
  double rotation_scale_{1.0};

  bool teleop_enabled_{false};
  bool offset_available_{false};
  Eigen::Matrix4d offset_T_ = Eigen::Matrix4d::Identity();
  Eigen::Vector3d offset_pos_ = Eigen::Vector3d::Zero();
	Eigen::Quaterniond offset_rot_{Eigen::Quaterniond::Identity()};
  rclcpp::Time last_5hz_time_;

  // Reference pose in mycobot_base used for movement scaling
  Eigen::Vector3d ref_pos_mycobot_base_ = Eigen::Vector3d::Zero();
  bool ref_pose_set_{false};

  std::deque<Sample> samples_;
  double smoothing_factor_{3.0};
  std::size_t smoothing_window_{3};
  
	Eigen::Vector3d ee_target_pos_ = Eigen::Vector3d::Zero();
	Eigen::Quaterniond ee_target_ori_{Eigen::Quaterniond::Identity()};
	int32_t gripper_smoothed_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleopControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
