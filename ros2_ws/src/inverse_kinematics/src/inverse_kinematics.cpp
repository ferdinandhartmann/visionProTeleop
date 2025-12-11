#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "teleoperation/msg/teleop_target.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace inverse_kinematics
{

struct DhParameter
{
  double alpha;
  double a;
  double d;
  double offset;
};

struct Pose
{
  Eigen::Vector3d position;
  Eigen::Matrix3d rotation;
};

class InverseKinematicsNode : public rclcpp::Node
{
public:
InverseKinematicsNode()
: Node("inverse_kinematics_node"),
  joint_names_{
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "gripper_controller"
  },
  joint_state_(6, 0.0)
{
  declare_parameter<std::string>("target_topic", "/teleop/ee_target");
  declare_parameter<std::string>("joint_target_topic", "/joint_states_mycobot");
  declare_parameter<std::string>("mycobot_base_frame", "mycobot_base");
  declare_parameter<std::string>("ee_child_frame", "gripper_ee");

  // IK tuning parameters (exposed as ROS 2 parameters / YAML)
  declare_parameter<double>("position_tolerance", 0.001);
  declare_parameter<double>("orientation_tolerance", 0.001);
  declare_parameter<int>("max_iterations", 300);
  declare_parameter<double>("damping", 0.008);
  declare_parameter<double>("step_tolerance", 2e-6);
  declare_parameter<int>("gripper_percent", 100);
  declare_parameter<double>("joint_states_ros2_update_rate", 50.0);  // Hz
  // Joint limits in degrees for joints 1..6
  declare_parameter<std::vector<double>>("joint_lower_limits_deg",
    std::vector<double>{-170.0, -170.0, -170.0, -170.0, -170.0, -180.0});
  declare_parameter<std::vector<double>>("joint_upper_limits_deg",
    std::vector<double>{ 170.0,  170.0,  170.0,  170.0,  170.0,  180.0});
  // Optional initial joint configuration in degrees (6 values: joint1..joint6).
  // Use a typed default (double array); YAML must then provide doubles.
  declare_parameter<std::vector<double>>("initial_joint_positions_deg", std::vector<double>{});

  const auto target_topic = get_parameter("target_topic").as_string();
  const auto joint_target_topic = get_parameter("joint_target_topic").as_string();
  mycobot_base_frame_ = get_parameter("mycobot_base_frame").as_string();
  ee_child_frame_ = get_parameter("ee_child_frame").as_string();

  // Read IK tuning parameters into member variables
  position_tolerance_ = get_parameter("position_tolerance").as_double();
  orientation_tolerance_ = get_parameter("orientation_tolerance").as_double();
  max_iterations_ = static_cast<size_t>(get_parameter("max_iterations").as_int());
  damping_ = get_parameter("damping").as_double();
  step_tolerance_ = get_parameter("step_tolerance").as_double();
	gripper_percent_ = get_parameter("gripper_percent").as_int();
  joint_states_ros2_update_rate_ = get_parameter("joint_states_ros2_update_rate").as_double();
  if (joint_states_ros2_update_rate_ <= 0.0) {
    RCLCPP_WARN(get_logger(),
      "joint_states_ros2_update_rate must be > 0. Using 50 Hz fallback.");
    joint_states_ros2_update_rate_ = 50.0;
  }

  // Read joint limits (degrees) and convert to radians
  joint_lower_limits_rad_.resize(6);
  joint_upper_limits_rad_.resize(6);
  const auto joint_lower_limits_deg = get_parameter("joint_lower_limits_deg").as_double_array();
  const auto joint_upper_limits_deg = get_parameter("joint_upper_limits_deg").as_double_array();
  if (joint_lower_limits_deg.size() == 6 && joint_upper_limits_deg.size() == 6) {
    for (size_t i = 0; i < 6; ++i) {
      joint_lower_limits_rad_[i] = joint_lower_limits_deg[i] * M_PI / 180.0;
      joint_upper_limits_rad_[i] = joint_upper_limits_deg[i] * M_PI / 180.0;
    }
  } else {
    RCLCPP_WARN(get_logger(),
      "Parameters 'joint_lower_limits_deg' and 'joint_upper_limits_deg' must have 6 values; using built-in defaults.");
    const std::array<double, 6> default_lower_deg{ -170.0, -170.0, -170.0, -170.0, -170.0, -180.0 };
    const std::array<double, 6> default_upper_deg{  170.0,  170.0,  170.0,  170.0,  170.0,  180.0 };
    for (size_t i = 0; i < 6; ++i) {
      joint_lower_limits_rad_[i] = default_lower_deg[i] * M_PI / 180.0;
      joint_upper_limits_rad_[i] = default_upper_deg[i] * M_PI / 180.0;
    }
  }

  // Optional: override initial joint_state_ from parameter (degrees -> radians)
  const auto initial_joints_deg = get_parameter("initial_joint_positions_deg").as_double_array();
    if (!initial_joints_deg.empty()) {
      if (initial_joints_deg.size() != 6) {
        RCLCPP_WARN(
          get_logger(),
          "Parameter 'initial_joint_positions_deg' must have 6 values; got %zu. Ignoring.",
          initial_joints_deg.size());
      } else {
        for (size_t i = 0; i < 6; ++i) {
          joint_state_[i] = initial_joints_deg[i] * M_PI / 180.0;
        }
        RCLCPP_INFO(get_logger(),
          "Initial joint_state set from initial_joint_positions_deg (degrees): [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
          initial_joints_deg[0], initial_joints_deg[1], initial_joints_deg[2],
          initial_joints_deg[3], initial_joints_deg[4], initial_joints_deg[5]);
      }
    }


  pose_subscription_ = create_subscription<teleoperation::msg::TeleopTarget>(target_topic, rclcpp::SensorDataQoS(),
                            std::bind(&InverseKinematicsNode::poseCallback, this, std::placeholders::_1));

  joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(joint_target_topic, 10);

  // Continuous JointState publisher on /joint_states_ros2
  joint_state_ros2_publisher_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  int period_ms = static_cast<int>(1000.0 / joint_states_ros2_update_rate_);
  if (period_ms <= 0) {
    period_ms = 20;  // fallback 50 Hz
  }
  joint_state_timer_ = create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&InverseKinematicsNode::jointStateTimerCallback, this));

  dh_chain_ = buildDhParameters();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  RCLCPP_INFO(get_logger(), "Inverse kinematics node listening to %s", target_topic.c_str());
  RCLCPP_INFO(get_logger(), "Publishing joint targets to %s", joint_target_topic.c_str());
}

private:

  void poseCallback(const teleoperation::msg::TeleopTarget::SharedPtr msg)
  {
    Pose target = poseFromMsg(msg->pose);
    gripper_percent_ = std::clamp(msg->gripper, 0, 100);

    // RCLCPP_INFO(get_logger(),
    //   "Received target pose: frame=%s pos=(%.3f, %.3f, %.3f)",
    //   msg->header.frame_id.c_str(),
    //   target.position.x(), target.position.y(), target.position.z());

    std::vector<double> solution;
    bool success = solveInverseKinematics(target, joint_state_, solution);

    if (success){ 
      joint_state_ = solution;
    }
    else{
      RCLCPP_WARN(get_logger(), "IK solution not found; publishing last known joint state");
    }

    publishEndEffectorTf(msg->pose.header);
    publishJointState(msg->pose.header);
    
    return;
  }

  // Periodic publisher for /joint_states_ros2
  void jointStateTimerCallback()
  {
    if (!joint_state_ros2_publisher_) {
      return;
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.header.frame_id = mycobot_base_frame_;
    msg.name = joint_names_;
    msg.position.assign(joint_state_.begin(), joint_state_.end());

    double gripper_joint_value = gripper_lower_limit_ +
      (gripper_upper_limit_ - gripper_lower_limit_) *
      (static_cast<double>(gripper_percent_) / 100.0);
    msg.position.push_back(gripper_joint_value);

    joint_state_ros2_publisher_->publish(msg);

    publishEndEffectorTf(msg.header);
  }

  Pose poseFromMsg(const geometry_msgs::msg::PoseStamped & msg) const
  {
    Pose pose{};
    pose.position = Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);

    tf2::Quaternion q;
    tf2::fromMsg(msg.pose.orientation, q);
    q.normalize();
    tf2::Matrix3x3 tf2_rot(q);
    pose.rotation << tf2_rot[0][0], tf2_rot[0][1], tf2_rot[0][2],
      tf2_rot[1][0], tf2_rot[1][1], tf2_rot[1][2],
      tf2_rot[2][0], tf2_rot[2][1], tf2_rot[2][2];
    return pose;
  }

  static Eigen::Matrix4d dhTransform(double alpha, double a, double d, double theta)
  {
    const double c_theta = std::cos(theta);
    const double s_theta = std::sin(theta);
    const double c_alpha = std::cos(alpha);
    const double s_alpha = std::sin(alpha);

    Eigen::Matrix4d T;
    T << c_theta, -s_theta * c_alpha, s_theta * s_alpha, a * c_theta,
      s_theta, c_theta * c_alpha, -c_theta * s_alpha, a * s_theta,
      0.0, s_alpha, c_alpha, d,
      0.0, 0.0, 0.0, 1.0;
    return T;
  }

  static Eigen::Matrix3d rpyToMatrix(double roll, double pitch, double yaw)
  {
    Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
    // URDF uses R = Rz(yaw) * Ry(pitch) * Rx(roll)
    Eigen::Matrix3d R = (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
    return R;
  }

  Pose forwardKinematics(const std::vector<double> & joints) const
  {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (size_t i = 0; i < dh_chain_.size(); ++i)
    {
      const auto & dh = dh_chain_[i];
      const double theta = joints[i] + dh.offset;
      T = T * dhTransform(dh.alpha, dh.a, dh.d, theta);
    }

    // joint7_to_camera:
    // origin xyz="0 0 0.01" rpy="1.579 0 0.7854"
    Eigen::Matrix4d T_joint7_to_camera = Eigen::Matrix4d::Identity();
    T_joint7_to_camera.block<3, 3>(0, 0) = rpyToMatrix(1.579, 0.0, 0.7854);
    T_joint7_to_camera.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, 0.0, 0.01);
    // camera_flange_to_gripper_base:
    //   origin xyz="0.0 0.041 0.0" rpy="0 -1.57 0"
    Eigen::Matrix4d T_camera_to_gripper = Eigen::Matrix4d::Identity();
    T_camera_to_gripper.block<3, 3>(0, 0) = rpyToMatrix(0.0, -1.57, 0.0);
    T_camera_to_gripper.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, 0.041, 0.0);

    T = T * T_joint7_to_camera * T_camera_to_gripper;

    // Constant offset to place the pose in the grippers middle
    constexpr double gripper_offset_front = 0.05;  
    constexpr double gripper_offset_down = -0.01; 
    Eigen::Vector3d tool_offset = T.block<3, 3>(0, 0) * Eigen::Vector3d(0.0, gripper_offset_front, gripper_offset_down);
    T.block<3, 1>(0, 3) += tool_offset;

    // Apply rotation: 90 deg around X
    Eigen::Matrix3d rot_z = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T.block<3,3>(0,0) = T.block<3,3>(0,0) * rot_z;

    Pose pose;
    pose.position = T.block<3, 1>(0, 3);
    pose.rotation = T.block<3, 3>(0, 0);
    return pose;
  }

  static Eigen::Vector3d orientationError(const Eigen::Matrix3d & current,
                                          const Eigen::Matrix3d & target)
  {
    const Eigen::Matrix3d r_err = current.transpose() * target;
    const double cos_angle = (r_err.trace() - 1.0) * 0.5;
    double angle = std::acos(std::clamp(cos_angle, -1.0, 1.0));

    if (angle < 1e-6)
    {
      return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d axis;
    axis << r_err(2, 1) - r_err(1, 2),
      r_err(0, 2) - r_err(2, 0),
      r_err(1, 0) - r_err(0, 1);
    axis *= 1.0 / (2.0 * std::sin(angle));
    return axis * angle;
  }

  Eigen::Matrix<double, 6, 6> computeJacobian(const std::vector<double> & joints,
                                              const Pose & current_pose) const
  {
    const double eps = 1e-6;
    Eigen::Matrix<double, 6, 6> jacobian;

    for (size_t i = 0; i < joints.size(); ++i)
    {
      std::vector<double> perturbed = joints;
      perturbed[i] += eps;
      Pose pert_pose = forwardKinematics(perturbed);

      Eigen::Vector3d dp = (pert_pose.position - current_pose.position) / eps;

      Eigen::Matrix3d r_err = current_pose.rotation.transpose() * pert_pose.rotation;
      const double cos_angle = (r_err.trace() - 1.0) * 0.5;
      double angle = std::acos(std::clamp(cos_angle, -1.0, 1.0));
      Eigen::Vector3d axis = Eigen::Vector3d::Zero();
      if (angle > 1e-9)
      {
        axis << r_err(2, 1) - r_err(1, 2),
          r_err(0, 2) - r_err(2, 0),
          r_err(1, 0) - r_err(0, 1);
        axis *= 1.0 / (2.0 * std::sin(angle));
      }
      Eigen::Vector3d dtheta = axis * angle / eps;

      jacobian.block<3, 1>(0, i) = dp;
      jacobian.block<3, 1>(3, i) = dtheta;
    }

    return jacobian;
  }

  Eigen::Matrix<double, 6, 1> computeError(const Pose & current, const Pose & target) const
  {
    Eigen::Matrix<double, 6, 1> error;
    error.block<3, 1>(0, 0) = target.position - current.position;
    error.block<3, 1>(3, 0) = orientationError(current.rotation, target.rotation);
    return error;
  }

  bool solveInverseKinematics(const Pose & target,
                              const std::vector<double> & seed,
                              std::vector<double> & solution) const
  {
    const double position_tolerance = position_tolerance_;      // meters
    const double orientation_tolerance = orientation_tolerance_;   // radians
    const size_t max_iterations = max_iterations_;
    const double damping = damping_;
    const double step_tolerance = step_tolerance_;          // rad magnitude of joint update

    Eigen::VectorXd joints = Eigen::Map<const Eigen::VectorXd>(seed.data(), seed.size());

    // Enforce joint limits on initial seed
    for (size_t i = 0; i < static_cast<size_t>(joints.size()) && i < joint_lower_limits_rad_.size(); ++i)
    {
      joints[i] = std::clamp(joints[i], joint_lower_limits_rad_[i], joint_upper_limits_rad_[i]);
    }

    Eigen::Matrix<double, 6, 1> last_error;

    for (size_t iter = 0; iter < max_iterations; ++iter)
    {
      Pose current = forwardKinematics(std::vector<double>(joints.data(), joints.data() + joints.size()));
      Eigen::Matrix<double, 6, 1> error = computeError(current, target);
      last_error = error;

      if (error.block<3, 1>(0, 0).norm() < position_tolerance &&
          error.block<3, 1>(3, 0).norm() < orientation_tolerance)
      {
        solution.assign(joints.data(), joints.data() + joints.size());
        // RCLCPP_INFO(rclcpp::get_logger("inverse_kinematics_node"),
        //   "IK converged in %zu iterations (pos_err=%.3e, ori_err=%.3e)", iter, error.block<3, 1>(0, 0).norm(), error.block<3, 1>(3, 0).norm());
        return true;
      }

      Eigen::Matrix<double, 6, 6> J = computeJacobian(
        std::vector<double>(joints.data(), joints.data() + joints.size()), current);

      Eigen::Matrix<double, 6, 6> damping_matrix = damping * damping * Eigen::Matrix<double, 6, 6>::Identity();
      Eigen::Matrix<double, 6, 6> lhs = J * J.transpose() + damping_matrix;
      Eigen::Matrix<double, 6, 1> delta = J.transpose() * lhs.ldlt().solve(error);

      joints += delta;

      // Enforce joint limits after each update
      for (size_t i = 0; i < static_cast<size_t>(joints.size()) && i < joint_lower_limits_rad_.size(); ++i)
      {
        joints[i] = std::clamp(joints[i], joint_lower_limits_rad_[i], joint_upper_limits_rad_[i]);
      }

    //   if (iter == 0 || (iter + 1) == max_iterations)
    //   {
    //     RCLCPP_INFO(rclcpp::get_logger("inverse_kinematics_node"),
    //       "IK iter %zu: pos_err=%.3e, ori_err=%.3e, |delta|=%.3e", iter, error.block<3, 1>(0, 0).norm(), error.block<3, 1>(3, 0).norm(), delta.norm());
    //   }

      // If only tiny joint updates, finish early
      if (delta.norm() < step_tolerance)
      {
        solution.assign(joints.data(), joints.data() + joints.size());
        RCLCPP_INFO(rclcpp::get_logger("inverse_kinematics_node"),
          "IK stopped by step tolerance at iter %zu (pos_err=%.3e, ori_err=%.3e)", iter, error.block<3, 1>(0, 0).norm(), error.block<3, 1>(3, 0).norm());
        return true;
      }

    // Log every 100 iterations
    // if (iter == 0 || (iter + 1) == max_iterations || (iter % 100 == 0))
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("inverse_kinematics_node"),
    //       "IK iter %zu: pos_err=%.3e, ori_err=%.3e, |delta|=%.3e", iter, error.block<3, 1>(0, 0).norm(), error.block<3, 1>(3, 0).norm(), delta.norm());
    // }
    }

    RCLCPP_INFO(rclcpp::get_logger("inverse_kinematics_node"),
      "IK failed after %zu iterations: final pos_err=%.3e, ori_err=%.3e", max_iterations, last_error.block<3, 1>(0, 0).norm(), last_error.block<3, 1>(3, 0).norm());

    return false;
  }

  void publishJointState(const std_msgs::msg::Header & header)
  {
    sensor_msgs::msg::JointState msg;
    msg.header = header;
    msg.name = joint_names_;
    msg.position.assign(joint_state_.begin(), joint_state_.end());
    double gripper_joint_value = gripper_lower_limit_ + (gripper_upper_limit_ - gripper_lower_limit_) * (static_cast<double>(gripper_percent_) / 100.0);
    msg.position.push_back(gripper_joint_value);
    joint_state_publisher_->publish(msg);

    publishEndEffectorTf(header);
  }

  void publishEndEffectorTf(const std_msgs::msg::Header & header)
  {
    if (!tf_broadcaster_)
    {
      RCLCPP_WARN(get_logger(), "TF broadcaster not initialized");
      return;
    }

    Pose ee_pose = forwardKinematics(joint_state_);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = header.stamp;
    tf_msg.header.frame_id = mycobot_base_frame_;
    tf_msg.child_frame_id = ee_child_frame_;

    tf_msg.transform.translation.x = ee_pose.position.x();
    tf_msg.transform.translation.y = ee_pose.position.y();
    tf_msg.transform.translation.z = ee_pose.position.z();

    Eigen::Quaterniond q(ee_pose.rotation);
    q.normalize();
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
  }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_ros2_publisher_;
    rclcpp::TimerBase::SharedPtr joint_state_timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::vector<DhParameter> dh_chain_;
    std::vector<std::string> joint_names_;
    std::vector<double> joint_state_;
    std::string mycobot_base_frame_;
    std::string ee_child_frame_;

    double position_tolerance_{};
    double orientation_tolerance_{};
    size_t max_iterations_{};
    double damping_{};
    double step_tolerance_{};

    int gripper_percent_{};
    const float gripper_lower_limit_ = -0.74;
    const float gripper_upper_limit_ = 0.15;

    double joint_states_ros2_update_rate_{};

    std::vector<double> joint_lower_limits_rad_;
    std::vector<double> joint_upper_limits_rad_;

    static std::vector<DhParameter> buildDhParameters()
    {
			constexpr double mm = 0.001;
			return {
					DhParameter{ +1.5708,      0.0,     131.22 * mm,    0.0 },
					DhParameter{  0.0,        -110.4 * mm, 0.0,        -1.5708 },
					DhParameter{  0.0,         -96.0 * mm, 0.0,         0.0 },
					DhParameter{ +1.5708,       0.0,      63.4 * mm,   -1.5708 },
					DhParameter{ -1.5708,       0.0,      75.05 * mm,  +1.5708 },
					DhParameter{  0.0,          0.0,      45.6 * mm,    0.0 }
			};
    }
};

}  // namespace inverse_kinematics

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<inverse_kinematics::InverseKinematicsNode>());
    rclcpp::shutdown();
    return 0;
}
