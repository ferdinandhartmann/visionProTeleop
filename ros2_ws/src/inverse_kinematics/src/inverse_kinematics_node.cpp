#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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
    joint_names_{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"},
    joint_state_(6, 0.0)
  {
    declare_parameter<std::string>("target_topic", "/teleop/ee_target");
    declare_parameter<std::string>("joint_target_topic", "/teleop/joint_targets");

    const auto target_topic = get_parameter("target_topic").as_string();
    const auto joint_target_topic = get_parameter("joint_target_topic").as_string();

    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      target_topic, rclcpp::SensorDataQoS(),
      std::bind(&InverseKinematicsNode::poseCallback, this, std::placeholders::_1));

    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      joint_target_topic, 10);

    dh_chain_ = buildDhParameters();

    RCLCPP_INFO(get_logger(), "Inverse kinematics node listening to %s", target_topic.c_str());
    RCLCPP_INFO(get_logger(), "Publishing joint targets to %s", joint_target_topic.c_str());
  }

private:
  static std::vector<DhParameter> buildDhParameters()
  {
    // Convert millimeter distances from the provided DH table into meters.
    constexpr double mm_to_m = 0.001;
    return {
      DhParameter{0.0, 0.0, 131.56 * mm_to_m, 0.0},
      DhParameter{M_PI_2, 0.0, 0.0, -M_PI_2},
      DhParameter{0.0, -110.4 * mm_to_m, 0.0, 0.0},
      DhParameter{0.0, -96.0 * mm_to_m, 64.62 * mm_to_m, -M_PI_2},
      DhParameter{M_PI_2, 0.0, 73.18 * mm_to_m, M_PI_2},
      DhParameter{-M_PI_2, 0.0, 48.6 * mm_to_m, 0.0},
    };
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    Pose target = poseFromMsg(*msg);

    std::vector<double> solution;
    bool success = solveInverseKinematics(target, joint_state_, solution);

    if (!success)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Inverse kinematics did not converge for the requested pose");
      return;
    }

    joint_state_ = solution;
    publishJointState(msg->header);
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

  Pose forwardKinematics(const std::vector<double> & joints) const
  {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (size_t i = 0; i < dh_chain_.size(); ++i)
    {
      const auto & dh = dh_chain_[i];
      const double theta = joints[i] + dh.offset;
      T = T * dhTransform(dh.alpha, dh.a, dh.d, theta);
    }

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
    const double position_tolerance = 1e-4;      // meters
    const double orientation_tolerance = 1e-3;   // radians
    const size_t max_iterations = 200;
    const double damping = 1e-3;

    Eigen::VectorXd joints = Eigen::Map<const Eigen::VectorXd>(seed.data(), seed.size());

    for (size_t iter = 0; iter < max_iterations; ++iter)
    {
      Pose current = forwardKinematics(std::vector<double>(joints.data(), joints.data() + joints.size()));
      Eigen::Matrix<double, 6, 1> error = computeError(current, target);

      if (error.block<3, 1>(0, 0).norm() < position_tolerance &&
          error.block<3, 1>(3, 0).norm() < orientation_tolerance)
      {
        solution.assign(joints.data(), joints.data() + joints.size());
        return true;
      }

      Eigen::Matrix<double, 6, 6> J = computeJacobian(
        std::vector<double>(joints.data(), joints.data() + joints.size()), current);

      Eigen::Matrix<double, 6, 6> damping_matrix = damping * damping * Eigen::Matrix<double, 6, 6>::Identity();
      Eigen::Matrix<double, 6, 6> lhs = J * J.transpose() + damping_matrix;
      Eigen::Matrix<double, 6, 1> delta = J.transpose() * lhs.ldlt().solve(error);

      joints += delta;
    }

    return false;
  }

  void publishJointState(const std_msgs::msg::Header & header)
  {
    sensor_msgs::msg::JointState msg;
    msg.header = header;
    msg.name = joint_names_;
    msg.position = joint_state_;
    joint_state_publisher_->publish(msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

  std::vector<DhParameter> dh_chain_;
  std::vector<std::string> joint_names_;
  std::vector<double> joint_state_;
};

}  // namespace inverse_kinematics

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<inverse_kinematics::InverseKinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
