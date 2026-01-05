#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <memory>
#include <string>
#include <optional>

class RGBPointCloudDownsampler : public rclcpp::Node
{
public:
  RGBPointCloudDownsampler()
  : Node("rgb_pointcloud_downsampler"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = this->declare_parameter<std::string>(
      "input_topic", "/camera/depth/color/points");
    output_topic_ = this->declare_parameter<std::string>(
      "output_topic", "/camera/depth/color/points_downsampled");
    target_frame_ = this->declare_parameter<std::string>(
      "target_frame", "mycobot_base");
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 5.0);
    downsample_factor_ = std::max(
      1, this->declare_parameter<int>("downsample_factor", 4));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::QoS(10));
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RGBPointCloudDownsampler::cloudCallback, this, std::placeholders::_1));

    if (publish_rate_hz_ > 0.0) {
      const double period = 1.0 / publish_rate_hz_;
      timer_ = this->create_wall_timer(
        std::chrono::duration<double>(period),
        std::bind(&RGBPointCloudDownsampler::publishLatest, this));
    }

    RCLCPP_INFO(
      this->get_logger(),
      "RGBPointCloudDownsampler started. input=%s output=%s target_frame=%s "
      "downsample_factor=%d publish_rate_hz=%.2f",
      input_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str(),
      downsample_factor_, publish_rate_hz_);
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 cloud = *msg;

    if (!target_frame_.empty() && msg->header.frame_id != target_frame_) {
      try {
        const geometry_msgs::msg::TransformStamped transform =
          tf_buffer_.lookupTransform(
            target_frame_, msg->header.frame_id, msg->header.stamp,
            tf2::durationFromSec(0.1));
        tf2::doTransform(*msg, cloud, transform);
        cloud.header.frame_id = target_frame_;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Transform from %s to %s failed: %s",
          msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
        return;
      }
    }

    latest_cloud_ = cloud;

    if (!timer_) {
      publishCloud(cloud);
    }
  }

  void publishLatest()
  {
    if (latest_cloud_.header.frame_id.empty()) {
      return;
    }
    publishCloud(latest_cloud_);
  }

  void publishCloud(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    const auto downsampled = downsampleCloud(cloud);
    if (!downsampled) {
      RCLCPP_DEBUG(this->get_logger(), "Downsampled cloud is empty, skipping publish.");
      return;
    }
    publisher_->publish(*downsampled);
  }

  std::optional<sensor_msgs::msg::PointCloud2> downsampleCloud(
    const sensor_msgs::msg::PointCloud2 & input) const
  {
    const int factor = std::max(1, downsample_factor_);
    if (factor == 1) {
      return input;
    }

    const size_t total_points =
      static_cast<size_t>(input.width) * static_cast<size_t>(input.height);
    if (total_points == 0) {
      return std::nullopt;
    }

    const size_t output_size = (total_points + static_cast<size_t>(factor) - 1) /
      static_cast<size_t>(factor);

    sensor_msgs::msg::PointCloud2 output;
    output.header = input.header;
    output.height = 1;
    output.is_bigendian = input.is_bigendian;
    output.is_dense = input.is_dense;

    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(output_size);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(input, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(input, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(input, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_rgb(input, "rgb");

    sensor_msgs::PointCloud2Iterator<float> out_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(output, "z");
    sensor_msgs::PointCloud2Iterator<float> out_rgb(output, "rgb");

    size_t written = 0;
    for (size_t idx = 0; idx < total_points; ++idx, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb) {
      if (idx % static_cast<size_t>(factor) != 0) {
        continue;
      }
      *out_x = *iter_x;
      *out_y = *iter_y;
      *out_z = *iter_z;
      *out_rgb = *iter_rgb;
      ++out_x;
      ++out_y;
      ++out_z;
      ++out_rgb;
      ++written;
    }

    modifier.resize(written);
    output.width = static_cast<uint32_t>(written);
    output.row_step = output.point_step * output.width;

    if (written == 0) {
      return std::nullopt;
    }

    return output;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  double publish_rate_hz_{5.0};
  int downsample_factor_{1};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::PointCloud2 latest_cloud_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RGBPointCloudDownsampler>());
  rclcpp::shutdown();
  return 0;
}
