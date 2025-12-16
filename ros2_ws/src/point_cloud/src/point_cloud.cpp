#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <librealsense2/rs.hpp>

#include <cmath>
#include <cstring>

class RealSensePointCloudNode : public rclcpp::Node
{
public:
  RealSensePointCloudNode()
  : Node("realsense_pointcloud_node"),
    align_(RS2_STREAM_COLOR)
  {
    topic_ = this->declare_parameter("topic", "/camera/depth/color/points");
    frame_id_ = this->declare_parameter("frame_id", "camera_color_optical_frame");

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      topic_, rclcpp::SensorDataQoS());

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);

    try {
      pipe_.start(cfg);
      RCLCPP_INFO(get_logger(), " RealSense pipeline started");
    } catch (const rs2::error &e) {
      RCLCPP_FATAL(get_logger(), " RealSense error: %s", e.what());
      throw;
    }

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&RealSensePointCloudNode::timer_callback, this));
  }

private:
  static float pack_rgb(uint8_t r, uint8_t g, uint8_t b)
  {
    uint32_t rgb = (r << 16) | (g << 8) | b;
    float out;
    std::memcpy(&out, &rgb, sizeof(float));
    return out;
  }

  void timer_callback()
  {
    rs2::frameset frames;
    if (!pipe_.poll_for_frames(&frames)) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                            " Waiting for frames...");
      return;
    }

    frames = align_.process(frames);

    auto depth = frames.get_depth_frame();
    auto color = frames.get_color_frame();

    if (!depth || !color) {
      RCLCPP_WARN(get_logger(), " Missing depth or color frame");
      return;
    }

    pc_.map_to(color);
    rs2::points points = pc_.calculate(depth);

    const auto *vertices = points.get_vertices();
    const auto *tex = points.get_texture_coordinates();
    const uint8_t *img = reinterpret_cast<const uint8_t *>(color.get_data());

    int w = color.get_width();
    int h = color.get_height();
    int stride = color.get_bytes_per_pixel();

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.height = depth.get_height();
    msg.width = depth.get_width();
    msg.is_dense = false;
    msg.is_bigendian = false;

    msg.fields.resize(4);

    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;

    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;

    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;

    msg.fields[3].name = "rgb";
    msg.fields[3].offset = 12;
    msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[3].count = 1;

    msg.point_step = 16;
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(msg.row_step * msg.height);

    sensor_msgs::PointCloud2Iterator<float> ix(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(msg, "z");
    sensor_msgs::PointCloud2Iterator<float> irgb(msg, "rgb");

    for (size_t i = 0; i < points.size(); ++i, ++ix, ++iy, ++iz, ++irgb) {
      const auto &v = vertices[i];
      *ix = v.x;
      *iy = v.y;
      *iz = v.z;

      // Print ix iy iz values for debugging
      RCLCPP_INFO(get_logger(), "Point %zu: x=%f, y=%f, z=%f", i, *ix, *iy, *iz);

      uint8_t r = 0, g = 0, b = 0;
      if (std::isfinite(v.z) && tex) {
        int u = std::clamp(int(tex[i].u * w), 0, w - 1);
        int v_ = std::clamp(int(tex[i].v * h), 0, h - 1);
        int idx = (v_ * w + u) * stride;
        r = img[idx];
        g = img[idx + 1];
        b = img[idx + 2];
      }
      *irgb = pack_rgb(r, g, b);
    }

    pub_->publish(msg);

    // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    //                      " Published point cloud: %zu points",
    //                      points.size());
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rs2::pipeline pipe_;
  rs2::pointcloud pc_;
  rs2::align align_;

  std::string topic_;
  std::string frame_id_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealSensePointCloudNode>());
  rclcpp::shutdown();
  return 0;
}