// 近场补点：FAST-LIO blind=0.5 会丢掉离雷达 0.5 m 内的所有点（狗头前左方站人、贴墙都看不见）。
// 不改 FAST-LIO（近点会让 LIO 抖），本节点直接订原始 /livox/lidar（CustomMsg），
// 只留 [min_range, max_range] 的点，平移到 body 系（FAST-LIO extrinsic_T，旋转为单位阵），
// 发小点云 /near_cloud 给 stage2 并入 /scan。Python 反序列化 2 万点/帧太慢，所以用 C++。
// 自身点（雷达外壳/头顶）实测都在 0.2 m 内，min_range 默认 0.25（2026-10-02）。
#include <cmath>
#include <memory>
#include <string>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class NearFieldCloud : public rclcpp::Node
{
public:
  NearFieldCloud()
  : Node("near_field_cloud")
  {
    min_range_ = declare_parameter("min_range", 0.25);
    max_range_ = declare_parameter("max_range", 0.70);
    frame_id_ = declare_parameter("frame_id", std::string("body"));
    off_x_ = declare_parameter("lidar_to_body_x", -0.011);
    off_y_ = declare_parameter("lidar_to_body_y", -0.02329);
    off_z_ = declare_parameter("lidar_to_body_z", 0.04412);
    const auto in = declare_parameter("input_topic", std::string("/livox/lidar"));
    const auto out = declare_parameter("output_topic", std::string("/near_cloud"));

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out, rclcpp::SensorDataQoS());
    sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      in, rclcpp::SensorDataQoS(),
      [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) {on_raw(*msg);});
    RCLCPP_INFO(
      get_logger(), "near_field_cloud: %s -> %s, range [%.2f, %.2f] m, frame %s",
      in.c_str(), out.c_str(), min_range_, max_range_, frame_id_.c_str());
  }

private:
  void on_raw(const livox_ros_driver2::msg::CustomMsg & msg)
  {
    const double lo2 = min_range_ * min_range_;
    const double hi2 = max_range_ * max_range_;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = msg.header.stamp;
    cloud.header.frame_id = frame_id_;
    cloud.height = 1;
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(msg.points.size());

    sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x"), iy(cloud, "y"), iz(cloud, "z");
    size_t n = 0;
    for (const auto & p : msg.points) {
      const double r2 = double(p.x) * p.x + double(p.y) * p.y + double(p.z) * p.z;
      if (r2 < lo2 || r2 > hi2 || !std::isfinite(r2)) {
        continue;
      }
      *ix = p.x + off_x_;
      *iy = p.y + off_y_;
      *iz = p.z + off_z_;
      ++ix; ++iy; ++iz; ++n;
    }
    mod.resize(n);
    pub_->publish(cloud);
  }

  double min_range_, max_range_, off_x_, off_y_, off_z_;
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NearFieldCloud>());
  rclcpp::shutdown();
  return 0;
}
