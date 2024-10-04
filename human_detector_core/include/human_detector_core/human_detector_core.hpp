#pragma once
#include <sys/stat.h>

#include <chrono>
#include <execution>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
// ROS
#include <pcl/point_cloud.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <pcl_conversions/pcl_conversions.h>
// OpenMP
#include <omp.h>
// OpenCV
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std::chrono_literals;

class HumanDetectorCore : public rclcpp::Node
{
public:
  HumanDetectorCore(const rclcpp::NodeOptions &options) : HumanDetectorCore("", options) {}
  HumanDetectorCore(
      const std::string &name_space = "",
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : rclcpp::Node("human_detector_core_node", name_space, options)
  {
    RCLCPP_INFO(this->get_logger(), "start human_detector_core_node");
    mask_image_ = std::nullopt;
    human_depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "human_detector_core/human_depth", rclcpp::QoS(10));
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "depth", rclcpp::QoS(10),
        [&](const sensor_msgs::msg::Image::SharedPtr msg)
        {
        });
  }

private:
  std::optional<sensor_msgs::msg::Image> mask_image_;
  // subscriber
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  // publisher
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr human_depth_pub_;

  template <class T>
  T param(const std::string &name, const T &def)
  {
    T value;
    declare_parameter(name, def);
    get_parameter(name, value);
    return value;
  }

  // pcl::PointXYZ depth_to_xyz(float depth, size_t u, size_t v)
  // {
  //   pcl::PointXYZ p;
  //   float half_w = WIDTH / 2.f;
  //   float half_h = HEIGHT / 2.f;
  //   float theta = (u - half_w) * (2.f * M_PI / WIDTH);
  //   float phi = (v - half_h) * (M_PI / HEIGHT);
  //   p.x = depth * std::sin(theta) * std::cos(phi);
  //   p.y = depth * std::sin(phi);
  //   p.z = depth * std::cos(theta) * std::cos(phi);
  //   return p;
  // }

  template <typename POINT_TYPE = pcl::PointXYZ>
  inline sensor_msgs::msg::PointCloud2 make_ros_pointcloud2(std_msgs::msg::Header header, const pcl::PointCloud<POINT_TYPE> &cloud)
  {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header = header;
    return cloud_msg;
  }
};