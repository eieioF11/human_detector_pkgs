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
// extention node
#include "extension_node/extension_node.hpp"
// common_utils
#define USE_PCL
#define USE_ROS2
#include "common_utils/common_utils.hpp"
// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/msg/float32.hpp>
// OpenMP
#include <omp.h>
// OpenCV
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
// PCL
#include <pcl/point_cloud.h>
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

class HumanDetectorCore : public ext_rclcpp::ExtensionNode
{
public:
  HumanDetectorCore(const rclcpp::NodeOptions &options) : HumanDetectorCore("", options) {}
  HumanDetectorCore(
      const std::string &name_space = "",
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : ext_rclcpp::ExtensionNode("human_detector_core_node", name_space, options)
  {
    RCLCPP_INFO(this->get_logger(), "start human_detector_core_node");
    VOXEL_SIZE = param<double>("human_detector_core.voxel_size", 700.0);
    human_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "human_detector_core/human_cloud", rclcpp::QoS(10));
    scale_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "scale", rclcpp::QoS(10),
        [&](const std_msgs::msg::Float32::SharedPtr msg)
        {
          scale_ = msg->data;
        });
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "in_points", rclcpp::QoS(10),
        [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
          cloud_header_ = msg->header;
          pcl::PointCloud<pcl::PointXYZ> cloud;
          pcl::fromROSMsg(*msg, cloud);
          cloud = pcl_utils::voxelgrid_filter(cloud, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
          // スケール補正
          // #pragma omp parallel for schedule(dynamic)
          for (auto &p : cloud.points)
          {
            p.x *= scale_;
            p.y *= scale_;
            p.z *= scale_;
          }
          human_cloud_pub_->publish(ros2_utils::make_ros_pointcloud2(msg->header, cloud));
        });
  }

private:
  double VOXEL_SIZE;
  double scale_;
  std_msgs::msg::Header cloud_header_;
  // subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr scale_sub_;
  // publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr human_cloud_pub_;
};