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
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
// OpenMP
#include <omp.h>
// OpenCV
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
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

class HumanDetectorCore : public ext_rclcpp::ExtensionNode {
public:
  HumanDetectorCore(const rclcpp::NodeOptions& options) : HumanDetectorCore("", options) {}
  HumanDetectorCore(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : ext_rclcpp::ExtensionNode("human_detector_core_node", name_space, options) {
    RCLCPP_INFO(this->get_logger(), "start human_detector_core_node");
    VOXEL_SIZE                = param<double>("human_detector_core.voxel_size", 700.0);
    CLUSTER_TOLERANCE         = param<double>("human_detector_core.cluster_tolerance", 0.5);
    MIN_CLUSTER_SIZE          = param<int>("human_detector_core.min_cluster_size", 50);
    box_size_                 = param<std::vector<double>>("human_detector_core.box_size", {0.5, 1.5, 0.5});
    std::string LOG_LEVEL_STR = param<std::string>("human_detector_core.log_level", "DEBUG"); // DEBUG, INFO, WARN, ERROR, NONE
    logger.set_log_level(ext::get_log_level(LOG_LEVEL_STR));
    human_cloud_pub_  = this->create_publisher<sensor_msgs::msg::PointCloud2>("human_detector_core/human_cloud", rclcpp::QoS(10));
    human_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("human_detector_core/human_marker", rclcpp::QoS(10));
    scale_sub_        = this->create_subscription<std_msgs::msg::Float32>("scale", rclcpp::QoS(10),
                                                                          [&](const std_msgs::msg::Float32::SharedPtr msg) { scale_ = msg->data; });
    cloud_sub_        = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "in_points", rclcpp::QoS(10), [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          cloud_header_ = msg->header;
          pcl::PointCloud<pcl::PointXYZ> cloud;
          pcl::fromROSMsg(*msg, cloud);
          cloud = pcl_utils::voxelgrid_filter(cloud, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
          // スケール補正
          // #pragma omp parallel for schedule(dynamic)
          for (auto& p : cloud.points) {
            p.x *= scale_;
            p.y *= scale_;
            p.z *= scale_;
          }
          std::vector<int> mapping;
          pcl::removeNaNFromPointCloud(cloud, cloud, mapping);
          human_cloud_pub_->publish(ros2_utils::make_ros_pointcloud2(msg->header, cloud));
          std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters = pcl_utils::euclidean_clustering(cloud, CLUSTER_TOLERANCE, MIN_CLUSTER_SIZE);
          std::cout << logger.DEBUG() << "cluster size: " << clusters.size() << logger.endl;
          int id                         = 0;
          std_msgs::msg::ColorRGBA color = ros2_utils::make_color(0.0, 0.0, 1.0, 0.5);
          visualization_msgs::msg::MarkerArray marker_array;
#pragma omp parallel for schedule(dynamic)
          for (const auto& cluster : clusters) {
            // calc inner product
            std::vector<double> x, y, z;
            for (auto& cp : cluster.points) {
              Eigen::Vector3d p(cp.x, cp.y, cp.z);
              x.push_back(cp.x);
              y.push_back(cp.y);
              z.push_back(cp.z);
            }
            // calc median
            double x_median = calc_median(x);
            double y_median = calc_median(y);
            double z_median = calc_median(z);
            std::cout << "median:" << x_median;
            std::cout << "," << y_median << ",";
            std::cout << z_median << std::endl;

            geometry_msgs::msg::Vector3 size       = ros2_utils::make_geometry_vector3(box_size_[0], box_size_[1], box_size_[2]);
            geometry_msgs::msg::Vector3 pos        = ros2_utils::make_geometry_vector3(x_median, y_median, z_median);
            visualization_msgs::msg::Marker marker = ros2_utils::make_area_maker(cloud_header_, pos, size, id, color);
            marker.ns                              = "human";
            marker_array.markers.push_back(marker);
            id++;
          }
          human_marker_pub_->publish(marker_array);
        });
  }

private:
  double VOXEL_SIZE;
  double CLUSTER_TOLERANCE;
  int MIN_CLUSTER_SIZE;
  double scale_;
  std::vector<double> box_size_{0.5, 1.5, 0.5};
  std_msgs::msg::Header cloud_header_;
  // log
  ext::Logger logger;
  // subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr scale_sub_;
  // publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr human_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr human_marker_pub_;
  double calc_median(std::vector<double>& v) {
    size_t size = v.size();
    std::sort(v.begin(), v.end());
    size_t median_index = size / 2;
    double median       = (size % 2 == 0 ? (v[median_index] + v[median_index - 1]) / 2 : v[median_index]);
    return median;
  }
};