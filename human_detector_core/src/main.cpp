#include <rclcpp/rclcpp.hpp>
#include "human_detector_core/human_detector_core.hpp"

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HumanDetectorCore>());
  rclcpp::shutdown();
  return 0;
}