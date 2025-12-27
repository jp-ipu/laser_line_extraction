#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "laser_line_extraction/line_extraction_ros.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<line_extraction::LineExtractionROS>();

  RCLCPP_DEBUG(node->get_logger(), "Starting line_extraction_node.");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}