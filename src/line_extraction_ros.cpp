#include "laser_line_extraction/line_extraction_ros.h"

#include <chrono>
#include <cmath>
#include <functional>

using namespace std::chrono_literals;

namespace line_extraction
{

///////////////////////////////////////////////////////////////////////////////
// Constructor / destructor
///////////////////////////////////////////////////////////////////////////////
LineExtractionROS::LineExtractionROS(const rclcpp::NodeOptions & options)
  : Node("line_extraction_node", options),
    data_cached_(false)
{
  declareParameters();
  loadParameters();

  // Create publishers
  line_publisher_ = this->create_publisher<laser_line_extraction::msg::LineSegmentList>(
    "line_segments", 10);

  if (pub_markers_) {
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "line_markers", 10);
  }

  // Create subscriber
  scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&LineExtractionROS::laserScanCallback, this, std::placeholders::_1));

  // Create timer for periodic processing
  auto period = std::chrono::duration<double>(1.0 / frequency_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&LineExtractionROS::run, this));

  // Register parameter change callback for dynamic updates
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&LineExtractionROS::onParameterChange, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Line extraction node started");
}

LineExtractionROS::~LineExtractionROS()
{
}

///////////////////////////////////////////////////////////////////////////////
// Declare ROS2 parameters
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::declareParameters()
{
  // Parameters used by this node
  this->declare_parameter<std::string>("frame_id", "laser");
  this->declare_parameter<std::string>("scan_topic", "scan");
  this->declare_parameter<bool>("publish_markers", false);
  this->declare_parameter<double>("frequency", 25.0);

  // Parameters used by the line extraction algorithm
  this->declare_parameter<double>("bearing_std_dev", 1e-3);
  this->declare_parameter<double>("range_std_dev", 0.02);
  this->declare_parameter<double>("least_sq_angle_thresh", 1e-4);
  this->declare_parameter<double>("least_sq_radius_thresh", 1e-4);
  this->declare_parameter<double>("max_line_gap", 0.4);
  this->declare_parameter<double>("min_line_length", 0.5);
  this->declare_parameter<double>("min_range", 0.4);
  this->declare_parameter<double>("max_range", 10000.0);
  this->declare_parameter<double>("min_split_dist", 0.05);
  this->declare_parameter<double>("outlier_dist", 0.05);
  this->declare_parameter<int>("min_line_points", 9);
}

///////////////////////////////////////////////////////////////////////////////
// Load ROS parameters
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::loadParameters()
{
  RCLCPP_DEBUG(this->get_logger(), "*************************************");
  RCLCPP_DEBUG(this->get_logger(), "PARAMETERS:");

  // Parameters used by this node
  frame_id_ = this->get_parameter("frame_id").as_string();
  RCLCPP_DEBUG(this->get_logger(), "frame_id: %s", frame_id_.c_str());

  scan_topic_ = this->get_parameter("scan_topic").as_string();
  RCLCPP_DEBUG(this->get_logger(), "scan_topic: %s", scan_topic_.c_str());

  pub_markers_ = this->get_parameter("publish_markers").as_bool();
  RCLCPP_DEBUG(this->get_logger(), "publish_markers: %s", pub_markers_ ? "true" : "false");

  frequency_ = this->get_parameter("frequency").as_double();
  RCLCPP_DEBUG(this->get_logger(), "frequency: %f", frequency_);

  // Parameters used by the line extraction algorithm
  double bearing_std_dev = this->get_parameter("bearing_std_dev").as_double();
  line_extraction_.setBearingVariance(bearing_std_dev * bearing_std_dev);
  RCLCPP_DEBUG(this->get_logger(), "bearing_std_dev: %f", bearing_std_dev);

  double range_std_dev = this->get_parameter("range_std_dev").as_double();
  line_extraction_.setRangeVariance(range_std_dev * range_std_dev);
  RCLCPP_DEBUG(this->get_logger(), "range_std_dev: %f", range_std_dev);

  double least_sq_angle_thresh = this->get_parameter("least_sq_angle_thresh").as_double();
  line_extraction_.setLeastSqAngleThresh(least_sq_angle_thresh);
  RCLCPP_DEBUG(this->get_logger(), "least_sq_angle_thresh: %f", least_sq_angle_thresh);

  double least_sq_radius_thresh = this->get_parameter("least_sq_radius_thresh").as_double();
  line_extraction_.setLeastSqRadiusThresh(least_sq_radius_thresh);
  RCLCPP_DEBUG(this->get_logger(), "least_sq_radius_thresh: %f", least_sq_radius_thresh);

  double max_line_gap = this->get_parameter("max_line_gap").as_double();
  line_extraction_.setMaxLineGap(max_line_gap);
  RCLCPP_DEBUG(this->get_logger(), "max_line_gap: %f", max_line_gap);

  double min_line_length = this->get_parameter("min_line_length").as_double();
  line_extraction_.setMinLineLength(min_line_length);
  RCLCPP_DEBUG(this->get_logger(), "min_line_length: %f", min_line_length);

  double min_range = this->get_parameter("min_range").as_double();
  line_extraction_.setMinRange(min_range);
  RCLCPP_DEBUG(this->get_logger(), "min_range: %f", min_range);

  double max_range = this->get_parameter("max_range").as_double();
  line_extraction_.setMaxRange(max_range);
  RCLCPP_DEBUG(this->get_logger(), "max_range: %f", max_range);

  double min_split_dist = this->get_parameter("min_split_dist").as_double();
  line_extraction_.setMinSplitDist(min_split_dist);
  RCLCPP_DEBUG(this->get_logger(), "min_split_dist: %f", min_split_dist);

  double outlier_dist = this->get_parameter("outlier_dist").as_double();
  line_extraction_.setOutlierDist(outlier_dist);
  RCLCPP_DEBUG(this->get_logger(), "outlier_dist: %f", outlier_dist);

  int min_line_points = this->get_parameter("min_line_points").as_int();
  line_extraction_.setMinLinePoints(static_cast<unsigned int>(min_line_points));
  RCLCPP_DEBUG(this->get_logger(), "min_line_points: %d", min_line_points);

  RCLCPP_DEBUG(this->get_logger(), "*************************************");
}

///////////////////////////////////////////////////////////////////////////////
// Update algorithm parameters (called on dynamic parameter change)
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::updateAlgorithmParameters()
{
  double bearing_std_dev = this->get_parameter("bearing_std_dev").as_double();
  line_extraction_.setBearingVariance(bearing_std_dev * bearing_std_dev);

  double range_std_dev = this->get_parameter("range_std_dev").as_double();
  line_extraction_.setRangeVariance(range_std_dev * range_std_dev);

  line_extraction_.setLeastSqAngleThresh(
    this->get_parameter("least_sq_angle_thresh").as_double());
  line_extraction_.setLeastSqRadiusThresh(
    this->get_parameter("least_sq_radius_thresh").as_double());
  line_extraction_.setMaxLineGap(this->get_parameter("max_line_gap").as_double());
  line_extraction_.setMinLineLength(this->get_parameter("min_line_length").as_double());
  line_extraction_.setMinRange(this->get_parameter("min_range").as_double());
  line_extraction_.setMaxRange(this->get_parameter("max_range").as_double());
  line_extraction_.setMinSplitDist(this->get_parameter("min_split_dist").as_double());
  line_extraction_.setOutlierDist(this->get_parameter("outlier_dist").as_double());
  line_extraction_.setMinLinePoints(
    static_cast<unsigned int>(this->get_parameter("min_line_points").as_int()));
}

///////////////////////////////////////////////////////////////////////////////
// Dynamic parameter change callback
///////////////////////////////////////////////////////////////////////////////
rcl_interfaces::msg::SetParametersResult LineExtractionROS::onParameterChange(
    const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    const std::string & name = param.get_name();

    // Parameters that can be updated dynamically
    if (name == "frame_id") {
      frame_id_ = param.as_string();
      RCLCPP_INFO(this->get_logger(), "Updated frame_id to: %s", frame_id_.c_str());
    }
    else if (name == "bearing_std_dev" || name == "range_std_dev" ||
             name == "least_sq_angle_thresh" || name == "least_sq_radius_thresh" ||
             name == "max_line_gap" || name == "min_line_length" ||
             name == "min_range" || name == "max_range" ||
             name == "min_split_dist" || name == "outlier_dist" ||
             name == "min_line_points") {
      // These will be applied after the loop
      RCLCPP_INFO(this->get_logger(), "Updated parameter: %s", name.c_str());
    }
    // Parameters that require restart to take effect
    else if (name == "scan_topic" || name == "frequency" || name == "publish_markers") {
      RCLCPP_WARN(this->get_logger(),
        "Parameter '%s' change requires node restart to take effect", name.c_str());
    }
  }

  // Update algorithm parameters after processing all changes
  updateAlgorithmParameters();

  return result;
}

///////////////////////////////////////////////////////////////////////////////
// Run
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::run()
{
  // Extract the lines
  std::vector<Line> lines;
  line_extraction_.extractLines(lines);

  // Populate message
  laser_line_extraction::msg::LineSegmentList msg;
  populateLineSegListMsg(lines, msg);

  // Publish the lines
  line_publisher_->publish(msg);

  // Also publish markers if parameter publish_markers is set to true
  if (pub_markers_) {
    visualization_msgs::msg::Marker marker_msg;
    populateMarkerMsg(lines, marker_msg);
    marker_publisher_->publish(marker_msg);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Populate messages
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::populateLineSegListMsg(const std::vector<Line> &lines,
                                                laser_line_extraction::msg::LineSegmentList &line_list_msg)
{
  for (const auto& line : lines) {
    laser_line_extraction::msg::LineSegment line_msg;
    line_msg.angle = static_cast<float>(line.getAngle());
    line_msg.radius = static_cast<float>(line.getRadius());

    const auto& cov = line.getCovariance();
    for (size_t i = 0; i < 4; ++i) {
      line_msg.covariance[i] = static_cast<float>(cov[i]);
    }

    const auto& start = line.getStart();
    line_msg.start[0] = static_cast<float>(start[0]);
    line_msg.start[1] = static_cast<float>(start[1]);

    const auto& end = line.getEnd();
    line_msg.end[0] = static_cast<float>(end[0]);
    line_msg.end[1] = static_cast<float>(end[1]);

    line_list_msg.line_segments.push_back(line_msg);
  }
  line_list_msg.header.frame_id = frame_id_;
  line_list_msg.header.stamp = this->now();
}

void LineExtractionROS::populateMarkerMsg(const std::vector<Line> &lines,
                                           visualization_msgs::msg::Marker &marker_msg)
{
  marker_msg.ns = "line_extraction";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_msg.action = visualization_msgs::msg::Marker::ADD;
  marker_msg.scale.x = 0.1;
  marker_msg.color.r = 1.0;
  marker_msg.color.g = 0.0;
  marker_msg.color.b = 0.0;
  marker_msg.color.a = 1.0;

  for (const auto& line : lines) {
    geometry_msgs::msg::Point p_start;
    p_start.x = line.getStart()[0];
    p_start.y = line.getStart()[1];
    p_start.z = 0;
    marker_msg.points.push_back(p_start);

    geometry_msgs::msg::Point p_end;
    p_end.x = line.getEnd()[0];
    p_end.y = line.getEnd()[1];
    p_end.z = 0;
    marker_msg.points.push_back(p_end);
  }
  marker_msg.header.frame_id = frame_id_;
  marker_msg.header.stamp = this->now();
}

///////////////////////////////////////////////////////////////////////////////
// Cache data on first LaserScan message received
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::cacheData(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  std::vector<double> bearings, cos_bearings, sin_bearings;
  std::vector<unsigned int> indices;
  const std::size_t num_measurements = std::ceil(
      (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

  for (std::size_t i = 0; i < num_measurements; ++i) {
    const double b = scan_msg->angle_min + i * scan_msg->angle_increment;
    bearings.push_back(b);
    cos_bearings.push_back(cos(b));
    sin_bearings.push_back(sin(b));
    indices.push_back(static_cast<unsigned int>(i));
  }

  line_extraction_.setCachedData(bearings, cos_bearings, sin_bearings, indices);
  RCLCPP_DEBUG(this->get_logger(), "Data has been cached.");
}

///////////////////////////////////////////////////////////////////////////////
// Main LaserScan callback
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  if (!data_cached_) {
    cacheData(scan_msg);
    data_cached_ = true;
  }

  std::vector<double> scan_ranges_doubles(scan_msg->ranges.begin(), scan_msg->ranges.end());
  line_extraction_.setRangeData(scan_ranges_doubles);
}

} // namespace line_extraction