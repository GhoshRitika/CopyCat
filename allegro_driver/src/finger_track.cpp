#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multiArray.h"
#include <opencv2/opencv.hpp>
#include "media"

using namespace std::chrono_literals;

class FingerTrack : public rclcpp::Node
{
  public:
    AllegroDriver()
    : Node("finger_track"), count_(0.0)
    {

        timestep_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
        marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);
    }
}