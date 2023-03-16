#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>


#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;
static const std::string PLANNING_GROUP = "allegro_hand";
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group");

class PlanHand : public rclcpp::Node
{
  public:
    PlanHand();
    moveit::planning_interface::MoveGroupInterface move_group;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_pose_sub_;
    std::vector<double> joint_group_positions;
  

  private:
    void target_pose_callback(const std_msgs::msg::Float64MultiArray &msg);

};

PlanHand::PlanHand(): Node("plan_hand"), move_group(std::shared_ptr<rclcpp::Node>(std::move(this)), PLANNING_GROUP)
{
  RCLCPP_INFO(LOGGER, "INITIALIZING!");
  target_pose_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("Joint_angles", 10, std::bind(&PlanHand::target_pose_callback, this, std::placeholders::_1));
  rclcpp::sleep_for(5s);
  joint_group_positions = {0.0, 0.0, 0.0, 0.0, 0.5235, 0.3665, 0.8901, 0.4886, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  this->move_group.setJointValueTarget(joint_group_positions);
  this->move_group.move();
  rclcpp::sleep_for(10s);
}

void PlanHand::target_pose_callback(const std_msgs::msg::Float64MultiArray &msg){
  joint_group_positions = msg.data;
  RCLCPP_INFO(LOGGER, "Received Joint angles");
  bool within_bounds = this->move_group.setJointValueTarget(joint_group_positions);
  if (!within_bounds)
  {
    RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  }
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);
  this->move_group.move();
}

int main(int argc, char** argv)
{

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  auto move_group_node = std::make_shared<PlanHand>();
  RCLCPP_INFO(LOGGER, "In main");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
