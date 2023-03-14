#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>


#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <geometry_msgs/msg/pose.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"


static const std::string PLANNING_GROUP = "allegro_hand";
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group");

class PlanHand : public rclcpp::Node
{
  public:
    PlanHand();
    moveit::planning_interface::MoveGroupInterface move_group;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_pose_sub_;
    // rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr moving_pub_;


    std::vector<double> joint_group_positions;
  

  private:
    void target_pose_callback(const std_msgs::msg::Float64MultiArray &msg);

};

PlanHand::PlanHand(): Node("plan_hand"), move_group(std::shared_ptr<rclcpp::Node>(std::move(this)), PLANNING_GROUP)
// , move_group_interface(std::shared_ptr<rclcpp::Node>(std::move(this)), "allegro_hand")
{
  RCLCPP_INFO(LOGGER, "INITIALIZING!!!!!!!!");
  // const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  // moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
  // Next get the current set of joint values for the group.
  // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  // moving_pub_= this->create_publisher<std_msgs::msg::Int32>("/moving", 10);
  target_pose_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("Joint_angles", 10, std::bind(&PlanHand::target_pose_callback, this, std::placeholders::_1));
}

void PlanHand::target_pose_callback(const std_msgs::msg::Float64MultiArray &msg){
  joint_group_positions = msg.data;
  RCLCPP_INFO(LOGGER, "Received Joint angles");
  // joint_group_positions[1] = 1.0;  // radians
  bool within_bounds = this->move_group.setJointValueTarget(joint_group_positions);
  if (!within_bounds)
  {
    RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  }
  // auto const [success, plan] = [&move_group]{
  //   moveit::planning_interface::MoveGroupInterface::Plan msg;
  //   auto const ok = static_cast<bool>(move_group.plan(msg));
  //   return std::make_pair(ok, msg);
  //   }();

  //   // // Execute the plan
  //   if(success) {
  //   move_group.execute(plan);
  //   } else {
  //   RCLCPP_ERROR(LOGGER, "Planing failed!");
  //   }
  this->move_group.move();
}

int main(int argc, char** argv)
{

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  // node_options.automatically_declare_parameters_from_overrides(true);
  // auto move_group_node = rclcpp::Node::make_shared("plan_hand", node_options);
  auto move_group_node = std::make_shared<PlanHand>();
  RCLCPP_INFO(LOGGER, "In main");
  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  // std::thread([&executor]() { executor.spin(); }).detach();
  executor.spin();

  rclcpp::shutdown();
  return 0;
}





  // static const std::string PLANNING_GROUP = "allegro_hand";
  // moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // const moveit::core::JointModelGroup* joint_model_group =
  //     move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
  // //
  // // Next get the current set of joint values for the group.
  // std::vector<double> joint_group_positions;
  // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // joint_group_positions[1] = 1.0;  // radians
  // bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
  // if (!within_bounds)
  // {
  //   RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  // }
//   else{
//     move_group.move();
//   }
  // auto const [success, plan] = [&move_group]{
  //   moveit::planning_interface::MoveGroupInterface::Plan msg;
  //   auto const ok = static_cast<bool>(move_group.plan(msg));
  //   return std::make_pair(ok, msg);
  //   }();

  //   // // Execute the plan
  //   if(success) {
  //   move_group.execute(plan);
  //   } else {
  //   RCLCPP_ERROR(LOGGER, "Planing failed!");
  //   }
  // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
  // The default values are 10% (0.1).
  // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
  // or set explicit factors in your code if you need your robot to move faster.
//   move_group.setMaxVelocityScalingFactor(0.05);
//   move_group.setMaxAccelerationScalingFactor(0.05);

//   success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        // success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

