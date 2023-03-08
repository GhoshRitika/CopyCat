#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
// #include <moveit/core/robot_state.h>
#include "tf2/LinearMath/Quaternion.h"

class FollowMoveit : public rclcpp::Node
{
  public:
  FollowMoveit();

  /// Move group interface for the robot
  moveit::planning_interface::MoveGroupInterface move_group_interface;
  /// Subscriber for target pose
  // rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  /// Target pose that is used to detect changes
  geometry_msgs::msg::Pose previous_target_pose_;

  private:
    // void angle_callback(const std_msgs::msg::Float64MultiArray &msg);
    void target_pose_callback(const geometry_msgs::msg::PoseStamped &msg);

};

FollowMoveit::FollowMoveit(): Node("hello_moveit"),
                              move_group_interface(std::shared_ptr<rclcpp::Node>(std::move(this)), "middle")
{
  // rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<HelloMoveit>());
  // auto const node = std::make_shared<rclcpp::Node>(
  //   "hello_moveit",
  //   rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  // );
  // move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "middle");
  // joint_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("Joint_angles", 10, std::bind(&FollowMoveit::angle_callback, this, std::placeholders::_1));
  target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose", 10, std::bind(&FollowMoveit::target_pose_callback, this, std::placeholders::_1));
}

void FollowMoveit::target_pose_callback(const geometry_msgs::msg::PoseStamped &msg){
  // Return if target pose is unchanged
  if (msg.pose == previous_target_pose_)
  {
    return;
  }

  RCLCPP_INFO(get_logger(), "Target pose has changed. Planning and executing...");

  // Plan and execute motion
  move_group_interface.setPoseTarget(msg.pose);
  move_group_interface.move();

  // Update for next callback
  previous_target_pose_ = msg.pose;
}
  // void FollowMoveit::angle_callback(const std_msgs::msg::Float64MultiArray &msg){
  //   // using moveit::planning_interface::MoveGroupInterface;
  //   // auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "middle");
      
  //   move_group_interface.setStartState(*move_group_interface.getCurrentState());
  //   // Set a target Pose
  //   auto const target_pose = []{
  //   geometry_msgs::msg::Pose pose;
  //   pose.orientation.w = 1.0;
  //   pose.position.x = msg[2][2];
  //   pose.position.y = msg[2][0];
  //   pose.position.z = msg[2][1];
  //   return pose;
  //   }();
  //   move_group_interface.setPoseTarget(target_pose);
  //   move_group_interface.setPlanningTime(10.0);
  //   move_group_interface.move();
  // }
    
    // moveit::planning_interface::MoveGroupInterface move_group_interface;

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);

  auto target_follower = std::make_shared<FollowMoveit>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(target_follower);
  executor.spin();
  // rclcpp::spin(std::make_shared<HelloMoveit>());
  // auto const node = std::make_shared<rclcpp::Node>(
    // "hello_moveit",
  //   rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  // );

  // Create a ROS logger
  // auto const logger = rclcpp::get_logger("hello_moveit");

    // Next step goes here
    // Create the MoveIt MoveGroup Interface
    // using moveit::planning_interface::MoveGroupInterface;
    // auto move_group_interface = MoveGroupInterface(node, "middle");
    // // const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup("middle");
    // // moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
    // // geometry_msgs::msg::Pose start_pose2;
    // // start_pose2.orientation.w = 1.0;
    // // start_pose2.orientation.x = 0.0;
    // // start_pose2.orientation.y = 0.0;
    // // start_pose2.orientation.z = 0.0;
    // // start_pose2.position.x = 0.0;
    // // start_pose2.position.y = 0.0;
    // // start_pose2.position.z = 0.1095;
    // // start_state.setFromIK(joint_model_group, start_pose2);
    // // move_group_interface.setStartState(start_state);
    

    // // Set a target Pose
    // auto const target_pose = []{
    // geometry_msgs::msg::Pose msg;
    // // tf2::Quaternion theta;
    // // theta.setRPY(0, 0, 1.57);
    // msg.orientation.w = 1.0;
    // // msg.orientation.x = theta.x();
    // // msg.orientation.y = theta.y();
    // // msg.orientation.z = theta.z();
    // msg.position.x = 0.0384;
    // msg.position.y = 0.0;
    // msg.position.z = 0.0711;
    // // msg.orientation.w = 1.0;
    // // msg.orientation.x = 0.0;
    // // msg.orientation.y = 0.0;
    // // msg.orientation.z = 0.0;
    // // msg.position.x = 0.0;
    // // msg.position.y = 0.0;
    // // msg.position.z = 0.1095;
    // return msg;
    // }();
    // move_group_interface.setPoseTarget(target_pose);
    // // auto const current_pose_index =  move_group_interface.getEndEffectorLink();
    // // RCLCPP_INFO_STREAM(logger, "current" << current_pose_index);
    // move_group_interface.setPlanningTime(10.0);
    // move_group_interface.move();
    //Create a plan to that target pose
    // auto const [success, plan] = [&move_group_interface]{
    // moveit::planning_interface::MoveGroupInterface::Plan msg;
    // auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    // return std::make_pair(ok, msg);
    // }();

    // // // Execute the plan
    // if(success) {
    // move_group_interface.execute(plan);
    // } else {
    // RCLCPP_ERROR(logger, "Planing failed!");
    // }
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}