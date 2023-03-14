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

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);

  auto target_follower = std::make_shared<FollowMoveit>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(target_follower);
  executor.spin();
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}