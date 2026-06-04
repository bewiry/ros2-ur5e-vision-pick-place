#include <memory>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("move_to_pose_rpy_node");

  // Declare parameters with default values
  node->declare_parameter("x", 0.30);
  node->declare_parameter("y", 0.00);
  node->declare_parameter("z", 0.40);

  node->declare_parameter("roll", 3.14);
  node->declare_parameter("pitch", 0.0);
  node->declare_parameter("yaw", 0.0);

  // Read parameters
  const double x = node->get_parameter("x").as_double();
  const double y = node->get_parameter("y").as_double();
  const double z = node->get_parameter("z").as_double();

  const double roll = node->get_parameter("roll").as_double();
  const double pitch = node->get_parameter("pitch").as_double();
  const double yaw = node->get_parameter("yaw").as_double();

  RCLCPP_INFO(node->get_logger(), "Target position:");
  RCLCPP_INFO(node->get_logger(), "x = %.3f, y = %.3f, z = %.3f", x, y, z);

  RCLCPP_INFO(node->get_logger(), "Target orientation (RPY in radians):");
  RCLCPP_INFO(node->get_logger(), "roll = %.3f, pitch = %.3f, yaw = %.3f", roll, pitch, yaw);

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface move_group(node, "ur_manipulator");

  // Use base_link as the pose reference frame
  move_group.setPoseReferenceFrame("base_link");

  // Planning settings
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(0.1);

  // Goal tolerances
  move_group.setGoalPositionTolerance(0.01);
  move_group.setGoalOrientationTolerance(0.05);

  // Convert RPY -> quaternion
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;

  target_pose.orientation = tf2::toMsg(q);

  RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());

  RCLCPP_INFO(
    node->get_logger(),
    "Quaternion: x=%.4f y=%.4f z=%.4f w=%.4f",
    target_pose.orientation.x,
    target_pose.orientation.y,
    target_pose.orientation.z,
    target_pose.orientation.w
  );

  move_group.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool success =
    (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    RCLCPP_INFO(node->get_logger(), "Plan successful. Executing...");
    const auto exec_result = move_group.execute(plan);

    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(node->get_logger(), "Execution successful.");
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "Execution failed.");
    }
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning failed.");
  }

  rclcpp::shutdown();
  return 0;
}
