#include <memory>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <control_msgs/action/gripper_command.hpp>

using namespace std::chrono_literals;

class PickPlaceNode : public rclcpp::Node
{
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;

  PickPlaceNode() : Node("pick_place_node")
  {
    declare_parameter("pick_x", 0.15);
    declare_parameter("pick_y", -0.20);
    declare_parameter("pick_z", 0.75);

    declare_parameter("place_x", 0.30);
    declare_parameter("place_y", 0.20);
    declare_parameter("place_z", 0.75);

    declare_parameter("roll", 1.6);
    declare_parameter("pitch", 0.0);
    declare_parameter("yaw", 0.0);

    declare_parameter("approach_height", 0.15);

    declare_parameter("gripper_action", "/robotiq_gripper_controller/gripper_cmd");
    declare_parameter("gripper_open", 0.085);
    declare_parameter("gripper_closed", 0.0);
    declare_parameter("gripper_effort", 40.0);

    gripper_action_name_ = get_parameter("gripper_action").as_string();

    gripper_client_ = rclcpp_action::create_client<GripperCommand>(
      this,
      gripper_action_name_
    );
  }

  bool moveToPose(
    moveit::planning_interface::MoveGroupInterface & move_group,
    double x,
    double y,
    double z,
    double roll,
    double pitch,
    double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    target_pose.orientation = tf2::toMsg(q);

    RCLCPP_INFO(get_logger(), "Moving to x=%.3f y=%.3f z=%.3f", x, y, z);

    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success)
    {
      RCLCPP_ERROR(get_logger(), "Planning failed.");
      return false;
    }

    auto result = move_group.execute(plan);

    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Execution failed.");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Motion successful.");
    return true;
  }

  bool commandGripper(double position)
  {
    if (!gripper_client_->wait_for_action_server(3s))
    {
      RCLCPP_ERROR(get_logger(), "Gripper action server not found: %s", gripper_action_name_.c_str());
      return false;
    }

    auto goal_msg = GripperCommand::Goal();
    goal_msg.command.position = position;
    goal_msg.command.max_effort = get_parameter("gripper_effort").as_double();

    RCLCPP_INFO(get_logger(), "Sending gripper command position=%.3f", position);

    auto goal_future = gripper_client_->async_send_goal(goal_msg);

    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to send gripper goal.");
      return false;
    }

    auto goal_handle = goal_future.get();

    if (!goal_handle)
    {
      RCLCPP_ERROR(get_logger(), "Gripper goal was rejected.");
      return false;
    }

    auto result_future = gripper_client_->async_get_result(goal_handle);

    if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to get gripper result.");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Gripper command done.");
    return true;
  }

private:
  std::string gripper_action_name_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PickPlaceNode>();

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface move_group(node, "ur_manipulator");

  move_group.setPoseReferenceFrame("base_link");
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(0.1);
  move_group.setGoalPositionTolerance(0.01);
  move_group.setGoalOrientationTolerance(0.05);

  double pick_x = node->get_parameter("pick_x").as_double();
  double pick_y = node->get_parameter("pick_y").as_double();
  double pick_z = node->get_parameter("pick_z").as_double();

  double place_x = node->get_parameter("place_x").as_double();
  double place_y = node->get_parameter("place_y").as_double();
  double place_z = node->get_parameter("place_z").as_double();

  double roll = node->get_parameter("roll").as_double();
  double pitch = node->get_parameter("pitch").as_double();
  double yaw = node->get_parameter("yaw").as_double();

  double approach_height = node->get_parameter("approach_height").as_double();

  double gripper_open = node->get_parameter("gripper_open").as_double();
  double gripper_closed = node->get_parameter("gripper_closed").as_double();

  RCLCPP_INFO(node->get_logger(), "Starting pick and place sequence.");

  node->commandGripper(gripper_open);
  std::this_thread::sleep_for(1s);

  if (!node->moveToPose(move_group, pick_x, pick_y, pick_z + approach_height, roll, pitch, yaw))
    return 1;

  if (!node->moveToPose(move_group, pick_x, pick_y, pick_z, roll, pitch, yaw))
    return 1;

  node->commandGripper(gripper_closed);
  std::this_thread::sleep_for(1s);

  if (!node->moveToPose(move_group, pick_x, pick_y, pick_z + approach_height, roll, pitch, yaw))
    return 1;

  if (!node->moveToPose(move_group, place_x, place_y, place_z + approach_height, roll, pitch, yaw))
    return 1;

  if (!node->moveToPose(move_group, place_x, place_y, place_z, roll, pitch, yaw))
    return 1;

  node->commandGripper(gripper_open);
  std::this_thread::sleep_for(1s);

  if (!node->moveToPose(move_group, place_x, place_y, place_z + approach_height, roll, pitch, yaw))
    return 1;

  RCLCPP_INFO(node->get_logger(), "Pick and place sequence finished.");

  rclcpp::shutdown();
  return 0;
}
