#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>

using namespace std::chrono_literals;

class CartesianPickPlaceNode : public rclcpp::Node
{
public:
    CartesianPickPlaceNode() : Node("cartesian_pick_place_node")
    {
        init_timer_ = create_wall_timer(
            100ms,
            [this]()
            {
                init_timer_->cancel();
                initialize_move_group();
                run_input_loop();
            });
    }

private:
    void initialize_move_group()
    {
        move_group_ =
            std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), "ur_manipulator");

        move_group_->setPlanningTime(15.0);
        move_group_->setMaxVelocityScalingFactor(0.2);
        move_group_->setMaxAccelerationScalingFactor(0.2);
        move_group_->setPoseReferenceFrame("base_link");

        RCLCPP_INFO(get_logger(), "MoveGroup ready");
    }

    geometry_msgs::msg::Quaternion get_current_orientation()
    {
        geometry_msgs::msg::Pose current = move_group_->getCurrentPose().pose;

        double norm = std::sqrt(
            std::pow(current.orientation.x, 2) +
            std::pow(current.orientation.y, 2) +
            std::pow(current.orientation.z, 2) +
            std::pow(current.orientation.w, 2));

        if (norm < 0.01)
        {
            RCLCPP_WARN(get_logger(), "Invalid orientation. Using fallback orientation.");

            current.orientation.x = 0.708;
            current.orientation.y = 0.004;
            current.orientation.z = 0.002;
            current.orientation.w = 0.706;
        }

        return current.orientation;
    }

    bool move_to_pose_plan(const geometry_msgs::msg::Pose & target)
    {
        move_group_->setStartStateToCurrentState();
        std::this_thread::sleep_for(300ms);

        move_group_->setPoseTarget(target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success =
            (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success)
        {
            RCLCPP_ERROR(get_logger(), "Normal planning failed.");
            return false;
        }

        auto result = move_group_->execute(plan);

        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "Normal execution failed.");
            return false;
        }

        RCLCPP_INFO(get_logger(), "Normal motion done.");
        return true;
    }

    bool move_cartesian_to_pose(const geometry_msgs::msg::Pose & target)
    {
        move_group_->setStartStateToCurrentState();
        std::this_thread::sleep_for(300ms);

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target);

        moveit_msgs::msg::RobotTrajectory trajectory;

        double fraction = move_group_->computeCartesianPath(
            waypoints,
            0.002,
            0.0,
            trajectory);

        RCLCPP_INFO(get_logger(), "Cartesian path coverage: %.1f%%", fraction * 100.0);

        if (fraction < 0.9)
        {
            RCLCPP_WARN(get_logger(), "Cartesian path failed.");
            return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        auto result = move_group_->execute(plan);

        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "Cartesian execution failed.");
            return false;
        }

        RCLCPP_INFO(get_logger(), "Cartesian motion done.");
        return true;
    }

    void open_gripper()
    {
        RCLCPP_INFO(get_logger(), "OPEN GRIPPER");
        std::this_thread::sleep_for(1s);
    }

    void close_gripper()
    {
        RCLCPP_INFO(get_logger(), "CLOSE GRIPPER");
        std::this_thread::sleep_for(1s);
    }

    void pick_place_sequence(
        double pick_x, double pick_y, double pick_z,
        double place_x, double place_y, double place_z)
    {
        double approach_height = 0.15;

        auto orientation = get_current_orientation();

        geometry_msgs::msg::Pose pick_above;
        pick_above.position.x = pick_x;
        pick_above.position.y = pick_y;
        pick_above.position.z = pick_z + approach_height;
        pick_above.orientation = orientation;

        geometry_msgs::msg::Pose pick_pose;
        pick_pose.position.x = pick_x;
        pick_pose.position.y = pick_y;
        pick_pose.position.z = pick_z;
        pick_pose.orientation = orientation;

        geometry_msgs::msg::Pose place_above;
        place_above.position.x = place_x;
        place_above.position.y = place_y;
        place_above.position.z = place_z + approach_height;
        place_above.orientation = orientation;

        geometry_msgs::msg::Pose place_pose;
        place_pose.position.x = place_x;
        place_pose.position.y = place_y;
        place_pose.position.z = place_z;
        place_pose.orientation = orientation;

        RCLCPP_INFO(get_logger(), "Starting pick and place sequence");

        open_gripper();

        RCLCPP_INFO(get_logger(), "1) Moving above box");
        if (!move_to_pose_plan(pick_above)) return;

        RCLCPP_INFO(get_logger(), "2) Moving straight down to box");
        if (!move_cartesian_to_pose(pick_pose)) return;

        RCLCPP_INFO(get_logger(), "3) Closing gripper");
        close_gripper();

        RCLCPP_INFO(get_logger(), "4) Moving straight up");
        if (!move_cartesian_to_pose(pick_above)) return;

        RCLCPP_INFO(get_logger(), "5) Moving above place point");
        if (!move_to_pose_plan(place_above)) return;

        RCLCPP_INFO(get_logger(), "6) Moving straight down to place point");
        if (!move_cartesian_to_pose(place_pose)) return;

        RCLCPP_INFO(get_logger(), "7) Opening gripper");
        open_gripper();

        RCLCPP_INFO(get_logger(), "8) Moving straight up");
        if (!move_cartesian_to_pose(place_above)) return;

        RCLCPP_INFO(get_logger(), "Pick and place finished");
    }

    void run_input_loop()
    {
        input_thread_ = std::thread([this]()
        {
            while (rclcpp::ok())
            {
                double pick_x, pick_y, pick_z;
                double place_x, place_y, place_z;

                std::cout << "\nEnter pick X Y Z: ";
                if (!(std::cin >> pick_x >> pick_y >> pick_z))
                    break;

                std::cout << "Enter place X Y Z: ";
                if (!(std::cin >> place_x >> place_y >> place_z))
                    break;

                pick_place_sequence(
                    pick_x, pick_y, pick_z,
                    place_x, place_y, place_z);
            }
        });

        input_thread_.detach();
    }

    rclcpp::TimerBase::SharedPtr init_timer_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::thread input_thread_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CartesianPickPlaceNode>());
    rclcpp::shutdown();
    return 0;
}
