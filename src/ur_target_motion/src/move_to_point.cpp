#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>

using namespace std::chrono_literals;

class XYZMoveNode : public rclcpp::Node
{
public:
    XYZMoveNode() : Node("xyz_move_node")
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
        move_group_->setMaxVelocityScalingFactor(0.3);
        move_group_->setMaxAccelerationScalingFactor(0.2);
        move_group_->setPoseReferenceFrame("base_link");

        RCLCPP_INFO(get_logger(), "MoveGroup ready");
    }

    void move_to_xyz(double x, double y, double z)
    {
        // ✅ Sync robot state first
        move_group_->setStartStateToCurrentState();
        std::this_thread::sleep_for(300ms);

        // ✅ Read current pose to preserve real orientation
        geometry_msgs::msg::Pose current = move_group_->getCurrentPose().pose;

        // ✅ Validate quaternion is not zeroed out
        double norm = std::sqrt(
            std::pow(current.orientation.x, 2) +
            std::pow(current.orientation.y, 2) +
            std::pow(current.orientation.z, 2) +
            std::pow(current.orientation.w, 2));

        if (norm < 0.01)
        {
            RCLCPP_WARN(get_logger(),
                "getCurrentPose() returned invalid orientation. Using fallback from tf2_echo.");

            // ✅ Fallback orientation from your tf2_echo output:
            // Rotation: in Quaternion (xyzw) [0.708, 0.004, 0.002, 0.706]
            current.orientation.x = 0.708;
            current.orientation.y = 0.004;
            current.orientation.z = 0.002;
            current.orientation.w = 0.706;
        }

        // ✅ Only override XYZ, keep orientation unchanged
        geometry_msgs::msg::Pose target = current;
        target.position.x = x;
        target.position.y = y;
        target.position.z = z;

        RCLCPP_INFO(get_logger(),
            "Planning to x=%.3f y=%.3f z=%.3f | orientation (xyzw): %.3f %.3f %.3f %.3f",
            target.position.x, target.position.y, target.position.z,
            target.orientation.x, target.orientation.y,
            target.orientation.z, target.orientation.w);

        std::vector<geometry_msgs::msg::Pose> waypoints = {target};
        moveit_msgs::msg::RobotTrajectory trajectory;

        double fraction = move_group_->computeCartesianPath(
            waypoints, 0.002, 0.0, trajectory);

        RCLCPP_INFO(get_logger(), "Cartesian path coverage: %.1f%%", fraction * 100.0);

        if (fraction < 0.9)
        {
            RCLCPP_WARN(get_logger(),
                "Could not plan full path (%.1f%%). Try a closer or more reachable target.",
                fraction * 100.0);
            return;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        RCLCPP_INFO(get_logger(), "Executing...");
        move_group_->execute(plan);
        RCLCPP_INFO(get_logger(), "Done.");
    }

    void run_input_loop()
    {
        //  Run in separate thread so rclcpp::spin() stays unblocked
        input_thread_ = std::thread([this]()
        {
            while (rclcpp::ok())
            {
                double x, y, z;
                std::cout << "\nEnter target X Y Z (e.g. 0.35 0.0 0.45): ";
                if (!(std::cin >> x >> y >> z))
                    break;

                RCLCPP_INFO(get_logger(), "Moving to x=%.3f y=%.3f z=%.3f", x, y, z);
                move_to_xyz(x, y, z);
            }
        });
        input_thread_.detach();
    }

    rclcpp::TimerBase::SharedPtr init_timer_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::thread input_thread_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<XYZMoveNode>());
    rclcpp::shutdown();
    return 0;
}
