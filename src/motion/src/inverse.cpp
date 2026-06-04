#include <memory>
#include <future>
#include <chrono>
#include <thread>
#include <cmath>
#include <mutex>
#include <deque>
#include <string>
#include <algorithm>
#include <cctype>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <robotiq_2f_urcap_adapter/action/gripper_command.hpp>

using namespace std::chrono_literals;

class VisionMovingConveyorPickNode : public rclcpp::Node
{
public:
    using GripperCommand = robotiq_2f_urcap_adapter::action::GripperCommand;

    VisionMovingConveyorPickNode() : Node("vision_moving_conveyor_pick_node")
    {
        declare_parameter("x_pick_camera", 70.0);
        declare_parameter("x_tolerance", 12.0);
        declare_parameter("min_speed", 0.05);
        declare_parameter("min_time_to_pick", 0.8);
        declare_parameter("max_time_to_pick", 20.0);
        declare_parameter("window_size", 6);

        declare_parameter("approach_z", 1.20);
        declare_parameter("pick_z", 1.00);

        // Green uses the current placing position
        declare_parameter("place_x", -0.152);
        declare_parameter("place_y", 0.147);

        // Red placing position
        declare_parameter("red_place_x", -0.152);
        declare_parameter("red_place_y", 0.347);

        // Blue placing position
        declare_parameter("blue_place_x", -0.152);
        declare_parameter("blue_place_y", -0.053);

        // Same table height for all colors
        declare_parameter("place_approach_z", 1.20);
        declare_parameter("place_z", 0.92);

        x_pick_camera_ = get_parameter("x_pick_camera").as_double();
        x_tolerance_ = get_parameter("x_tolerance").as_double();
        min_speed_ = get_parameter("min_speed").as_double();
        min_time_to_pick_ = get_parameter("min_time_to_pick").as_double();
        max_time_to_pick_ = get_parameter("max_time_to_pick").as_double();
        window_size_ = get_parameter("window_size").as_int();

        approach_z_ = get_parameter("approach_z").as_double();
        pick_z_ = get_parameter("pick_z").as_double();

        place_x_ = get_parameter("place_x").as_double();
        place_y_ = get_parameter("place_y").as_double();

        red_place_x_ = get_parameter("red_place_x").as_double();
        red_place_y_ = get_parameter("red_place_y").as_double();

        blue_place_x_ = get_parameter("blue_place_x").as_double();
        blue_place_y_ = get_parameter("blue_place_y").as_double();

        place_approach_z_ = get_parameter("place_approach_z").as_double();
        place_z_ = get_parameter("place_z").as_double();

        gripper_client_ = rclcpp_action::create_client<GripperCommand>(
            this,
            "/robotiq_2f_urcap_adapter/gripper_command"
        );

        vision_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/conveyor/object_position",
            10,
            std::bind(&VisionMovingConveyorPickNode::vision_callback, this, std::placeholders::_1)
        );

        init_timer_ = create_wall_timer(
            100ms,
            [this]()
            {
                init_timer_->cancel();
                initialize_move_group();

                std::thread([this]()
                {
                    run_loop();
                }).detach();
            });

        RCLCPP_INFO(get_logger(), "Moving conveyor pick node started.");
        RCLCPP_INFO(get_logger(), "Fixed camera pick x = %.2f", x_pick_camera_);
        RCLCPP_INFO(get_logger(), "Green place: x=%.3f y=%.3f", place_x_, place_y_);
        RCLCPP_INFO(get_logger(), "Red place:   x=%.3f y=%.3f", red_place_x_, red_place_y_);
        RCLCPP_INFO(get_logger(), "Blue place:  x=%.3f y=%.3f", blue_place_x_, blue_place_y_);
    }

private:
    struct Sample
    {
        double t;
        double x;
        double y;
        std::string color;
    };

    struct Estimate
    {
        bool valid;
        double x;
        double y;
        double vx;
        double vy;
        double time_to_pick;
        double predicted_y;
        std::string color;
    };

    void initialize_move_group()
    {
        move_group_ =
            std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), "ur_manipulator");

        move_group_->setPlanningTime(15.0);
        move_group_->setMaxVelocityScalingFactor(0.3);
        move_group_->setMaxAccelerationScalingFactor(0.2);
        move_group_->setPoseReferenceFrame("base_link");

        RCLCPP_INFO(get_logger(), "MoveGroup ready.");
    }

    void vision_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        double t;

        if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
        {
            t = now().seconds();
        }
        else
        {
            t = rclcpp::Time(msg->header.stamp).seconds();
        }

        double x = msg->point.x;
        double y = msg->point.y;
        std::string color = msg->header.frame_id;

        std::lock_guard<std::mutex> lock(data_mutex_);

        latest_cam_x_ = x;
        latest_cam_y_ = y;
        latest_color_ = color;
        has_target_ = true;

        samples_.push_back({t, x, y, color});

        while (static_cast<int>(samples_.size()) > window_size_)
        {
            samples_.pop_front();
        }
    }

    Estimate get_estimate()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        Estimate est;
        est.valid = false;

        if (!has_target_ || samples_.size() < 2)
        {
            return est;
        }

        const auto & first = samples_.front();
        const auto & last = samples_.back();

        double dt = last.t - first.t;

        if (dt <= 0.05)
        {
            return est;
        }

        double vx = (last.x - first.x) / dt;
        double vy = (last.y - first.y) / dt;

        if (std::abs(vx) < min_speed_)
        {
            return est;
        }

        double time_to_pick = (x_pick_camera_ - last.x) / vx;

        if (time_to_pick <= 0.0)
        {
            return est;
        }

        double predicted_y = last.y + vy * time_to_pick;

        est.valid = true;
        est.x = last.x;
        est.y = last.y;
        est.vx = vx;
        est.vy = vy;
        est.time_to_pick = time_to_pick;
        est.predicted_y = predicted_y;
        est.color = last.color;

        return est;
    }

    void clear_samples()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        samples_.clear();
        has_target_ = false;
        latest_color_.clear();
    }

    void camera_to_robot(double cam_x, double cam_y, double & robot_x, double & robot_y)
    {
        robot_x = 0.00005593 * cam_x + 0.01187338 * cam_y - 0.70538828;
        robot_y = 0.00818009 * cam_x - 0.00046080 * cam_y - 0.52292797;
    }

    std::string to_lower(std::string text)
    {
        std::transform(
            text.begin(),
            text.end(),
            text.begin(),
            [](unsigned char c)
            {
                return static_cast<char>(std::tolower(c));
            }
        );

        return text;
    }

    bool get_place_position_for_color(
        const std::string & color,
        double & target_place_x,
        double & target_place_y)
    {
        std::string c = to_lower(color);

        if (c == "green")
        {
            target_place_x = place_x_;
            target_place_y = place_y_;

            RCLCPP_INFO(
                get_logger(),
                "Detected GREEN box. Using green/current place position."
            );

            return true;
        }

        if (c == "red")
        {
            target_place_x = red_place_x_;
            target_place_y = red_place_y_;

            RCLCPP_INFO(
                get_logger(),
                "Detected RED box. Using red place position."
            );

            return true;
        }

        if (c == "blue")
        {
            target_place_x = blue_place_x_;
            target_place_y = blue_place_y_;

            RCLCPP_INFO(
                get_logger(),
                "Detected BLUE box. Using blue place position."
            );

            return true;
        }

        RCLCPP_WARN(
            get_logger(),
            "Unknown color '%s'. Skipping this box.",
            color.c_str()
        );

        return false;
    }

    bool command_gripper(double position)
    {
        if (!gripper_client_->wait_for_action_server(5s))
        {
            RCLCPP_ERROR(get_logger(), "Gripper action server not available.");
            return false;
        }

        auto goal_msg = GripperCommand::Goal();
        goal_msg.command.position = position;
        goal_msg.command.max_effort = 140.0;
        goal_msg.command.max_speed = 0.15;

        RCLCPP_INFO(get_logger(), "Sending gripper command: %.3f", position);

        auto goal_future = gripper_client_->async_send_goal(goal_msg);

        if (goal_future.wait_for(5s) != std::future_status::ready)
        {
            RCLCPP_ERROR(get_logger(), "Timeout sending gripper goal.");
            return false;
        }

        auto goal_handle = goal_future.get();

        if (!goal_handle)
        {
            RCLCPP_ERROR(get_logger(), "Gripper goal rejected.");
            return false;
        }

        auto result_future = gripper_client_->async_get_result(goal_handle);

        if (result_future.wait_for(10s) != std::future_status::ready)
        {
            RCLCPP_ERROR(get_logger(), "Timeout waiting for gripper result.");
            return false;
        }

        RCLCPP_INFO(get_logger(), "Gripper command done.");
        return true;
    }

    void open_gripper()
    {
        RCLCPP_INFO(get_logger(), "Opening gripper...");
        command_gripper(0.085);
        std::this_thread::sleep_for(50ms);
    }

    void close_gripper()
    {
        RCLCPP_INFO(get_logger(), "Closing gripper...");
        command_gripper(0.035);
        std::this_thread::sleep_for(50ms);
    }

    bool move_to_xyz(double x, double y, double z)
    {
        move_group_->setStartStateToCurrentState();
        std::this_thread::sleep_for(100ms);

        geometry_msgs::msg::Pose current = move_group_->getCurrentPose().pose;

        RCLCPP_INFO(
            get_logger(),
            "Current pose: x=%.3f y=%.3f z=%.3f",
            current.position.x,
            current.position.y,
            current.position.z
        );

        double norm = std::sqrt(
            std::pow(current.orientation.x, 2) +
            std::pow(current.orientation.y, 2) +
            std::pow(current.orientation.z, 2) +
            std::pow(current.orientation.w, 2)
        );

        if (norm < 0.01)
        {
            RCLCPP_WARN(get_logger(), "Invalid orientation detected. Using fallback.");

            current.orientation.x = 1.000;
            current.orientation.y = 0.000;
            current.orientation.z = -0.019;
            current.orientation.w = 0.005;
        }

        geometry_msgs::msg::Pose target = current;
        target.position.x = x;
        target.position.y = y;
        target.position.z = z;

        RCLCPP_INFO(get_logger(), "Planning to x=%.3f y=%.3f z=%.3f", x, y, z);

        std::vector<geometry_msgs::msg::Pose> waypoints = {target};
        moveit_msgs::msg::RobotTrajectory trajectory;

        double fraction = move_group_->computeCartesianPath(
            waypoints,
            0.002,
            0.0,
            trajectory
        );

        RCLCPP_INFO(get_logger(), "Path coverage: %.1f%%", fraction * 100.0);

        if (fraction < 0.9)
        {
            RCLCPP_WARN(
                get_logger(),
                "Path planning failed %.1f%%. Skipping move.",
                fraction * 100.0
            );
            return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        RCLCPP_INFO(get_logger(), "Executing...");
        auto result = move_group_->execute(plan);

        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Execution successful.");
            std::this_thread::sleep_for(250ms);
            return true;
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Execution failed.");
            return false;
        }
    }

    bool wait_until_box_reaches_pick_line(
        double vx_direction,
        double & robot_x,
        double & robot_y)
    {
        RCLCPP_INFO(get_logger(), "Waiting for box to reach camera x=%.2f", x_pick_camera_);

        auto start = std::chrono::steady_clock::now();

        while (rclcpp::ok())
        {
            double x_now;
            double y_now;
            bool has_now;

            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                x_now = latest_cam_x_;
                y_now = latest_cam_y_;
                has_now = has_target_;
            }

            if (!has_now)
            {
                std::this_thread::sleep_for(50ms);
                continue;
            }

            double new_robot_x;
            double new_robot_y;
            camera_to_robot(x_pick_camera_, y_now, new_robot_x, new_robot_y);

            double dx_robot = new_robot_x - robot_x;
            double dy_robot = new_robot_y - robot_y;
            double robot_shift = std::sqrt(dx_robot * dx_robot + dy_robot * dy_robot);

            if (robot_shift > min_speed_)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Box y changed while waiting. Re-adjusting robot: old(%.3f, %.3f) -> new(%.3f, %.3f)",
                    robot_x,
                    robot_y,
                    new_robot_x,
                    new_robot_y
                );

                robot_x = new_robot_x;
                robot_y = new_robot_y;

                if (!move_to_xyz(robot_x, robot_y, approach_z_))
                {
                    return false;
                }
            }

            double error = x_now - x_pick_camera_;

            RCLCPP_INFO(
                get_logger(),
                "Waiting: current camera x=%.2f y=%.2f error=%.2f",
                x_now,
                y_now,
                error
            );

            if (std::abs(error) <= x_tolerance_)
            {
                RCLCPP_INFO(get_logger(), "Box reached pick line.");
                return true;
            }

            if (vx_direction > 0.0 && x_now > x_pick_camera_ + x_tolerance_)
            {
                RCLCPP_WARN(get_logger(), "Box passed pick line.");
                return false;
            }

            if (vx_direction < 0.0 && x_now < x_pick_camera_ - x_tolerance_)
            {
                RCLCPP_WARN(get_logger(), "Box passed pick line.");
                return false;
            }

            auto now_time = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now_time - start).count();

            if (elapsed > max_time_to_pick_)
            {
                RCLCPP_WARN(get_logger(), "Timeout waiting for box to reach pick line.");
                return false;
            }

            std::this_thread::sleep_for(100ms);
        }

        return false;
    }

    void moving_pick_place_sequence(
        double robot_x,
        double robot_y,
        double vx_direction,
        const std::string & color)
    {
        double selected_place_x;
        double selected_place_y;

        if (!get_place_position_for_color(color, selected_place_x, selected_place_y))
        {
            return;
        }

        RCLCPP_INFO(get_logger(), "Starting moving conveyor pick-place sequence.");
        RCLCPP_INFO(get_logger(), "Box color: %s", color.c_str());
        RCLCPP_INFO(get_logger(), "Robot fixed pick point: x=%.3f y=%.3f", robot_x, robot_y);
        RCLCPP_INFO(
            get_logger(),
            "Selected place point: x=%.3f y=%.3f z=%.3f",
            selected_place_x,
            selected_place_y,
            place_z_
        );

        open_gripper();

        if (!move_to_xyz(robot_x, robot_y, approach_z_)) return;

        if (!wait_until_box_reaches_pick_line(vx_direction, robot_x, robot_y)) return;

        if (!move_to_xyz(robot_x, robot_y, pick_z_)) return;

        close_gripper();

        if (!move_to_xyz(robot_x, robot_y, approach_z_)) return;

        if (!move_to_xyz(selected_place_x, selected_place_y, place_approach_z_)) return;
        if (!move_to_xyz(selected_place_x, selected_place_y, place_z_)) return;

        open_gripper();

        if (!move_to_xyz(selected_place_x, selected_place_y, place_approach_z_)) return;

        RCLCPP_INFO(get_logger(), "Moving conveyor pick-place sequence finished.");
    }

    void run_loop()
    {
        std::this_thread::sleep_for(2s);

        while (rclcpp::ok())
        {
            Estimate est = get_estimate();

            if (!est.valid)
            {
                RCLCPP_INFO(get_logger(), "Waiting for valid camera speed estimate...");
                std::this_thread::sleep_for(1s);
                continue;
            }

            RCLCPP_INFO(
                get_logger(),
                "Camera: x=%.2f y=%.2f vx=%.3f vy=%.3f time_to_pick=%.3f predicted_y=%.2f color=%s",
                est.x,
                est.y,
                est.vx,
                est.vy,
                est.time_to_pick,
                est.predicted_y,
                est.color.c_str()
            );

            if (est.time_to_pick < min_time_to_pick_)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Box too close to pick line. time_to_pick=%.3f < %.3f. Waiting for next target.",
                    est.time_to_pick,
                    min_time_to_pick_
                );

                std::this_thread::sleep_for(500ms);
                continue;
            }

            if (est.time_to_pick > max_time_to_pick_)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Box too far from pick line. time_to_pick=%.3f > %.3f.",
                    est.time_to_pick,
                    max_time_to_pick_
                );

                std::this_thread::sleep_for(500ms);
                continue;
            }

            double robot_x;
            double robot_y;

            camera_to_robot(x_pick_camera_, est.predicted_y, robot_x, robot_y);

            RCLCPP_INFO(
                get_logger(),
                "Transformed fixed pick point: cam(%.2f, %.2f) -> robot(%.3f, %.3f)",
                x_pick_camera_,
                est.predicted_y,
                robot_x,
                robot_y
            );

            moving_pick_place_sequence(robot_x, robot_y, est.vx, est.color);

            RCLCPP_INFO(get_logger(), "Cycle done. Clearing samples and waiting for next object...");
            clear_samples();
            std::this_thread::sleep_for(1s);
        }
    }

    rclcpp::TimerBase::SharedPtr init_timer_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr vision_sub_;
    rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;

    std::mutex data_mutex_;
    std::deque<Sample> samples_;

    bool has_target_ = false;
    double latest_cam_x_ = 0.0;
    double latest_cam_y_ = 0.0;
    std::string latest_color_;

    double x_pick_camera_;
    double x_tolerance_;
    double min_speed_;
    double min_time_to_pick_;
    double max_time_to_pick_;
    int window_size_;

    double approach_z_;
    double pick_z_;

    double place_x_;
    double place_y_;

    double red_place_x_;
    double red_place_y_;

    double blue_place_x_;
    double blue_place_y_;

    double place_approach_z_;
    double place_z_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionMovingConveyorPickNode>());
    rclcpp::shutdown();
    return 0;
}
