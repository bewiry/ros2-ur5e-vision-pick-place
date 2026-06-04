#include <deque>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

class ConveyorSpeedEstimator : public rclcpp::Node
{
public:
    ConveyorSpeedEstimator() : Node("conveyor_speed_estimator")
    {
        declare_parameter("target_x", 70.0);
        declare_parameter("window_size", 10);
        declare_parameter("min_dt", 0.05);

        target_x_ = get_parameter("target_x").as_double();
        window_size_ = get_parameter("window_size").as_int();
        min_dt_ = get_parameter("min_dt").as_double();

        sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/conveyor/object_position",
            10,
            std::bind(&ConveyorSpeedEstimator::callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(get_logger(), "Conveyor speed estimator started.");
        RCLCPP_INFO(get_logger(), "Fixed picking x = %.2f", target_x_);
    }

private:
    struct Sample
    {
        double t;
        double x;
        double y;
    };

    void callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
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

        samples_.push_back({t, x, y});

        while (static_cast<int>(samples_.size()) > window_size_)
        {
            samples_.pop_front();
        }

        if (samples_.size() < 2)
        {
            RCLCPP_INFO(get_logger(), "Waiting for more samples...");
            return;
        }

        const auto & first = samples_.front();
        const auto & last = samples_.back();

        double dt = last.t - first.t;

        if (dt < min_dt_)
        {
            return;
        }

        double vx = (last.x - first.x) / dt;
        double vy = (last.y - first.y) / dt;

        double speed = std::sqrt(vx * vx + vy * vy);

        RCLCPP_INFO(
            get_logger(),
            "Current camera point: x=%.2f y=%.2f | vx=%.3f units/s vy=%.3f units/s | speed=%.3f units/s",
            x, y, vx, vy, speed
        );

        if (std::abs(vx) < 0.001)
        {
            RCLCPP_WARN(get_logger(), "vx is almost zero. Cannot estimate arrival time.");
            return;
        }

        double time_to_pick = (target_x_ - x) / vx;

        if (time_to_pick < 0.0)
        {
            RCLCPP_WARN(
                get_logger(),
                "Box is moving away from x_pick or already passed it. time_to_pick=%.3f s",
                time_to_pick
            );
            return;
        }

        double predicted_y = y + vy * time_to_pick;

        RCLCPP_INFO(
            get_logger(),
            "Estimated arrival to x=%.2f in %.3f s | predicted_y=%.2f",
            target_x_,
            time_to_pick,
            predicted_y
        );
    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;

    std::deque<Sample> samples_;

    double target_x_;
    int window_size_;
    double min_dt_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConveyorSpeedEstimator>());
    rclcpp::shutdown();
    return 0;
}
