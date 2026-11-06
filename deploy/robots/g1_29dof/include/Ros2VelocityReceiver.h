#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>

class Ros2VelocityReceiver
{
public:
    Ros2VelocityReceiver()
    {
        if (!rclcpp::ok())
            rclcpp::init(0, nullptr);

        node_ = std::make_shared<rclcpp::Node>(
            "helene_g1_velocity_receiver");

        sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "/helene/g1_cmd_vel", 10,
            [this](geometry_msgs::msg::Twist::SharedPtr msg)
            {
                float vx = msg->linear.x;
                float vy = msg->linear.y;
                float wz = msg->angular.z;

                if (!std::isfinite(vx) ||
                    !std::isfinite(vy) ||
                    !std::isfinite(wz))
                    return;

                std::lock_guard<std::mutex> lock(mutex_);
                command_ = {vx, vy, wz};
                last_received_ = std::chrono::steady_clock::now();
                received_ = true;
            });

        thread_ = std::thread([this]() {
            rclcpp::spin(node_);
        });
    }

    ~Ros2VelocityReceiver()
    {
        if (rclcpp::ok())
            rclcpp::shutdown();

        if (thread_.joinable())
            thread_.join();
    }

    std::array<float, 3> latest()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!received_ ||
            std::chrono::steady_clock::now() - last_received_ >
                std::chrono::milliseconds(500))
            return {0.0f, 0.0f, 0.0f};

        return command_;
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    std::thread thread_;
    std::mutex mutex_;

    std::array<float, 3> command_{0.0f, 0.0f, 0.0f};
    std::chrono::steady_clock::time_point last_received_;
    bool received_ = false;
};
