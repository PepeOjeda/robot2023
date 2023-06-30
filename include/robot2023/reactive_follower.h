#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <robot2023/PID.h>
#include <robot2023/BufferWrapper.h>

class ReactiveFollower : public rclcpp::Node
{
    using Twist = geometry_msgs::msg::Twist;
    public:
    ReactiveFollower();
    void execute();
    void render();

    private:
    BufferWrapper tf_buffer;
    rclcpp::Publisher<Twist>::SharedPtr cmdPub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stopSub;

    std::unique_ptr<PID> pid;
    std::string m_master_frame_id;
    std::string m_local_frame_id;
    geometry_msgs::msg::PointStamped m_master_offset;

    double m_linearSpeed;
    double m_directionTolerance;


    tf2::Transform m_currentTransform;
    tf2::Vector3 m_currentTarget;
    void updateTFs();

    bool m_running;
    void stopCB(const std_msgs::msg::Bool::SharedPtr msg);
};