#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <robot2023/PID.h>
#include <robot2023/BufferWrapper.h>
#include <diagnostic_msgs/msg/key_value.hpp>

class ReactiveFollower : public rclcpp::Node
{
    using Twist = geometry_msgs::msg::Twist;
    public:
    ReactiveFollower();
    void execute();
    void render();
    bool m_running;

    private:
    BufferWrapper tf_buffer;
    rclcpp::Publisher<Twist>::SharedPtr cmdPub;

    rclcpp::Subscription<diagnostic_msgs::msg::KeyValue>::SharedPtr masterPoseSub;
    void mqttCallback(diagnostic_msgs::msg::KeyValue::SharedPtr msg);

    std::unique_ptr<PID> directionPID;
    std::unique_ptr<PID> speedPID;
    std::string m_local_frame_id;
    std::string m_master_loc_topic;
    tf2::Transform m_master_offset;

    double m_linearSpeed;
    double m_directionTolerance;


    tf2::Transform m_currentTransform;
    tf2::Vector3 m_currentTarget;
    void updateTFs();

};