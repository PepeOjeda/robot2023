#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/transform_datatypes.h>
#include <robot2023/BufferWrapper.h>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <robot2023/PID.h>

using GoToPose = nav2_msgs::action::NavigateToPose;

class ReactiveMaster : public rclcpp::Node
{
    using Twist = geometry_msgs::msg::Twist;
    using Marker = visualization_msgs::msg::Marker;
public:
    ReactiveMaster();
    ~ReactiveMaster();
    void execute(const GoToPose::Goal& goal);
    void render();

private:
    BufferWrapper tf_buffer;
    std::unique_ptr<PID> pid;

    double m_linearSpeed;
    double m_stoppingDistance;
    double m_directionTolerance;
    std::string m_localFrame;
    tf2::Transform m_currentTransform;

    rclcpp::Publisher<Twist>::SharedPtr cmdPub;
    rclcpp::Publisher<Marker>::SharedPtr targetMarkerPub;
    rclcpp::Publisher<Marker>::SharedPtr arrowMarkerPub;

    void updateTFs();

};