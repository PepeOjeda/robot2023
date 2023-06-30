#include <robot2023/reactive_follower.h>
#include <ament_imgui/ament_imgui.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/transform_datatypes.h>
#include <robot2023/common.h>

ReactiveFollower::ReactiveFollower() : Node ("Follower"),  tf_buffer(get_clock())
{
    m_linearSpeed = declare_parameter<double>("/follower/linearSpeed", 0.3);
    m_directionTolerance = declare_parameter<double>("/follower/directionTolerance", 0.1);

    pid = std::make_unique<PID>(get_clock(), 0.1, 0.1, 0.1);
    m_local_frame_id = declare_parameter<std::string>("/follower/local_frame_id", "");
    m_master_frame_id = declare_parameter<std::string>("/follower/master_frame_id", "");


    cmdPub = create_publisher<Twist>("cmd_vel", 5);
    RCLCPP_INFO(get_logger(), "cmd_vel topic=%s", cmdPub->get_topic_name());

    stopSub = create_subscription<std_msgs::msg::Bool>("/stop_reactive", 1, std::bind(&ReactiveFollower::stopCB, this, std::placeholders::_1));

    m_master_offset.header.frame_id = "map";
    m_master_offset.point.y = 3;
}

void ReactiveFollower::execute()
{
    m_running = true;
    while(rclcpp::ok() && m_running)
    {
        rclcpp::spin_some(shared_from_this());
        updateTFs();

        tf2::Vector3 forward = tf2::quatRotate(m_currentTransform.getRotation(), {1,0,0});
        double error = signedDistanceToLine(m_currentTransform.getOrigin(), forward, m_currentTarget);
        
        Twist twist;
        if(std::abs(error) < m_directionTolerance)
            twist.linear.x = m_linearSpeed;
        else
            twist.linear.x = 0;
            
        twist.angular.z = pid->DoUpdate(error); //rotate slower as you approach the correct direction

        cmdPub->publish(twist);
    }
}


void ReactiveFollower::updateTFs()
{
    try
    {
        auto geo_tf_stamped = tf_buffer.buffer.lookupTransform("map", m_local_frame_id, tf2_ros::fromRclcpp(now()) );
        tf2::fromMsg(geo_tf_stamped.transform, m_currentTransform);

        auto targetAsPoint = tf_buffer.buffer.transform( m_master_offset, m_master_frame_id);
        m_currentTarget = {targetAsPoint.point.x, targetAsPoint.point.y, targetAsPoint.point.z};
    }
    catch(std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), "%s", e.what());
    }
}

void ReactiveFollower::render()
{
    ImGui::Begin("Follower");
    {
        ImGui::BeginChild("PID");
        {
            ImGui::InputFloat("P", &(pid->kP));
            ImGui::InputFloat("I", &(pid->kI));
            ImGui::InputFloat("D", &(pid->kD));
        }
        ImGui::EndChild();
    }
    ImGui::End(); 
}

void ReactiveFollower::stopCB(const std_msgs::msg::Bool::SharedPtr msg)
{
    m_running = false;
}
