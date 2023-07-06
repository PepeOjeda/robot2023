#include <robot2023/reactive_master.h>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ament_imgui/ament_imgui.h>
#include <std_msgs/msg/bool.hpp>
#include <robot2023/common.h>

ReactiveMaster::ReactiveMaster() : Node("Reactive_master"), tf_buffer(get_clock())
{
    m_linearSpeed = declare_parameter<double>("/master/linearSpeed", 0.3);
    m_stoppingDistance = declare_parameter<double>("/master/stoppingDistance", 0.3);
    m_directionTolerance = declare_parameter<double>("/master/directionTolerance", 0.1);

    std::string frame_namespace = get_namespace();
    frame_namespace.erase(0, 1);
    m_localFrame = declare_parameter<std::string>("/master/local_frame", frame_namespace+"_base_link");
    RCLCPP_INFO(get_logger(), "local frame=%s", m_localFrame.c_str());

    cmdPub = create_publisher<Twist>("cmd_vel", 5);
    RCLCPP_INFO(get_logger(), "cmd_vel topic=%s", cmdPub->get_topic_name());

    targetMarkerPub = create_publisher<Marker>("master_target", 5);
    arrowMarkerPub = create_publisher<Marker>("master_forward", 5);

    pid = std::make_unique<PID>(get_clock(), 0.1, 0.1, 0.1);

    runningPub = this->create_publisher<diagnostic_msgs::msg::KeyValue>("ros2mqtt", 5);
}

ReactiveMaster::~ReactiveMaster()
{
    //stop
    Twist twist;
    cmdPub->publish(twist);
}

static geometry_msgs::msg::Point VecToPoint(const tf2::Vector3& vec)
    {
        geometry_msgs::msg::Point p;
        p.x = vec.x();
        p.y = vec.y();
        p.z = vec.z();
        
        return p;
    }


void ReactiveMaster::execute(const GoToPose::Goal& goal)
{
    RCLCPP_INFO(get_logger(), "GOAL RECEIVED, STARTING REACTIVE NAVIGATION");
    
    //tell the follower to start
    {
        diagnostic_msgs::msg::KeyValue msg;
        msg.key = "/giraff/run";
        msg.value= R"({"run": "true"})";
        runningPub->publish(msg);
    }

    tf2::Vector3 target_position; 
    {
        geometry_msgs::msg::PoseStamped goal_map_frame = tf_buffer.buffer.transform(goal.pose, "map");
        tf2::fromMsg(goal_map_frame.pose.position, target_position);
    }

    Marker targetMarker;
    {
        targetMarker.header.frame_id="map";
        targetMarker.action=Marker::ADD;
        targetMarker.id=0;
        targetMarker.type = Marker::SPHERE;

        targetMarker.pose = goal.pose.pose;

        targetMarker.scale.x = 0.1;
        targetMarker.scale.y = 0.1;
        targetMarker.scale.z = 0.1;

        targetMarker.color.r = 1;
        targetMarker.color.g = 0;
        targetMarker.color.b = 0;
        targetMarker.color.a = 1;
        targetMarker.lifetime.sec = 100;
        targetMarkerPub->publish(targetMarker);
    }

    Marker forwardMarker;
    {
        forwardMarker.header.frame_id="map";
        forwardMarker.action=Marker::ADD;
        forwardMarker.id=1;
        forwardMarker.type = Marker::ARROW;


        forwardMarker.scale.x = 0.1;
        forwardMarker.scale.y = 0.3;
        forwardMarker.scale.z = 0.2;

        forwardMarker.color.r = 0;
        forwardMarker.color.g = 1;
        forwardMarker.color.b = 0;
        forwardMarker.color.a = 1;
    }
    auto distance = [this, &target_position]()
    {
        return tf2::tf2Distance(m_currentTransform.getOrigin(), target_position);
    };

    pid->reset(0);

    rclcpp::Rate control_rate(20);
    while(rclcpp::ok() && !(distance()< m_stoppingDistance) )
    {
        rclcpp::spin_some(shared_from_this());
        updateTFs();

        tf2::Vector3 forward = tf2::quatRotate(m_currentTransform.getRotation(), {1,0,0});
        double error = signedDistanceToLine(m_currentTransform.getOrigin(), forward, target_position);
        
        Twist twist;
        if(std::abs(error) < m_directionTolerance)
            twist.linear.x = m_linearSpeed;
        else
            twist.linear.x = 0;
            
        twist.angular.z = pid->DoUpdate(error); //rotate slower as you approach the correct direction

        cmdPub->publish(twist);
        forwardMarker.points.clear();

        //marker
        forwardMarker.points.push_back(VecToPoint(m_currentTransform.getOrigin()));
        forwardMarker.points.push_back(VecToPoint(forward+m_currentTransform.getOrigin()));
        arrowMarkerPub->publish(forwardMarker);
        control_rate.sleep();
    }

    if(!rclcpp::ok())
        return;
    //stop
    Twist twist;
    cmdPub->publish(twist);
    
    //tell the follower to stop
    {
        diagnostic_msgs::msg::KeyValue msg;
        msg.key = "/giraff/run";
        msg.value= R"({"run": "false"})";
        runningPub->publish(msg);
    }
}


void ReactiveMaster::updateTFs()
{
    try
    {
        auto geo_tf_stamped = tf_buffer.buffer.lookupTransform("map", m_localFrame, tf2_ros::fromRclcpp(now()) );
        tf2::fromMsg(geo_tf_stamped.transform, m_currentTransform);
    }
    catch(std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), "%s", e.what());
    }
}



void ReactiveMaster::render()
{
    ImGui::Begin("Master");
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