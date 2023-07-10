#include <robot2023/reactive_follower.h>
#include <ament_imgui/ament_imgui.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/transform_datatypes.h>
#include <robot2023/common.h>
#include <json.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>


int main (int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ReactiveFollower>();

    std::thread renderThread(&ReactiveFollower::render, node);

    rclcpp::Rate rate(10);
    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
        if(node->m_running)
            node->execute();
        rate.sleep();
    }

    renderThread.join();
}

ReactiveFollower::ReactiveFollower() : Node ("Follower"),  tf_buffer(get_clock())
{
    m_linearSpeed = declare_parameter<double>("/follower/linearSpeed", 0.3);
    m_directionTolerance = declare_parameter<double>("/follower/directionTolerance", 0.1);

    directionPID = std::make_unique<PID>(get_clock(), 0.2, 0.2, 0.1);
    speedPID = std::make_unique<PID>(get_clock(), 0.1, 0.1, 0.1);
    m_local_frame_id = declare_parameter<std::string>("/follower/local_frame_id", "");
    m_master_loc_topic = declare_parameter<std::string>("/follower/master_loc_topic", "/rhodon/status");


    cmdPub = create_publisher<Twist>("cmd_vel", 5);
    RCLCPP_INFO(get_logger(), "cmd_vel topic=%s", cmdPub->get_topic_name());

    masterPoseSub = create_subscription<diagnostic_msgs::msg::KeyValue>("/mqtt2ros", 5, std::bind(&ReactiveFollower::mqttCallback, this, std::placeholders::_1));

    float offsetDistance = declare_parameter<float>("/follower/offsetDistance", 1);
    m_master_offset.setOrigin({0, offsetDistance, 0});
}

void ReactiveFollower::execute()
{

    rclcpp::Rate control_rate(20);

    speedPID->reset(0);
    directionPID->reset(0);
    while(rclcpp::ok() && m_running)
    {
        rclcpp::spin_some(shared_from_this());
        updateTFs();

        tf2::Vector3 forward = tf2::quatRotate(m_currentTransform.getRotation(), {1,0,0});
        double directionError = signedDistanceToLine(m_currentTransform.getOrigin(), forward, m_currentTarget);
        double distance = tf2::tf2Distance(m_currentTarget, m_currentTransform.getOrigin());

        Twist twist;
        if(std::abs(directionError) < m_directionTolerance)
            twist.linear.x = speedPID->DoUpdate(distance);
        else
        {
            speedPID->reset(distance);
            twist.linear.x = 0;
        }
            
        twist.angular.z = directionPID->DoUpdate(directionError); //rotate slower as you approach the correct direction

        cmdPub->publish(twist);
        control_rate.sleep();
    }
}


void ReactiveFollower::updateTFs()
{
    try
    {
        auto geo_tf_stamped = tf_buffer.buffer.lookupTransform("map", m_local_frame_id, tf2_ros::fromRclcpp(now()) );
        tf2::fromMsg(geo_tf_stamped.transform, m_currentTransform);
    }
    catch(std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), "%s", e.what());
    }
}

void ReactiveFollower::render()
{
    // may throw ament_index_cpp::PackageNotFoundError exception
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("robot2023");
    AMENT_IMGUI::setup( (package_share_directory+"/imgui.ini").c_str() );
    rclcpp::Rate rate(30);
    while (rclcpp::ok())
    {
        AMENT_IMGUI::StartFrame();  
        
        ImGui::Begin("Direction");
        {
            ImGui::InputFloat("P", &(directionPID->kP));
            ImGui::InputFloat("I", &(directionPID->kI));
            ImGui::InputFloat("D", &(directionPID->kD));
        }
        ImGui::End(); 
        ImGui::Begin("Speed");
        {
            ImGui::InputFloat("P", &(speedPID->kP));
            ImGui::InputFloat("I", &(speedPID->kI));
            ImGui::InputFloat("D", &(speedPID->kD));
        }
        ImGui::End(); 

        AMENT_IMGUI::Render();
    }
    AMENT_IMGUI::close();
}

void ReactiveFollower::mqttCallback(diagnostic_msgs::msg::KeyValue::SharedPtr msg)
{
    if(msg->key==m_master_loc_topic)
    {
        RCLCPP_INFO(get_logger(), "RECEIVED MASTER POSE");

        auto json =  nlohmann::json::parse(msg->value); 
        std::string x_y_yaw_string = json["data"]["pose"].get<std::string>();
        
        tf2::Transform master_tf;
        //parse the string
        {
            std::array<double, 3> x_y_yaw;
            std::stringstream s_stream(x_y_yaw_string);
            int i = 0;
            
            s_stream.ignore(2,'[');
            while(!s_stream.eof())
            {
                //for debugging
                //std::string remaining = (s_stream.str().substr(s_stream.tellg()));
                
                s_stream >> x_y_yaw[i];
                s_stream.ignore(2,' ');
                
                if(s_stream.fail())
                    RCLCPP_ERROR(get_logger(), "ERROR PARSING THE JSON STRING: %s, \n\nPOSE SUBSTRING: %s:", msg->value.c_str(), x_y_yaw_string.c_str());
                i++;   
            }

            master_tf.setOrigin( {x_y_yaw[0], x_y_yaw[1], 0} );
            master_tf.setRotation( tf2::Quaternion({0,0,1}, x_y_yaw[2]) );
        }

        m_currentTarget = (master_tf * m_master_offset).getOrigin();
    }
    else if(msg->key=="/giraff/run")
    {
        RCLCPP_INFO(get_logger(), "STARTING REACTIVE NAVIGATION");
        nlohmann::json json = msg->value; 
        m_running = json["run"].get<bool>();
    }
}
