#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <ament_imgui/ament_imgui.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <PoseJSON.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using KeyValue=diagnostic_msgs::msg::KeyValue;
using NavToPose=nav2_msgs::action::NavigateToPose;


class Test: public rclcpp::Node
{
public:
    rclcpp_action::Client<NavToPose>::SharedPtr navToPoseClient1;
    rclcpp_action::Client<NavToPose>::SharedPtr navToPoseClient2;
    rclcpp::Publisher<KeyValue>::SharedPtr reactiveClient;

    Test() : Node("Test")
    {
        navToPoseClient1 = rclcpp_action::create_client<NavToPose>(this, "/rhodon/nav2MQTT");
        navToPoseClient2 = rclcpp_action::create_client<NavToPose>(this, "/giraff/nav2MQTT");
        
        reactiveClient = create_publisher<KeyValue>("/ros2mqtt", 1);

        using namespace std::chrono_literals;
        while(rclcpp::ok() && !navToPoseClient1->wait_for_action_server(5s))
                RCLCPP_INFO(get_logger(), "WAITING FOR NAV 1");
        while(rclcpp::ok() && !navToPoseClient2->wait_for_action_server(5s))
                RCLCPP_INFO(get_logger(), "WAITING FOR NAV 2");
    }

    rclcpp_action::ClientGoalHandle<NavToPose>::SharedPtr sendGoal(rclcpp_action::Client<NavToPose>::SharedPtr client, double x, double y, double z)
    {
        NavToPose::Goal goal;
        goal.pose.header.frame_id = "map";
        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;
        goal.pose.pose.position.z = z;
        
        goal.pose.pose.orientation = tf2::toMsg(
            tf2::Quaternion({0,0,1}, 0)
        );
        
        rclcpp_action::ClientGoalHandle<NavToPose>::SharedPtr goal_handle{nullptr};
        bool accepted = false;
        while(rclcpp::ok() && !accepted)
        {
            RCLCPP_INFO(get_logger(), "Sending..");
            auto future = client->async_send_goal(goal);
            rclcpp::spin_until_future_complete(shared_from_this(), future);

            goal_handle = future.get(); 
            accepted = goal_handle && goal_handle->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED;
            if(goal_handle)
                RCLCPP_INFO(get_logger(), "Status: %i, expected 1", goal_handle->get_status());
            else
                RCLCPP_INFO(get_logger(), "Future completed but goal_handle is null, trying again");
        }

        return goal_handle;
    }

    void blockUntilGoalComplete(rclcpp_action::Client<NavToPose>::SharedPtr client, rclcpp_action::ClientGoalHandle<NavToPose>::SharedPtr handle)
    {
        if(rclcpp::ok())
        {
            RCLCPP_INFO(get_logger(), "BLOCKING");

            auto future = client->async_get_result(handle);
            rclcpp::spin_until_future_complete(shared_from_this(), future);
        }   
    }

    void render()
    {
        // may throw ament_index_cpp::PackageNotFoundError exception
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("robot2023");
        AMENT_IMGUI::setup( (package_share_directory+"/test_imgui.ini").c_str() );
        rclcpp::Rate rate(30);
        while (rclcpp::ok())
        {
            AMENT_IMGUI::StartFrame();
            
            if(ImGui::Button("Start"))
                run();
            
            if (ImGui::Button("Reactive"))
                reactive();

            rate.sleep();
            AMENT_IMGUI::Render();
        }
        AMENT_IMGUI::close();
    }

    void run()
    {
        RCLCPP_WARN(shared_from_this()->get_logger(), "SENDING FIRST GOAL");
        auto goal1 = sendGoal(navToPoseClient1, -2, 4.5, 0);


        RCLCPP_WARN(shared_from_this()->get_logger(), "SENDING SECOND GOAL");
        sendGoal(navToPoseClient2, -2, 5.5, 0);

        RCLCPP_INFO(get_logger(), "GOAL DONE!");
    }

    void reactive()
    {
        RCLCPP_INFO(get_logger(), "STARTING REACTIVE");
        {
            geometry_msgs::msg::PoseStamped reactiveGoal;
            reactiveGoal.header.frame_id = "map";
            reactiveGoal.pose.position.x = 1;
            reactiveGoal.pose.position.y = 4.5;
            reactiveGoal.pose.position.z = 0;

            KeyValue msg;
            msg.key = "/reactive";
            msg.value = nav2MQTT::to_json(reactiveGoal).dump();
            reactiveClient->publish(msg);
            //blockUntilGoalComplete(reactiveClient, goal_reactive1);
            //RCLCPP_INFO(get_logger(), "REACTIVE DONE");
        }
    }
};


int main(int argc, char** argv)
{
    
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Test>();
    
    node->render();

    return 0;
}