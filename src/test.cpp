#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

using NavToPose=nav2_msgs::action::NavigateToPose;


rclcpp_action::ClientGoalHandle<NavToPose>::SharedPtr sendGoal(rclcpp::Node::SharedPtr node, rclcpp_action::Client<NavToPose>::SharedPtr client, double x, double y, double z)
{
    NavToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.position.z = z;
    
    rclcpp_action::ClientGoalHandle<NavToPose>::SharedPtr goal_handle{nullptr};
    bool accepted = false;
    while(rclcpp::ok() && !accepted)
    {
        RCLCPP_INFO(node->get_logger(), "Sending..");
        auto future = client->async_send_goal(goal);
        rclcpp::spin_until_future_complete(node, future);

        goal_handle = future.get(); 
        accepted = goal_handle && goal_handle->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED;
        if(goal_handle)
            RCLCPP_INFO(node->get_logger(), "Status: %i, expected 1", goal_handle->get_status());
        else
            RCLCPP_INFO(node->get_logger(), "Future completed but goal_handle is null, trying again");
    }

    return goal_handle;
}

void blockUntilGoalComplete(rclcpp::Node::SharedPtr node, rclcpp_action::Client<NavToPose>::SharedPtr client, rclcpp_action::ClientGoalHandle<NavToPose>::SharedPtr handle)
{
    if(rclcpp::ok())
    {
        RCLCPP_INFO(node->get_logger(), "BLOCKING");

        auto future = client->async_get_result(handle);
        rclcpp::spin_until_future_complete(node, future);
    }   
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("Teeeest");
    
    auto navToPoseClientGiraff = rclcpp_action::create_client<NavToPose>(node, "/giraff/navigate_to_pose");
    auto navToPoseClientRhodon = rclcpp_action::create_client<NavToPose>(node, "/rhodon/navigate_to_pose");
    
    auto reactiveClient = rclcpp_action::create_client<NavToPose>(node, "/Reactive/go_to_pose");

    using namespace std::chrono_literals;
    while(rclcpp::ok() && !navToPoseClientGiraff->wait_for_action_server(5s))
            RCLCPP_INFO(node->get_logger(), "WAITING FOR NAV GIRAFF");
    while(rclcpp::ok() && !navToPoseClientRhodon->wait_for_action_server(5s))
            RCLCPP_INFO(node->get_logger(), "WAITING FOR NAV RHODON");
    while(rclcpp::ok() && !reactiveClient->wait_for_action_server(5s))
            RCLCPP_INFO(node->get_logger(), "WAITING FOR REACTIVE MASTER");


    {
        using namespace std::chrono_literals;
        while(rclcpp::ok() && !navToPoseClientGiraff->wait_for_action_server(1s))
            RCLCPP_INFO(node->get_logger(), "WAITING");
            
        if(!rclcpp::ok())
            return -1;
    }
    
    RCLCPP_WARN(node->get_logger(), "SENDING FIRST GOAL");
    auto goal1 = sendGoal(node, navToPoseClientGiraff, 1.6, 4.7, 0);


    RCLCPP_WARN(node->get_logger(), "SENDING SECOND GOAL");
    sendGoal(node, navToPoseClientRhodon, 1.6, 5.7, 0);

    blockUntilGoalComplete(node, navToPoseClientGiraff, goal1);
    RCLCPP_INFO(node->get_logger(), "GOAL DONE!");
    RCLCPP_INFO(node->get_logger(), "STARTING REACTIVE");

    auto goal_reactive1 = sendGoal(node, reactiveClient, 2.6, 4.7, 0);
    blockUntilGoalComplete(node, reactiveClient, goal_reactive1);
    RCLCPP_INFO(node->get_logger(), "REACTIVE DONE");

    return 0;
}