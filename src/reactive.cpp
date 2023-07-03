#include <robot2023/reactive_master.h>
#include <robot2023/reactive_follower.h>
#include <ament_imgui/ament_imgui.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

class Reactive : public rclcpp::Node
{
    public:
    std::shared_ptr<ReactiveMaster> master;


    Reactive() : Node("Reactive")
    {
        using namespace std::placeholders;
        m_actionServer = rclcpp_action::create_server<GoToPose>(this, "/Reactive/go_to_pose", 
            std::bind(&Reactive::handle_goal, this, _1, _2), 
            std::bind(&Reactive::handle_cancel, this, _1), 
            std::bind(&Reactive::handle_accepted, this, _1)
        );
        master = std::make_shared<ReactiveMaster>();
    }


    rclcpp_action::Server<GoToPose>::SharedPtr m_actionServer;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<GoToPose>> m_activeGoal{nullptr};

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GoToPose::Goal> goal)
    {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoToPose>> goal_handle)
    {
        rclcpp::shutdown();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoToPose>> goal_handle)
    {
        m_activeGoal = goal_handle;
    }


    void execute()
    {
        auto handle_master = std::thread(&ReactiveMaster::execute, master, *m_activeGoal->get_goal());
        
        handle_master.join();

        auto result =std::make_shared<GoToPose::Result>();
        m_activeGoal->succeed(result);
        m_activeGoal = {nullptr};
    }

    void render()
    {
        // may throw ament_index_cpp::PackageNotFoundError exception
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("robot2023");
        AMENT_IMGUI::setup( (package_share_directory+"/imgui.ini").c_str() );
        rclcpp::Rate rate(30);
        while (rclcpp::ok())
        {
            AMENT_IMGUI::StartFrame();
            master->render();
            rate.sleep();
            AMENT_IMGUI::Render();
        }
        AMENT_IMGUI::close();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<Reactive> node = std::make_shared<Reactive>();
    
    std::thread renderThread(&Reactive::render, node.get());

    rclcpp::Rate rate(20);
    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);

        if(node->m_activeGoal.get() != nullptr)
            node->execute();
        
        rate.sleep();
    }
    renderThread.join();
}