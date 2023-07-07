#include <robot2023/reactive_master.h>
#include <robot2023/reactive_follower.h>
#include <ament_imgui/ament_imgui.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <PoseJSON.hpp>

using KeyValue = diagnostic_msgs::msg::KeyValue;

class Reactive : public rclcpp::Node
{
    public:
    std::shared_ptr<ReactiveMaster> master;
    rclcpp::Subscription<KeyValue>::SharedPtr mqttSub;


    Reactive() : Node("Reactive")
    {
        using namespace std::placeholders;
        mqttSub = create_subscription<KeyValue>("/mqtt2ros", 1, std::bind(&Reactive::mqttCallback, this, std::placeholders::_1));
        
        master = std::make_shared<ReactiveMaster>();
    }

    void mqttCallback(KeyValue::SharedPtr msg)
    {
        if(msg->key == "/reactive")
        {
            RCLCPP_INFO(get_logger(), "Reactive goal received, starting");
            auto json = nlohmann::json::parse(msg->value);
            geometry_msgs::msg::PoseStamped pose = nav2MQTT::from_json(json);
            execute(pose);
        }
    }


    void execute(const geometry_msgs::msg::PoseStamped& goal_pose)
    {
        auto handle_master = std::thread(&ReactiveMaster::execute, master, goal_pose);
        handle_master.join();
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
        rate.sleep();
    }
    renderThread.join();
}