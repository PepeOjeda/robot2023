#pragma once
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Core>
#include <DDA/DDA.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class MapGenerator : public rclcpp::Node
{

public:
    MapGenerator();
    void readFile();
    void solve();
    void getEnvironment();
    nav_msgs::msg::OccupancyGrid::SharedPtr m_map_msg{nullptr};
private:
    int m_num_cells;
    int m_num_measurements;

    Eigen::VectorXd m_concentration; //(num_cells);
    Eigen::VectorXd m_measurements; //(num_measurements);
    Eigen::MatrixXd m_lengthRayInCell; //(num_measurements, num_cells);

    std::vector<std::vector<bool>> m_occupancy_map;
    float m_rayMarchResolution;
    glm::vec2 m_mapOrigin;
    
    void runDDA(const glm::vec2& origin, const glm::vec2& direction, const glm::vec2& reflectorPosition, uint rowIndex);

    uint index2Dto1D(const glm::ivec2& index)
    {
        int num_cells_x = m_occupancy_map.size();
        int num_cells_y = m_occupancy_map[0].size();
	    return index.x + index.y*num_cells_x;
    }
    
    void mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        m_map_msg = msg;
    }
};


namespace glm
{
    static glm::vec2 fromTF(const tf2::Vector3& v)
    {
        return glm::vec2(v.x(), v.y());
    }
}