#include <robot2023/generate_map.h>
#include <fstream>
#include <PoseJSON.hpp>
#include <robot2023/common.h>
#include <olfaction_msgs/msg/tdlas.hpp>
#include <tf2/LinearMath/Transform.h>
#include <eigen3/Eigen/SVD>

using PoseStamped = geometry_msgs::msg::PoseStamped;
using TDLAS=olfaction_msgs::msg::TDLAS;
using namespace std::chrono_literals;

static TDLAS jsonToTDLAS(const nlohmann::json& json)
{
    TDLAS tdlas;
    tdlas.average_ppmxm = json["average_ppmxm"].get<double>();
    tdlas.average_absorption_strength = json["average_absorption_strength"].get<double>();
    tdlas.average_reflection_strength = json["average_reflection_strength"].get<double>();
    return tdlas;
}

MapGenerator::MapGenerator() : Node("MapGenerator")
{
}

void MapGenerator::readFile()
{
    std::string filepath = declare_parameter<std::string>("filepath", "measurement_log");
    std::ifstream file(filepath);
    
    //Count the number of entries and size the matrices accordingly
    int numberOfMeasurements=0;
    for(std::string line; std::getline(file, line);)
        numberOfMeasurements++;
    
    m_measurements.resize(numberOfMeasurements);
    m_lengthRayInCell.resize(numberOfMeasurements, m_num_cells);
    m_concentration.resize(m_num_cells);
    
    //restart the ifstream
    file.clear();
    file.seekg(0, std::ios::beg);
    
    int measurementIndex = 0;
    for(std::string line; std::getline(file, line); measurementIndex++)
    {
        auto json = nlohmann::json::parse(line);
        tf2::Transform rhodon = poseToTransform( nav2MQTT::from_json(json["rhodon"]).pose );
        tf2::Transform giraff = poseToTransform( nav2MQTT::from_json(json["giraff"]).pose );
        TDLAS tdlas = jsonToTDLAS(json["reading"]);

        m_measurements[measurementIndex] = tdlas.average_ppmxm;

        //fill in the cell raytracing thing
        glm::vec2 rayOrigin = glm::fromTF( rhodon.getOrigin() );
        glm::vec2 rayDirection = glm::fromTF( tf2::quatRotate(rhodon.getRotation(), {0,1,0}) ); //TODO check this
        glm::vec2 reflectorPosition = glm::fromTF( giraff.getOrigin() );

        runDDA(rayOrigin, rayDirection, reflectorPosition, measurementIndex);
    }
}

void MapGenerator::solve()
{
    m_concentration = m_lengthRayInCell.bdcSvd((Eigen::ComputeThinU | Eigen::ComputeThinV)).solve(m_measurements);
}



void MapGenerator::getEnvironment()
{    
    double resoultionRatio = m_map_msg->info.resolution / m_rayMarchResolution;
    int num_cells_x = m_map_msg->info.width * resoultionRatio;
    int num_cells_y = m_map_msg->info.height * resoultionRatio;
    m_num_cells = num_cells_x * num_cells_y;
    
    m_mapOrigin.x = m_map_msg->info.origin.position.x;
    m_mapOrigin.y = m_map_msg->info.origin.position.y;
    
    m_occupancy_map.resize(num_cells_x, std::vector<bool>(num_cells_y ) );


    auto cellFree = [this, resoultionRatio](int argI, int argJ)
    {
        int nx = m_map_msg->info.width;
        bool cellIsFree = true;
        for(int i = argI/resoultionRatio; i<(argI+1)/resoultionRatio; i++)
        {
            for(int j = argJ/resoultionRatio; j<(argJ+1)/resoultionRatio; j++)
            {
                int value = m_map_msg->data[i + j*nx];
                cellIsFree = cellIsFree &&  value == 0;
            }
        }
        return cellIsFree;
    };

    for(int i=0; i<m_occupancy_map.size(); i++)
    {
        for(int j=0; j<m_occupancy_map[0].size(); j++)
        {
            m_occupancy_map[i][j] = cellFree(i,j);
        }
    }
    
}


void MapGenerator::runDDA(const glm::vec2& origin, const glm::vec2& direction, const glm::vec2& reflectorPosition, uint rowIndex)
{
    constexpr float reflectorRadius = 0.26;

    static auto identity = [](const bool& b){return b;};
    
    auto doesNotCollideWithReflector = [reflectorPosition, reflectorRadius](const glm::vec2& position)
    {
        return glm::distance(position, reflectorPosition) > reflectorRadius;
    };

    DDA::_2D::RayMarchInfo rayData =  DDA::_2D::marchRay<bool>(origin, direction, 10, 
        {m_occupancy_map, m_mapOrigin, m_rayMarchResolution},
        identity, doesNotCollideWithReflector);
    
    for(const auto& [index, length] : rayData.lengthInCell)
    {
        uint columnIndex = index2Dto1D(index);
        m_lengthRayInCell(rowIndex,columnIndex) = length; 
    }
}




int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapGenerator>();
    
    rclcpp::Rate rate(10);
    while(rclcpp::ok() && node->m_map_msg.get() == nullptr)
    {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    node->getEnvironment();
    node->readFile();
    node->solve();
    return 0;
}