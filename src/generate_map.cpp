#include <robot2023/generate_map.h>
#include <fstream>
#include <PoseJSON.hpp>
#include <robot2023/common.h>
#include <olfaction_msgs/msg/tdlas.hpp>
#include <tf2/LinearMath/Transform.h>
#include <eigen3/Eigen/SVD>
#include <eigen3-nnls/src/nnls.h>

#include <opencv4/opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

static cv::Mat rays_image;

using PoseStamped = geometry_msgs::msg::PoseStamped;
using TDLAS=olfaction_msgs::msg::TDLAS;
using namespace std::chrono_literals;

static TDLAS jsonToTDLAS(const nlohmann::json& json)
{
    TDLAS tdlas;
    tdlas.average_ppmxm = (double) json["average_ppmxm"].get<int>();
    tdlas.average_absorption_strength = json["average_absorption_strength"].get<double>();
    tdlas.average_reflection_strength = json["average_reflection_strength"].get<double>();
    return tdlas;
}

MapGenerator::MapGenerator() : Node("MapGenerator")
{
    m_mapSub = create_subscription<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).reliable().transient_local(), 
        std::bind(&MapGenerator::mapCallback, this, std::placeholders::_1));

    m_rayMarchResolution = declare_parameter<float>("rayMarchResolution", 0.05f);
}

void MapGenerator::readFile()
{
    std::string filepath = declare_parameter<std::string>("filepath", "/mnt/HDD/colcon_ws/measurement_log");
    std::ifstream file(filepath);
    
    //Count the number of entries and size the matrices accordingly
    int numberOfMeasurements=0;
    std::string line;
    while( std::getline(file, line) )
        numberOfMeasurements++;
    
    m_measurements.resize(numberOfMeasurements);
    m_lengthRayInCell.resize(numberOfMeasurements, m_num_cells);
    m_concentration.resize(m_num_cells);
    
    //restart the ifstream
    file.clear();
    file.seekg(0, std::ios::beg);
    
    int measurementIndex = 0;

    rays_image.create(cv::Size(m_occupancy_map[0].size(), m_occupancy_map.size()), CV_8UC1);
    while(std::getline(file, line))
    {
        auto json = nlohmann::json::parse(line);
        tf2::Transform rhodon = poseToTransform( nav2MQTT::from_json(json["rhodon"]).pose );
        tf2::Transform giraff = poseToTransform( nav2MQTT::from_json(json["giraff"]).pose );
        TDLAS tdlas = jsonToTDLAS(json["reading"]);

        m_measurements[measurementIndex] = tdlas.average_ppmxm;

        //fill in the cell raytracing thing
        glm::vec2 rayOrigin = glm::fromTF( rhodon.getOrigin() );
        glm::vec2 rayDirection = glm::fromTF( tf2::quatRotate(rhodon.getRotation(), {0,1,0}) ); 
        glm::vec2 reflectorPosition = glm::fromTF( giraff.getOrigin() );

        runDDA(rayOrigin, rayDirection, reflectorPosition, measurementIndex, tdlas.average_ppmxm);
        measurementIndex++;
    }
    file.close();

    for(int i = 0; i<m_occupancy_map.size(); i++)
    {
        for(int j=0; j<m_occupancy_map[0].size(); j++)
        {
            if(!m_occupancy_map[i][j])
                rays_image.at<cv::uint8_t>(i, j) = 50;
        }
    }
    cv::imwrite("rays.png", rays_image);
    
    std::ofstream test("lengths");
    test<<m_lengthRayInCell;
    test.close();
}

void MapGenerator::solve()
{
    //m_concentration = m_lengthRayInCell.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(m_measurements);
    
    //m_concentration = m_lengthRayInCell.colPivHouseholderQr().solve(m_measurements);

    //m_concentration = m_lengthRayInCell.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(m_measurements);
    
    Eigen::NNLS<Eigen::MatrixXf> solver(m_lengthRayInCell, 10000, 0.00001);
    solver.solve(m_measurements);
    m_concentration = solver.x();
}



void MapGenerator::getEnvironment()
{    
    if(m_rayMarchResolution < m_map_msg->info.resolution)
    {
        RCLCPP_WARN(get_logger(), "Chosen raymarch resolution (%fm) is smaller than the resolution of the occupancy map (%fm). This can cause problems, so raymarch will happen at a %fm resolution",
            m_rayMarchResolution, m_map_msg->info.resolution, m_map_msg->info.resolution);
        m_rayMarchResolution = m_map_msg->info.resolution;
    }

    double resoultionRatio = m_map_msg->info.resolution / m_rayMarchResolution;
    int num_cells_x = m_map_msg->info.width * resoultionRatio;
    int num_cells_y = m_map_msg->info.height * resoultionRatio;
    m_num_cells = num_cells_x * num_cells_y;
    
    m_mapOrigin.x = m_map_msg->info.origin.position.x;
    m_mapOrigin.y = m_map_msg->info.origin.position.y;
    
    m_occupancy_map.resize(num_cells_x, std::vector<bool>(num_cells_y ) );


    auto cellFree = [this, resoultionRatio](int argI, int argJ)
    {
        int width = m_map_msg->info.width;
        bool cellIsFree = true;
        for(int i = argI/resoultionRatio; i<(argI+1)/resoultionRatio; i++)
        {
            for(int j = argJ/resoultionRatio; j<(argJ+1)/resoultionRatio; j++)
            {
                int value = m_map_msg->data[i + j*width];
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


void MapGenerator::runDDA(const glm::vec2& origin, const glm::vec2& direction, const glm::vec2& reflectorPosition, uint rowIndex, int ppmxm)
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
        uint8_t& r_image = rays_image.at<uint8_t>(index.x, index.y); 
        r_image = std::max((double)r_image, 255 * (ppmxm/100.0));
    }
}

void MapGenerator::writeHeatmap()
{
    cv::Mat image(cv::Size(m_occupancy_map[0].size(), m_occupancy_map.size()), CV_8UC1, cv::Scalar(0,0,0));
    
    {
        std::ofstream csvFile("map.csv");
        float max = 0;
        for(int i = 0; i<m_concentration.size();i++)
            if(m_concentration[i]>max)
                max = m_concentration[i];

        RCLCPP_INFO(get_logger(), "MAX: %f", max);

        for(int i = 0; i<m_occupancy_map.size(); i++)
        {
            for(int j=0; j<m_occupancy_map[0].size(); j++)
            {
                float concentration = m_concentration[i + j*m_occupancy_map.size()];
                image.at<uint8_t>(i, j) = (concentration / max) * 255;
                csvFile << concentration << ",";
            }
            csvFile<<"\n";
        }
        csvFile.close();
    }

    {
        Eigen::VectorXf residuals = m_measurements - m_lengthRayInCell * m_concentration;
        RCLCPP_INFO(get_logger(), "Residual: %f", residuals.norm()); 
    }

    cv::Mat img_color;
    cv::applyColorMap(image, img_color, cv::COLORMAP_JET);

    for(int i = 0; i<m_occupancy_map.size(); i++)
    {
        for(int j=0; j<m_occupancy_map[0].size(); j++)
        {
            if(!m_occupancy_map[i][j])
                img_color.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0 , 0);
        }
    }

    std::string path = "methane_map.png";
    cv::imwrite(path, img_color);

    RCLCPP_INFO(get_logger(), "MAP WAS GENERATED AT PATH: %s", path.c_str());
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
    node->writeHeatmap();

    rclcpp::shutdown();
    return 0;
}