#ifndef CONFIG_PARSER_H
#define CONFIG_PARSER_H

#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <ros/package.h>

struct LidarConfig {
    std::string frame_id;
    std::string topic;
    int beams;
    double range_min;
    double range_max;
};

struct RobotConfig {
    int id;
    std::string frame_id;
    double radius;
    double start_x;
    double start_y;
    double start_alpha;
    double max_linear_velocity;
    double max_angular_velocity;
    std::vector<LidarConfig> lidars;
};

struct SimulatorConfig {
    std::string map_file;
    double resolution;
    std::vector<RobotConfig> robots;
};

SimulatorConfig parseConfig(const std::string& config_file) {
    YAML::Node config = YAML::LoadFile(config_file);
    SimulatorConfig sim_config;

    // Get the package path
    std::string package_path = ros::package::getPath("rb_project");

    // Construct the full path to the map file
    std::string map_filename = config["environment"]["map"].as<std::string>();
    sim_config.map_file = package_path + "/maps/" + map_filename;

    sim_config.resolution = config["environment"]["resolution"].as<double>();

    for (const auto& robot_node : config["robots"]) {
        RobotConfig robot;
        robot.id = robot_node["id"].as<int>();
        robot.frame_id = robot_node["frame_id"].as<std::string>();
        robot.radius = robot_node["radius"].as<double>();
        robot.start_x = robot_node["start_position"]["x"].as<double>();
        robot.start_y = robot_node["start_position"]["y"].as<double>();
        robot.start_alpha = robot_node["start_position"]["alpha"].as<double>();
        robot.max_linear_velocity = robot_node["max_velocity"]["linear"].as<double>();
        robot.max_angular_velocity = robot_node["max_velocity"]["angular"].as<double>();

        for (const auto& device_node : robot_node["devices"]) {
            if (device_node["type"].as<std::string>() == "lidar") {
                LidarConfig lidar;
                lidar.frame_id = device_node["frame_id"].as<std::string>();
                lidar.topic = device_node["topic"].as<std::string>();
                lidar.beams = device_node["beams"].as<int>();
                lidar.range_min = device_node["range"]["min"].as<double>();
                lidar.range_max = device_node["range"]["max"].as<double>();
                robot.lidars.push_back(lidar);
            }
        }

        sim_config.robots.push_back(robot);
    }

    return sim_config;
}

#endif // CONFIG_PARSER_H