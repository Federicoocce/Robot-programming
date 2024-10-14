
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include "grid_map.h"
#include "laser_scan.h"
#include "laser_scanner.h"
#include "world_item.h"
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/LaserScan.h>
#include "config_parser.h"

using namespace std;

// Global variables for robot velocity
float tvel = 0;  // Translational velocity
float rvel = 0;  // Rotational velocity
Eigen::Vector2f flipY(const Eigen::Vector2f& p, int height) {
    return Eigen::Vector2f(p.x(), height - 1 - p.y());
}

Eigen::Vector2f adjustedWorld2Grid(const GridMap& grid_map, const Eigen::Vector2f& world_pos) {
    Eigen::Vector2f grid_origin = grid_map.origin();
    float resolution = grid_map.resolution();
    return Eigen::Vector2f(
        (world_pos.x() - grid_origin.x()) / resolution,
        (world_pos.y() - grid_origin.y()) / resolution
    );
}

// Callback to receive velocity commands
void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    tvel = msg->linear.x;
    rvel = msg->angular.z;
    ROS_INFO("Received velocity command: linear=%f, angular=%f", tvel, rvel);
}



void publishOdometry(ros::Publisher &odom_pub, const Eigen::Isometry2f& robot_pose, float tvel, float rvel, const std::string& robot_frame_id) {
    static tf2_ros::TransformBroadcaster tf_broadcaster;
    ros::Time current_time = ros::Time::now();

    // Transform: map -> robot_frame_id
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = current_time;
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = robot_frame_id;
    transform_stamped.transform.translation.x = robot_pose.translation().x();
    transform_stamped.transform.translation.y = robot_pose.translation().y();
    transform_stamped.transform.translation.z = 0.0;

    float yaw = atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0));
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();

    tf_broadcaster.sendTransform(transform_stamped);

    // Publish odometry
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "map";
    odom.child_frame_id = robot_frame_id;

    odom.pose.pose.position.x = robot_pose.translation().x();
    odom.pose.pose.position.y = robot_pose.translation().y();
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = tvel;
    odom.twist.twist.angular.z = rvel;

    odom_pub.publish(odom);
}
nav_msgs::OccupancyGrid gridMapToOccupancyGrid(const GridMap& grid_map) {
      // MAP MESSAGE
    nav_msgs::OccupancyGrid map_msg;                 // Create a message of type OccupancyGrid
    map_msg.header.stamp = ros::Time::now();         // Set the timestamp of the map messagee
    map_msg.header.frame_id = "map";                 // Set the FRAME_ID of the map message
    map_msg.info.resolution = grid_map.resolution(); // Set the resolution of the map from the GridMap instance
    map_msg.info.width = grid_map.cols;              // Set the width of the map from the GridMap instance
    map_msg.info.height = grid_map.rows;           // Set the height of the map from the GridMap instance
    map_msg.info.origin.position.x = grid_map.origin().x();
    map_msg.info.origin.position.y = grid_map.origin().y();
    map_msg.info.origin.position.z = 0.0;

    tf::Quaternion q;
    q.setRPY(0, 0, 0); // Se l'orientazione è zero
    map_msg.info.origin.orientation.x = q.x();
    map_msg.info.origin.orientation.y = q.y();
    map_msg.info.origin.orientation.z = q.z();
    map_msg.info.origin.orientation.w = q.w();

    // Populate the data of the map message
    map_msg.data.resize(grid_map.rows * grid_map.cols);
    for (int row = 0; row < grid_map.rows; ++row)
    {
        for (int col = 0; col < grid_map.cols; ++col)
        {
            uint8_t value = grid_map(row, col);
            if (value < 127)
            {
                map_msg.data[row * grid_map.cols + col] = 100; // Occupied
            }
            else
            {
                map_msg.data[row * grid_map.cols + col] = 0; // Free
            }
        }
    }

    return map_msg;
}

sensor_msgs::LaserScan convertToROSLaserScan(const LaserScan& laser_scan) {
    sensor_msgs::LaserScan ros_scan;
    ros_scan.header.stamp = ros::Time::now();
    ros_scan.header.frame_id = "base_laser";
    ros_scan.angle_min = laser_scan.angle_min;
    ros_scan.angle_max = laser_scan.angle_max;
    ros_scan.angle_increment = (laser_scan.angle_max - laser_scan.angle_min) / (laser_scan.ranges_num - 1);
    ros_scan.time_increment = 0;
    ros_scan.scan_time = 0.1;
    ros_scan.range_min = laser_scan.range_min;
    ros_scan.range_max = laser_scan.range_max;
    ros_scan.ranges = laser_scan.ranges;

    ros_scan.intensities.clear();
    return ros_scan;
}
void publishLaserTransform(const std::string& robot_frame_id, const std::string& laser_frame_id, const Eigen::Isometry2f& laser_pose) {
    static tf2_ros::TransformBroadcaster tf_broadcaster;
    ros::Time current_time = ros::Time::now();

    geometry_msgs::TransformStamped laser_transform;
    laser_transform.header.stamp = current_time;
    laser_transform.header.frame_id = robot_frame_id;
    laser_transform.child_frame_id = laser_frame_id;
    laser_transform.transform.translation.x = laser_pose.translation().x();
    laser_transform.transform.translation.y = laser_pose.translation().y();
    laser_transform.transform.translation.z = 0.0;

    float yaw = atan2(laser_pose.rotation()(1, 0), laser_pose.rotation()(0, 0));
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    laser_transform.transform.rotation.x = q.x();
    laser_transform.transform.rotation.y = q.y();
    laser_transform.transform.rotation.z = q.z();
    laser_transform.transform.rotation.w = q.w();

    tf_broadcaster.sendTransform(laser_transform);
}

void publishRobotMarker(ros::Publisher& marker_pub, const Eigen::Isometry2f& robot_pose, float radius, const std::string& robot_frame_id, int id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "robot";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = robot_pose.translation().x();
    marker.pose.position.y = robot_pose.translation().y();
    marker.pose.position.z = 0;
    
    float yaw = atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0));
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    
    marker.scale.x = radius * 2;
    marker.scale.y = radius * 2;
    marker.scale.z = 0.25;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker_pub.publish(marker);
}


int main(int argc, char** argv) {
    if (argc < 2) {
        cout << "usage: " << argv[0] << " <config_file>" << endl;
        return -1;
    }
    const char* config_file = argv[1];

    // Parse configuration
    SimulatorConfig sim_config = parseConfig(config_file);

    // ROS node initialization
    ros::init(argc, argv, "robot_simulator");
    ros::NodeHandle nh;

    // Publishers and subscribers
    std::vector<ros::Publisher> odom_pubs;
    std::vector<ros::Publisher> laser_pubs;
    std::vector<ros::Subscriber> vel_subs;
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    // Set loop rate (e.g., 10 Hz)
    ros::Rate loop_rate(10);

    // Load the map
    GridMap grid_map(0, 0, sim_config.resolution);
    grid_map.loadFromImage(sim_config.map_file.c_str(), sim_config.resolution);
    ROS_INFO("Map loaded. Size: %dx%d, Resolution: %f", grid_map.cols, grid_map.rows, grid_map.resolution());

    // Initialize world and robots
    World world_object(grid_map);
    std::vector<UnicyclePlatform*> robots;
    std::vector<LaserScanner*> scanners;

    for (const auto& robot_config : sim_config.robots) {
        Eigen::Isometry2f robot_in_world = Eigen::Isometry2f::Identity();
        robot_in_world.translation() << robot_config.start_x, robot_config.start_y;
        robot_in_world.rotate(Eigen::Rotation2Df(robot_config.start_alpha));

        UnicyclePlatform *robot = new UnicyclePlatform(world_object, robot_in_world);
        robot->radius = robot_config.radius;
        // Add the robot to the vector of pointers to robots
        robots.push_back(robot);

        // Create publishers and subscribers for this robot
        std::string odom_topic = "/robot" + std::to_string(robot_config.id) + "/odom";
        odom_pubs.push_back(nh.advertise<nav_msgs::Odometry>(odom_topic, 50));

        std::string cmd_vel_topic = "/robot" + std::to_string(robot_config.id) + "/cmd_vel";
        vel_subs.push_back(nh.subscribe<geometry_msgs::Twist>(cmd_vel_topic, 10, 
            [&robots, i = robots.size() - 1](const geometry_msgs::Twist::ConstPtr& msg) {
                robots[i]->tvel = msg->linear.x;
                robots[i]->rvel = msg->angular.z;
            }));

        // Initialize laser scanners for this robot
        for (const auto& lidar_config : robot_config.lidars) {
            LaserScan *scan = new LaserScan();
            scan->angle_min = -M_PI;
            scan->angle_max = M_PI;
            scan->range_min = lidar_config.range_min;
            scan->range_max = lidar_config.range_max;

            Eigen::Isometry2f scanner_in_robot = Eigen::Isometry2f::Identity();
            scanner_in_robot.translation().x() = 0; // Adjust as needed

            LaserScanner *scanner = new LaserScanner(*scan, *robot, scanner_in_robot,10.0f);
            scanners.push_back(scanner);
            scanners.back() -> radius = 0.1; // Adjust as needed

            laser_pubs.push_back(nh.advertise<sensor_msgs::LaserScan>(lidar_config.topic, 50));
        }
    }

    // Convert and publish the map once
    nav_msgs::OccupancyGrid og = gridMapToOccupancyGrid(grid_map);
    map_pub.publish(og);

    while (ros::ok()) {
        // Update world
        float dt = 0.1;  // Time step (assuming 10Hz loop rate)
        world_object.tick(dt);

         for (size_t i = 0; i < robots.size(); ++i) {
            auto& robot = robots[i];
            auto& robot_config = sim_config.robots[i];

            // Get updated robot pose
            Eigen::Isometry2f robot_pose = robot->pose_in_parent;

            // Publish odometry
            publishOdometry(odom_pubs[i], robot_pose, robot->tvel, robot->rvel, robot_config.frame_id);

            // Update and publish laser scans
            for (size_t j = 0; j < robot_config.lidars.size(); ++j) {
                auto& scanner = scanners[i * robot_config.lidars.size() + j];
                auto& lidar_config = robot_config.lidars[j];

                scanner->tick(dt);
                if (scanner->scan_ready) {
                    sensor_msgs::LaserScan ros_scan = convertToROSLaserScan(scanner->scan);
                    ros_scan.header.frame_id = lidar_config.frame_id;
                    laser_pubs[i * robot_config.lidars.size() + j].publish(ros_scan);
                }

                // Publish laser scanner transform
                publishLaserTransform(robot_config.frame_id, lidar_config.frame_id, scanner->pose_in_parent);
            }

            // Publish robot visualization marker
            publishRobotMarker(marker_pub, robot_pose, robot->radius, robot_config.frame_id, i);
        }

        // Handle ROS callbacks
        ros::spinOnce();

        // Sleep for the remaining loop time
        loop_rate.sleep();
    }

    return 0;
}
