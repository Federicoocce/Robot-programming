
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


using namespace std;

// Global variables for robot velocity
float tvel = 1;  // Translational velocity
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



void publishOdometry(ros::Publisher &odom_pub, const Eigen::Isometry2f& robot_pose, float tvel, float rvel) {
    static tf2_ros::TransformBroadcaster tf_broadcaster;
    ros::Time current_time = ros::Time::now();

    // Trasformazione diretta: map -> base_link
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = current_time;
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "base_link";
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
    ROS_INFO("Published transform: map -> base_link (x=%f, y=%f)", robot_pose.translation().x(), robot_pose.translation().y());

    // Pubblicazione dell'odometria
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "map"; // Cambiato da "odom" a "map"
    odom.child_frame_id = "base_link";

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
    ROS_INFO("Published odometry message");
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
            int flipped_row = grid_map.rows - 1 - row;
            uint8_t value = grid_map(flipped_row, col);
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


int main(int argc, char** argv) {
    if (argc < 2) {
        cout << "usage: " << argv[0] << " <image_file> <resolution>" << endl;
        return -1;
    }
    const char* filename = argv[1];
    float resolution = atof(argv[2]);

    // ROS node initialization
    ros::init(argc, argv, "robot_simulator");
    ros::NodeHandle nh;

    // Publisher for odometry
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    //pub for laser
    ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 50);
    // New publisher for the map
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);

    // Subscriber for velocity commands
    ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 10, velocityCallback);

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Set loop rate (e.g., 10 Hz)
    ros::Rate loop_rate(10);

    cout << "Running " << argv[0] << " with arguments" << endl
         << "-filename:" << argv[1] << endl
         << "-resolution: " << argv[2] << endl;

    // Load the map
    GridMap grid_map(0, 0, 0.1);
    grid_map.loadFromImage(filename, resolution);
    // Print debug information about the loaded map
    ROS_INFO("Map loaded. Size: %dx%d, Resolution: %f", grid_map.cols, grid_map.rows, grid_map.resolution());
    ROS_INFO("Map origin: (%f, %f)", grid_map.origin().x(), grid_map.origin().y());

    // Initialize world and robot
    WorldItem* items[100];
    memset(items, 0, sizeof(WorldItem*) * 100);

    World world_object(grid_map);
    items[0] = &world_object;

    Eigen::Isometry2f robot_in_world = Eigen::Isometry2f::Identity();
    robot_in_world.translation() << 10,50;
    UnicyclePlatform robot(world_object, robot_in_world);
    robot.radius = 1;
    robot.tvel = 0;
    robot.rvel = 0;
    items[1] = &robot;

    // Initialize laser scanner
    LaserScan scan;
    Eigen::Isometry2f scanner_in_robot = Eigen::Isometry2f::Identity();
    scanner_in_robot.translation().x() = 1;
    LaserScanner scanner(scan, robot, scanner_in_robot, 10);
    scanner.radius = 1;
    items[2] = &scanner;
    // Convert and publish the map once
    nav_msgs::OccupancyGrid og = gridMapToOccupancyGrid(grid_map);
    map_pub.publish(og);

    int count = 0;
    while (ros::ok()) {
        // Update robot velocities
        robot.tvel = tvel;
        robot.rvel = rvel;

        // Update world (this will update robot position through UnicyclePlatform::tick)
        float dt = 0.1;  // Time step (assuming 10Hz loop rate)
        world_object.tick(dt);

        // Get updated robot pose
        Eigen::Isometry2f robot_pose = robot.pose_in_parent;
        float yaw = atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0));

        // Debug output
        ROS_INFO("Robot position (world): x=%f, y=%f, theta=%f", 
                robot_pose.translation().x(), 
                robot_pose.translation().y(),
                yaw);
        
        // Convert world position to grid position with flipped Y
        Eigen::Vector2f grid_pos = grid_map.world2grid(robot_pose.translation());
        grid_pos = flipY(grid_pos, grid_map.rows);
        ROS_INFO("Robot position (grid, flipped): x=%f, y=%f", grid_pos.x(), grid_pos.y());

        // Check grid cell value at robot's position
        Eigen::Vector2i grid_pos_int = grid_pos.cast<int>();
        if (grid_map.inside(grid_pos_int)) {
            uint8_t cell_value = grid_map(grid_pos_int);
            ROS_INFO("Grid cell value at robot position: %d", cell_value);
            ROS_INFO("Is cell occupied: %s", (cell_value < 127) ? "Yes" : "No");
        } else {
            ROS_WARN("Robot position is outside the grid map!");
        }

        ROS_INFO("Robot velocities: linear=%f, angular=%f", robot.tvel, robot.rvel);

        // Publish odometry
        publishOdometry(odom_pub, robot_pose, tvel, rvel);
        // Update laser scanner
        scanner.tick(dt);  

        // Publish laser scan data if a new scan is ready
        if (scanner.scan_ready) {
            sensor_msgs::LaserScan ros_scan = convertToROSLaserScan(scan);
            laser_pub.publish(ros_scan);
        }

        // Publish laser scanner transform
        static tf2_ros::TransformBroadcaster tf_broadcaster;
        geometry_msgs::TransformStamped laser_transform;
        laser_transform.header.stamp = ros::Time::now();
        laser_transform.header.frame_id = "base_link";
        laser_transform.child_frame_id = "base_laser";
        laser_transform.transform.translation.x = scanner_in_robot.translation().x();
        laser_transform.transform.translation.y = scanner_in_robot.translation().y();
        laser_transform.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        laser_transform.transform.rotation.x = q.x();
        laser_transform.transform.rotation.y = q.y();
        laser_transform.transform.rotation.z = q.z();
        laser_transform.transform.rotation.w = q.w();
        tf_broadcaster.sendTransform(laser_transform);

        // Publish robot visualization marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";  // Changed from "base_link" to "map"
        marker.header.stamp = ros::Time::now();
        marker.ns = "robot";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = robot_pose.translation().x();
        marker.pose.position.y = robot_pose.translation().y();
        marker.pose.position.z = 0;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.25;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker_pub.publish(marker);

        // Handle ROS callbacks
        ros::spinOnce();

        // Sleep for the remaining loop time
        loop_rate.sleep();
    }

    return 0;
}
