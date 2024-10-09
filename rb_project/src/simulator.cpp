
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

using namespace std;

// Global variables for robot velocity
float tvel = 0;  // Translational velocity
float rvel = 0;  // Rotational velocity

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
    static tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time = ros::Time::now();

    // Create quaternion from yaw for odometry
    float yaw = atan2(robot_pose.rotation()(1,0), robot_pose.rotation()(0,0));
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);
    
    // Publish static transform: map -> odom
    geometry_msgs::TransformStamped map_to_odom_trans;
    map_to_odom_trans.header.stamp = current_time;
    map_to_odom_trans.header.frame_id = "map";
    map_to_odom_trans.child_frame_id = "odom";
    map_to_odom_trans.transform.translation.x = 0;
    map_to_odom_trans.transform.translation.y = 0;
    map_to_odom_trans.transform.translation.z = 0;
    map_to_odom_trans.transform.rotation.w = 1;
    odom_broadcaster.sendTransform(map_to_odom_trans);
    ROS_INFO("Published transform: map -> odom");

    // Publish dynamic transform: odom -> base_link
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = robot_pose.translation().x();
    odom_trans.transform.translation.y = robot_pose.translation().y();
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);
    ROS_INFO("Published transform: odom -> base_link (x=%f, y=%f)", robot_pose.translation().x(), robot_pose.translation().y());

    // Publish odometry message
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = robot_pose.translation().x();
    odom.pose.pose.position.y = robot_pose.translation().y();
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.twist.twist.linear.x = tvel;
    odom.twist.twist.angular.z = rvel;

    odom_pub.publish(odom);
    ROS_INFO("Published odometry message");
}
nav_msgs::OccupancyGrid gridMapToOccupancyGrid(const GridMap& grid_map) {
    nav_msgs::OccupancyGrid og;
    og.header.frame_id = "map";
    og.header.stamp = ros::Time::now();
    og.info.resolution = grid_map.resolution();
    og.info.width = grid_map.cols;
    og.info.height = grid_map.rows;
    og.info.origin.position.x = grid_map.origin().x();
    og.info.origin.position.y = grid_map.origin().y();
    og.info.origin.orientation.w = 1.0;

    og.data.resize(grid_map.cols * grid_map.rows);
    for (int i = 0; i < grid_map.cols * grid_map.rows; ++i) {
        og.data[i] = grid_map.cells[i] < 127 ? 100 : 0;
    }

    return og;
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
    Canvas canvas;
    Eigen::Vector2f center = grid_map.grid2world(grid_map.origin());
    cerr << "center: " << center.transpose() << endl;
    cerr << "origin: " << grid_map.origin().transpose() << endl;

    grid_map.draw(canvas);

    // Initialize world and robot
    WorldItem* items[100];
    memset(items, 0, sizeof(WorldItem*) * 100);

    World world_object(grid_map);
    items[0] = &world_object;

    Eigen::Isometry2f robot_in_world = Eigen::Isometry2f::Identity();
    robot_in_world.translation() << 2.5,2.5;
    UnicyclePlatform robot(world_object, robot_in_world);
    robot.radius = 0.001;
    robot.tvel = 0;
    robot.rvel = 0;
    items[1] = &robot;

    // Initialize laser scanner
    LaserScan scan;
    Eigen::Isometry2f scanner_in_robot = Eigen::Isometry2f::Identity();
    scanner_in_robot.translation().x() = 1;
    LaserScanner scanner(scan, robot, scanner_in_robot, 10);
    scanner.radius = 0.05;
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
        
        // Convert world position to grid position
        Eigen::Vector2f grid_pos = adjustedWorld2Grid(grid_map, robot_pose.translation());

        // After calculating grid_pos, add this debug output
        ROS_INFO("Grid map origin: x=%f, y=%f", grid_map.origin().x(), grid_map.origin().y());
        ROS_INFO("Grid map resolution: %f", grid_map.resolution());
        ROS_INFO("Robot position (grid): x=%f, y=%f", grid_pos.x(), grid_pos.y());

        // Check grid cell value at robot's position
        Eigen::Vector2i grid_pos_int = grid_pos.cast<int>();
        if (grid_map.inside(grid_pos_int)) {
            uint8_t cell_value = grid_map(grid_pos_int);
            ROS_INFO("Grid cell value at robot position: %d", cell_value);
        } else {
            ROS_WARN("Robot position is outside the grid map!");
        }

        ROS_INFO("Robot velocities: linear=%f, angular=%f", robot.tvel, robot.rvel);

        // Publish odometry
        publishOdometry(odom_pub, robot_pose, tvel, rvel);

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
