#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Robot {
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    
    double x_, y_, theta_;
    bool odom_received_;

public:
    Robot() : nh_(""), x_(0), y_(0), theta_(0), odom_received_(false) {
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("robot0/cmd_vel", 1);
        odom_sub_ = nh_.subscribe("robot0/odom", 10, &Robot::odomCallback, this);
        ROS_INFO("Robot node initialized. Publishing to %s, subscribing to %s", 
                 cmd_vel_pub_.getTopic().c_str(), odom_sub_.getTopic().c_str());
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        ROS_INFO("Odometry callback received");
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, theta_);

        odom_received_ = true;
        ROS_INFO("Odometry received: x=%.2f, y=%.2f, theta=%.2f", x_, y_, theta_);
    }

    void move(double linear_vel, double angular_vel) {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = linear_vel;
        cmd_vel.angular.z = angular_vel;
        cmd_vel_pub_.publish(cmd_vel);
        ROS_INFO("Publishing velocity command: linear=%.2f, angular=%.2f", linear_vel, angular_vel);
    }

    void run() {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            if (!odom_received_) {
                ROS_WARN_THROTTLE(1, "No odometry received yet");
            }
            else{
                move(0.2, 0.1);  // Muovi il robot in avanti e ruota leggermente
                ros::Duration(0.1).sleep();

            }
            
            
            ROS_INFO("Robot position: x=%.2f, y=%.2f, theta=%.2f", x_, y_, theta_);
            
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_node");
    Robot robot;
    robot.run();
    return 0;
}