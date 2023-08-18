#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>

ros::Publisher cmd_vel_pub;
ros::Subscriber odom_sub;
geometry_msgs::Pose current_pose;
bool received_initial_pose = false;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_pose = msg->pose.pose;
    received_initial_pose = true;
}

double getYawFromQuaternion(const geometry_msgs::Quaternion& quat)
{
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "square_trajectory_node");
    ros::NodeHandle nh;

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    odom_sub = nh.subscribe("odom", 100, odomCallback);

    double side_length = 0.5;  // 0.5 meters
    double loop_rate = 10.0;    // 10 Hz

    ros::Rate rate(loop_rate);

    while (ros::ok())
    {

        // Move forward
        geometry_msgs::Twist move_cmd;
        move_cmd.linear.x = 0.1;  // Adjust this value for desired speed
        move_cmd.angular.z = 0.0;
        cmd_vel_pub.publish(move_cmd);

        double start_x = current_pose.position.x;
        double start_y = current_pose.position.y;
        double distance_traveled = 0.0;

        while (distance_traveled < side_length)
        {
            ros::spinOnce();
            double current_x = current_pose.position.x;
            double current_y = current_pose.position.y;
            distance_traveled = std::sqrt(std::pow(current_x - start_x, 2) + std::pow(current_y - start_y, 2));
            rate.sleep();
        }

        // Stop
        move_cmd.linear.x = 0.0;
        cmd_vel_pub.publish(move_cmd);

        // Turn 90 degrees
        move_cmd.linear.x = 0.0;
        move_cmd.angular.z = 0.785;  // 90 degrees in radians
        cmd_vel_pub.publish(move_cmd);

        double initial_yaw = getYawFromQuaternion(current_pose.orientation);
        double target_yaw = initial_yaw + 0.785;  // 90 degrees in radians

        while (getYawFromQuaternion(current_pose.orientation) < target_yaw)
        {
            ros::spinOnce();
            rate.sleep();
        }

        // Stop
        move_cmd.angular.z = 0.0;
        cmd_vel_pub.publish(move_cmd);

        rate.sleep();
    }

    return 0;
}





























/* #include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>

nav_msgs::Odometry odom_msg;


void odomCallback(const nav_msgs::Odometry& msg)    
{
    odom_msg.header.frame_id = "odom";
    odom_msg.pose.pose.position.x = msg.pose.pose.position.x;
    odom_msg.pose.pose.position.y = msg.pose.pose.position.y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.z = msg.pose.pose.orientation.z;

}




int main(int argc, char **argv){
  ros::init(argc, argv, "square");
  ros::NodeHandle n;

  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  ros::Subscriber odom_sub = n.subscribe("odom", 100, odomCallback);
  
  ros::Rate loop_rate(10);
  //geometry_msgs::Quaternion prev_orientation = tf::createQuaternionMsgFromYaw(0.0);
  geometry_msgs::Twist cmd_vel_msg;
  double x,y, prev_orientation;
  double start_x = odom_msg.pose.pose.position.x;
  double start_y = odom_msg.pose.pose.position.y;
  double distance_traveled = 0.0;


  while (ros::ok()){
    y = odom_msg.pose.pose.position.y;
    x = odom_msg.pose.pose.position.x;
        while (distance_traveled < side_length)
     {
            ros::spinOnce();
            double current_x = current_pose.position.x;
            double current_y = current_pose.position.y;
            distance_traveled = std::sqrt(std::pow(current_x - start_x, 2) + std::pow(current_y - start_y, 2));
            rate.sleep();
     }
    if (y < (double) 0.5){
      cmd_vel_msg.linear.x = 0.5;
      cmd_vel_msg.angular.z = 0.0;
    }else if(y >= (double) 0.5){
      while (odom_msg.pose.pose.orientation - prev_orientation < 90){
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.3;
      }
      //prev_orientation = 0;
    }
    prev_orientation = odom_msg.pose.pose.orientation;

    cmd_vel_pub.publish(cmd_vel_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
} */
