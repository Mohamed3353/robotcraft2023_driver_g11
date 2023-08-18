#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>




geometry_msgs::Twist move_cmd;


void cmdVelCallback(const geometry_msgs::Twist& msg)    //geometry_msgs::PoseStamped
{
    move_cmd = msg;
}

void poseCallback(const geometry_msgs::Pose2D& pose_msg)    //geometry_msgs::PoseStamped
{
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose_msg.theta);
    // Republish pose information as Odometry message
    nav_msgs::Odometry odom_msg;
    //odom_msg.header = pose_msg->header;
    odom_msg.header.frame_id = "odom";
    //odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = pose_msg.x;
    odom_msg.pose.pose.position.y = pose_msg.y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.twist.twist = move_cmd;
    odom_msg.pose.pose.orientation = odom_quat;

    // Publish Odometry message
    ros::NodeHandle nh;
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);
    odom_pub.publish(odom_msg);

    // Broadcast TF transform
    tf::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = odom_msg.pose.pose.position.x;
    transformStamped.transform.translation.y = odom_msg.pose.pose.position.y;
    transformStamped.transform.translation.z = odom_msg.pose.pose.position.z;
    transformStamped.transform.rotation = odom_msg.pose.pose.orientation;

    br.sendTransform(transformStamped);
}


void frontDistanceCallback(const std_msgs::Float32& front_distance_msg)
{
    sensor_msgs::Range ir_front_msg;
    ir_front_msg.header.frame_id = "front_ir";
    ir_front_msg.radiation_type = 1;
    ir_front_msg.field_of_view = 0.034906585;
    ir_front_msg.min_range = 0.1;
    ir_front_msg.max_range = 0.8;
    ir_front_msg.range = front_distance_msg.data;

    ros::NodeHandle nh;
    ros::Publisher ir_front_pub = nh.advertise<sensor_msgs::Range>("ir_front_sensor", 100);
    ir_front_pub.publish(ir_front_msg);
}

void rightDistanceCallback(const std_msgs::Float32& right_distance_msg)
{
    sensor_msgs::Range ir_right_msg;
    ir_right_msg.header.frame_id = "right_ir";
    ir_right_msg.radiation_type = 1;
    ir_right_msg.field_of_view = 0.034906585;
    ir_right_msg.min_range = 0.1;
    ir_right_msg.max_range = 0.8;
    ir_right_msg.range = right_distance_msg.data;

    ros::NodeHandle nh;
    ros::Publisher ir_right_pub = nh.advertise<sensor_msgs::Range>("ir_right_sensor", 100);
    ir_right_pub.publish(ir_right_msg);
}

void leftDistanceCallback(const std_msgs::Float32& left_distance_msg)
{
    sensor_msgs::Range ir_left_msg;
    ir_left_msg.header.frame_id = "left_ir";
    ir_left_msg.radiation_type = 1;
    ir_left_msg.field_of_view = 0.034906585;
    ir_left_msg.min_range = 0.1;
    ir_left_msg.max_range = 0.8;
    ir_left_msg.range = left_distance_msg.data;

    ros::NodeHandle nh;
    ros::Publisher ir_left_pub = nh.advertise<sensor_msgs::Range>("ir_left_sensor", 100);
    ir_left_pub.publish(ir_left_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_driver_node");
    ros::NodeHandle nh;

    // Subscribe to the /pose topic
    ros::Subscriber pose_sub = nh.subscribe("pose", 100, poseCallback);
    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 100, cmdVelCallback);
    ros::Subscriber front_distance_sub = nh.subscribe("front_distance", 100, frontDistanceCallback);
    ros::Subscriber right_distance_sub = nh.subscribe("right_distance", 100, rightDistanceCallback);
    ros::Subscriber left_distance_sub = nh.subscribe("left_distance", 100, leftDistanceCallback);

    
    ros::spin();

    return 0;
}
