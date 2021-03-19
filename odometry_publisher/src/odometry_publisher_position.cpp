#include "dynamixel_workbench_msgs/DynamixelStateList.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <vector>

const float wheelRadius = 0.036f;  // meter
const float encodeNumber = 4096;  // encode/r
const float ratio = 212.6f;  // Transmission ratio
const float pi = 3.1415926;

using namespace std;

int left_last_position = 0, left_current_position = 0;  // encode
int right_last_position = 0, right_current_position = 0;  // encode
int back_last_position = 0, back_current_position = 0;
float delta_left_position = 0.0f, delta_right_position = 0.0f, delta_back_position = 0.0f;  // m

float left_velocity = 0.0;  // rad/s
float right_velocity = 0.0;  // rad/s
float back_velocity = 0.0; //rad/s

float wheelBase = 0.25f;  // meter
float coeffient = 1.5f;  // Distance calibration factor

float convertValue2Velocity(const int32_t &value) //check if the .present_velocities are rpm or m/s!!
{
    float velocity = 0;
    const float RPM2RADPERSEC = 0.104719755f;
    const float RPM = 0.732f;
  
    //if (value == 32768 || value == 0) velocity = 0.0f;
    //else if (value > 0 && value < 32768)
    velocity = value * RPM * RPM2RADPERSEC;
    //else if (value > 32768 && value < 65536) velocity = (value - 32768) * RPM * RPM2RADPERSEC  * (-1.0f);

    return velocity;  //rad/s
}


void DynamixelStateCallback(const dynamixel_workbench_msgs::DynamixelStateListConstPtr& msg)
{
    static bool first = true;
    if(msg->dynamixel_state[0].name == "left")
    {
        left_current_position = msg->dynamixel_state[0].present_position;  // encode
        right_current_position = msg->dynamixel_state[1].present_position;  // encode
        back_current_position = msg->dynamixel_state[2].present_position;  // encode
    }
    else
    {
        left_current_position = msg->dynamixel_state[1].present_position;  // encode
        right_current_position = msg->dynamixel_state[0].present_position;  // encode
        back_current_position = msg->dynamixel_state[2].present_position;  // encode
    }
    
    if(first)
    {
        left_last_position = left_current_position;
        right_last_position = right_current_position;
        back_last_position = back_current_position;
        first = false;
    }

    // Left wheel, reverse forward
    if(left_current_position >= left_last_position && left_current_position - left_last_position < encodeNumber/2.0)  // Forward rotation without overflow
    {
        delta_left_position = (double)(left_current_position - left_last_position) * (-1.0);
    }
    else if(left_current_position < left_last_position && left_last_position - left_current_position >= encodeNumber/2.0)  // Forward overflow
    {
        delta_left_position = (double)(left_current_position + encodeNumber - left_last_position) * (-1.0);
    }
    else if(left_current_position < left_last_position && left_last_position - left_current_position < encodeNumber/2.0)  // Reverse does not overflow
    {
        delta_left_position = (double)(left_last_position - left_current_position);
    }
    else  // Reverse overflow，left_current_position >= left_last_position && left_current_position - left_last_position >= encodeNumber/2.0
    {
        delta_left_position = (double)(left_last_position + encodeNumber - left_current_position);
    }

    // Right wheel, turn forward
    if(right_current_position >= right_last_position && right_current_position - right_last_position < encodeNumber/2.0)  // Forward rotation without overflow
    {
        delta_right_position = (double)(right_current_position - right_last_position);
    }
    else if(right_current_position < right_last_position && right_last_position - right_current_position >= encodeNumber/2.0)  // Forward overflow
    {
        delta_right_position = (double)(right_current_position + encodeNumber - right_last_position);
    }
    else if(right_current_position < right_last_position && right_last_position - right_current_position < encodeNumber/2.0)  // Reverse does not overflow
    {
        delta_right_position = (double)(right_last_position - right_current_position) * (-1.0);
    }
    else  // Reverse overflow，right_current_position >= right_last_position && right_current_position - right_last_position >= encodeNumber/2.0
    {
        delta_right_position = (double)(right_last_position + encodeNumber - right_current_position) * (-1.0);
    }

    // Back wheel, turn forward
    if(back_current_position >= back_last_position && back_current_position - back_last_position < encodeNumber/2.0)  // Forward rotation without overflow
    {
        delta_back_position = (double)(back_current_position - back_last_position) * (-1.0);
    }
    else if(back_current_position < back_last_position && back_last_position - back_current_position >= encodeNumber/2.0)  // Forward overflow
    {
        delta_back_position = (double)(back_current_position + encodeNumber - back_last_position) * (-1.0);
    }
    else if(back_current_position < back_last_position && back_last_position - back_current_position < encodeNumber/2.0)  // Reverse does not overflow
    {
        delta_back_position = (double)(back_last_position - back_current_position);
    }
    else  // Reverse overflow，back_current_position >= back_last_position && back_current_position - back_last_position >= encodeNumber/2.0
    {
        delta_back_position = (double)(back_last_position + encodeNumber - back_current_position);
    }


    //delta_left_position = delta_left_position / encodeNumber * 2.0 * pi * wheelRadius * ratio * coeffient;  // Left wheel displacement
    //delta_right_position = delta_right_position / encodeNumber * 2.0 * pi * wheelRadius * ratio * coeffient;  // Right wheel displacement
    //delta_back_position = delta_back_position / encodeNumber * 2.0 * pi * wheelRadius * ratio * coeffient;  // Back wheel displacement

    // debug
    // printf("coeffient: %f\ndelta_left_position: %f\ndelta_right_position: %f\n\n", coeffient, delta_left_position, delta_right_position);

    // ROS_INFO("coeffient: %f\ndelta_left_position: %f\ndelta_right_position: %f\n\n", coeffient, delta_left_position, delta_right_position);

    left_last_position = left_current_position;
    right_last_position = right_current_position;
    back_last_position = back_current_position;

    left_velocity = convertValue2Velocity(msg->dynamixel_state[0].present_velocity) * ratio;  // rad/s
    right_velocity = convertValue2Velocity(msg->dynamixel_state[1].present_velocity) * (-1.0f) * ratio;  // rad/s
    back_velocity = convertValue2Velocity(msg->dynamixel_state[2].present_velocity) * ratio;  // rad/s
}


int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle node;
    ros::param::get("~wheelBase", wheelBase);
    ros::param::get("~coeffient", coeffient);

    ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>("odom", 20);

    tf::TransformBroadcaster odom_broadcaster;
    ros::Subscriber dynamixel_state_sub = node.subscribe("dynamixel_workbench/dynamixel_state", 1, DynamixelStateCallback);

    float x = 0.0;
    float y = 0.0;
    float th = 0.0;

    float w_l = 0.0;
    float w_r = 0.0;
    float w_b = 0.0;
    float v_l = 0.0;
    float v_r = 0.0;
    float v_b = 0.0;
    float v_x = 0.0;
    float v_y = 0.0;
    float w_k = 0.0;
    float delta_x = 0.0;
    float delta_y = 0.0;
    float delta_th = 0.0;

    ros::Rate r(10.0);
    while(node.ok()){
        ros::spinOnce();               // check for incoming messages

    ros::Time current_time = ros::Time::now();

        // rotation velocity of each wheel
        w_l = left_velocity;
        w_r = right_velocity;
        w_b = back_velocity;                                                                       

        // linear velocity of each wheel
        v_l = w_l * wheelRadius * coeffient;
        v_r = w_r * wheelRadius * coeffient;
        v_b = w_b * wheelRadius * coeffient;

        // linear and angular velocity of the robot
        v_x = (2*v_b - v_l - v_r ) / 3; // m/s
        v_y = (sqrt(3)*v_l - sqrt(3)*v_r ) / 3; // m/s
        w_k = (v_r + v_l + v_b) / (3* wheelBase); // rad/s

    

        // update new pose
        delta_x = 2 * pi * wheelRadius * (2*delta_back_position - delta_right_position - delta_left_position ) / (3.0 * encodeNumber );
        delta_y = 2 * pi * wheelRadius * (sqrt(3)*delta_right_position - sqrt(3)*delta_left_position ) / (3.0 * encodeNumber );
        delta_th = wheelRadius * (delta_right_position + delta_left_position + delta_back_position ) / (3.0 * wheelBase * 360 );

        x += delta_x * cos(delta_th) - delta_y * sin(delta_th);
        y += delta_x * sin(delta_th) + delta_y * cos(delta_th);
        th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_footprint";
        odom.twist.twist.linear.x = v_x;
        odom.twist.twist.linear.y = v_y;
        odom.twist.twist.angular.z = w_k;

        //publish the message
        odom_pub.publish(odom);

    //debug
    /*
        ros::Publisher position_pub = node.advertise<geometry_msgs::Pose2D>("delta_position", 20);
        geometry_msgs::Pose2D pose;
    pose.x = left_velocity;
    pose.y = right_velocity;
    pose.theta = delta_th;
    position_pub.publish(pose);
    */

        r.sleep();
  }

  return 0;
}

