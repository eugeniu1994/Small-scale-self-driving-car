#include "six_wheel_odom/six_wheel_odom.h"
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

namespace SIXWHEELODOM
{
template <typename T>
inline bool getRequiredParam(const ros::NodeHandle &nh, std::string name, T &value);

six_wheel_odom::six_wheel_odom(ros::NodeHandle nh, ros::NodeHandle private_nh) : odom_frame("odom"), base_frame("base_link"), publish_tf(false), x(0.0), y(0.0), yaw(0.0)
{
    // get ROS parameters
    private_nh.param("odom_frame", odom_frame, odom_frame);
    private_nh.param("base_frame", base_frame, base_frame);
    if (!getRequiredParam(nh, "speed_gain", speed_gain))
        return;
    if (!getRequiredParam(nh, "speed_offset", speed_offset))
        return;

    private_nh.param("publish_tf", publish_tf, publish_tf);

    // create odom publisher
    odom_pub = nh.advertise<nav_msgs::Odometry>("wheel_odom", 10);

    // create tf broadcaster
    if (publish_tf)
    {
        tf_pub.reset(new tf::TransformBroadcaster);
    }

    // subscribe to vesc state and. optionally, servo command
    six_wheel_info_sub = nh.subscribe("/serial_communicator/motor_controler_info", 10, &six_wheel_odom::six_wheel_infoCallback, this);
}

void six_wheel_odom::six_wheel_infoCallback(const sixwd_msgs::SixWheelInfo::ConstPtr &motor_info)
{
    message_time = ros::Time::now();
    double sign_right = (motor_info->motor5_speed > 0) ? 1.0 : -1.0;
    double sign_left = (motor_info->motor2_speed > 0) ? 1.0 : -1.0;


    right_velocity = ((motor_info->motor5_speed) + (1.458243*sign_right)) / 143.1781;
    left_velocity = ((motor_info->motor2_speed) + (1.458243*sign_left)) / 143.1781;

    if ((abs(right_velocity)*10000)<(0.02*10000))
    {
        right_velocity=0;
        
    }
        if ((abs(left_velocity)*10000)<(0.02*10000))
    {
        left_velocity=0;
        
    }





    linear_velocity = (left_velocity+right_velocity)/2;
    angular_velocity = (left_velocity-right_velocity)/0.27;

    if((abs(angular_velocity)*10000)<(0.15*10000))
    {
        angular_velocity=0;
    }

    ROS_INFO_ONCE("Message Received");

    // use current state as last state if this is our first time here
    if (!last_state)
    {
        last_state = motor_info;
        last_message_time = message_time;
    }

    // calc elapsed time
    dt = message_time - last_message_time;

    /** @todo could probably do better propigating odometry, e.g. trapezoidal integration */

    // propigate odometry
    double xdot = linear_velocity * cos(yaw);
    double ydot = linear_velocity * sin(yaw);
    x += xdot * dt.toSec();
    y += ydot * dt.toSec();
    yaw += angular_velocity * dt.toSec();

    // save state for next time
    last_state = motor_info;
    last_message_time = message_time;

    // publish odometry message
    nav_msgs::Odometry::Ptr odom(new nav_msgs::Odometry);
    odom->header.frame_id = odom_frame;
    odom->header.stamp = message_time;
    odom->child_frame_id = base_frame;

    // Position
    odom->pose.pose.position.x = x;
    odom->pose.pose.position.y = y;
    odom->pose.pose.orientation.x = 0.0;
    odom->pose.pose.orientation.y = 0.0;
    odom->pose.pose.orientation.z = sin(yaw/2.0);
    odom->pose.pose.orientation.w = cos(yaw/2.0);

    // Position uncertainty
    /** @todo Think about position uncertainty, perhaps get from parameters? */
    odom->pose.covariance[0] = 0.02;  ///< x
    odom->pose.covariance[7] = 0.02;  ///< y
    odom->pose.covariance[35] = 0.04; ///< yaw

    // Velocity ("in the coordinate frame given by the child_frame_id")
    odom->twist.twist.linear.x = linear_velocity;
    odom->twist.twist.linear.y = 0.0;
    odom->twist.twist.angular.z = angular_velocity;

    // Velocity uncertainty
    /** @todo Think about velocity uncertainty */
    odom->twist.covariance[0] = 0.002;
    odom->twist.covariance[7] = 0.002;
    //odom->twist.covariance[14] = 0.02;
    //odom->twist.covariance[21] = 0.02;
    //odom->twist.covariance[28] = 0.02;
    odom->twist.covariance[35] = 0.002;

    if (publish_tf)
    {
        geometry_msgs::TransformStamped tf;
        tf.header.frame_id = odom_frame;
        tf.child_frame_id = base_frame;
        tf.header.stamp = ros::Time::now();
        tf.transform.translation.x = x;
        tf.transform.translation.y = y;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation = odom->pose.pose.orientation;
        if (ros::ok())
        {
            tf_pub->sendTransform(tf);
        }
    }

    if (ros::ok())
    {
        odom_pub.publish(odom);
    }
}

template <typename T>
inline bool getRequiredParam(const ros::NodeHandle &nh, std::string name, T &value)
{
    if (nh.getParam(name, value))
        return true;

    ROS_FATAL("VescToOdom: Parameter %s is required.", name.c_str());
    return false;
}
} // namespace SIXWHEELODOM