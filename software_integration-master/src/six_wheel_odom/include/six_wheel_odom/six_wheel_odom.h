#ifndef SIX_WHEEL_ODOM_H_
#define SIX_WHEEL_ODOM_H_

#include <ros/ros.h>
#include "sixwd_msgs/SixWheelCommand.h"
#include "sixwd_msgs/SixWheelInfo.h"
#include <std_msgs/Float64.h>
#include <boost/shared_ptr.hpp>
#include <tf/transform_broadcaster.h>

namespace SIXWHEELODOM
{

class six_wheel_odom
{
public:
    six_wheel_odom(ros::NodeHandle nh, ros::NodeHandle private_nh);

private:
    // ROS parameters
    std::string odom_frame;
    std::string base_frame;
    // conversion gain and offset
    double speed_gain, speed_offset;
    double steering_gain, steering_offset;
    bool publish_tf;
    //Calculation Varibles
    ros::Time message_time;
    ros::Time last_message_time;
    ros::Duration dt;
    double right_velocity;
    double left_velocity;
    double linear_velocity;
    double angular_velocity;

    // odometry state
    double x, y, yaw;
    sixwd_msgs::SixWheelInfo::ConstPtr last_state;

    // ROS services
    ros::Publisher odom_pub;
    ros::Subscriber six_wheel_info_sub;

    boost::shared_ptr<tf::TransformBroadcaster> tf_pub;

    // ROS callbacks
    void six_wheel_infoCallback(const sixwd_msgs::SixWheelInfo::ConstPtr &motor_info);
};
} // namespace SIXWHEELODOM
#endif
