#ifndef FUTURAKART_HARDWARE_H
#define FUTURAKART_HARDWARE_H

#include "boost/thread.hpp"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "futurakart_msgs/Drive.h"
#include "futurakart_msgs/Feedback.h"
#include "realtime_tools/realtime_publisher.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"


namespace futurakart_base
{

class FuturakartHardware : public hardware_interface::RobotHW
{

public:
    FuturakartHardware();
    void copyJointsFromHardware();
    void publishDriveFromController();

private:
    void feedbackCallback(const futurakart_msgs::Feedback::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber feedback_sub_;
    realtime_tools::RealtimePublisher<futurakart_msgs::Drive> cmd_drive_pub_;

    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    // These are mutated on the controls thread only.
    struct Joint
    {
        double position;
        double velocity;
        double effort;
        double velocity_command;

        Joint() : position(0), velocity(0), effort(0), velocity_command(0)
        {
        }
    }
    joints_[4];

    // This pointer is set from the ROS thread.
    futurakart_msgs::Feedback::ConstPtr feedback_msg_;
    boost::mutex feedback_msg_mutex_;

};

}  // namespace futurakart_base

#endif  // FUTURAKART_HARDWARE_H
