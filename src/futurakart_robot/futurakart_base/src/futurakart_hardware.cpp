
#include <boost/assign.hpp>
#include "futurakart_base/futurakart_hardware.h"

namespace futurakart_base
{

//*********************************************************************************************************************

FuturakartHardware::FuturakartHardware()
{
    ros::V_string joint_names = boost::assign::list_of("front_left_wheel")
                    ("front_right_wheel")("rear_left_wheel")("rear_right_wheel");

    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
        hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                     &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
        joint_state_interface_.registerHandle(joint_state_handle);

        hardware_interface::JointHandle joint_handle(joint_state_handle, &joints_[i].velocity_command);
        velocity_joint_interface_.registerHandle(joint_handle);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);

    feedback_sub_ = nh_.subscribe("feedback", 1, &FuturakartHardware::feedbackCallback, this);

    // Realtime publisher, initializes differently from regular ros::Publisher
    cmd_drive_pub_.init(nh_, "cmd_drive", 1);
}

//*********************************************************************************************************************
/**
 * Populates the internal joint state struct from the most recent Feedback message
 * received from the MCU.
 *
 * Called from the controller thread.
 */
void FuturakartHardware::copyJointsFromHardware()
{
    boost::mutex::scoped_lock feedback_msg_lock(feedback_msg_mutex_, boost::try_to_lock);
    if (feedback_msg_ && feedback_msg_lock)
    {
        for (int i = 0; i < 4; i++)
        {
            joints_[i].position = feedback_msg_->drivers[i % 2].measured_travel;
            joints_[i].velocity = feedback_msg_->drivers[i % 2].measured_velocity;
            joints_[i].effort = 0;
        }
    }
}

//*********************************************************************************************************************
/**
 * Populates and publishes Drive message based on the controller outputs.
 *
 * Called from the controller thread.
 */
void FuturakartHardware::publishDriveFromController()
{
    if (cmd_drive_pub_.trylock())
    {
        cmd_drive_pub_.msg_.mode = futurakart_msgs::Drive::MODE_VELOCITY;
        cmd_drive_pub_.msg_.drivers[futurakart_msgs::Drive::LEFT] = joints_[0].velocity_command;
        cmd_drive_pub_.msg_.drivers[futurakart_msgs::Drive::RIGHT] = joints_[1].velocity_command;
        cmd_drive_pub_.unlockAndPublish();
    }
}

//*********************************************************************************************************************

void FuturakartHardware::feedbackCallback(const futurakart_msgs::Feedback::ConstPtr& msg)
{
  // Update the feedback message pointer to point to the current message. Block
  // until the control thread is not using the lock.
  boost::mutex::scoped_lock lock(feedback_msg_mutex_);
  feedback_msg_ = msg;
}

//*********************************************************************************************************************

}  // namespace futurakart_base
