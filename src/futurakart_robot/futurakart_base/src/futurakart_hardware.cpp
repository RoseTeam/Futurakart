
#include <boost/assign.hpp>
#include "futurakart_base/futurakart_hardware.h"

namespace futurakart_base
{

//*********************************************************************************************************************

FuturakartHardware::FuturakartHardware() :
  robot_description_("")
{

  std::string hw_feedback_topic, cmd_vel_topic;

  nh_.param("hw_feedback_topic", hw_feedback_topic, std::string("feedback"));
  nh_.param("cmd_vel_topic", cmd_vel_topic, std::string("cmd_vel"));


  // Define and register direction interface:
  ros::V_string dir_joint_names = boost::assign::list_of("front_left_wheel")("front_right_wheel");

  for (unsigned int i = 0; i < dir_joint_names.size(); i++)
  {
    hardware_interface::JointStateHandle dir_joint_state_handle(
          dir_joint_names[i],
          &direction_joint_.pos,
          &direction_joint_.vel,
          &direction_joint_.eff
          );
    joint_state_interface_.registerHandle(dir_joint_state_handle);
    hardware_interface::JointHandle dir_joint_handle(dir_joint_state_handle, &direction_joint_.cmd);
    position_joint_interface_.registerHandle(dir_joint_handle);
  }

  // Define and register propulsion interface:
  ros::V_string prop_joint_names = boost::assign::list_of("rear_left_wheel")("rear_right_wheel");
  for (unsigned int i = 0; i < prop_joint_names.size(); i++)
  {

    hardware_interface::JointStateHandle prop_joint_state_handle(
          prop_joint_names[i],
          &propulsion_joint_.pos,
          &propulsion_joint_.vel,
          &propulsion_joint_.eff
          );
    joint_state_interface_.registerHandle(prop_joint_state_handle);
    hardware_interface::JointHandle prop_joint_handle(prop_joint_state_handle, &propulsion_joint_.cmd);
    velocity_joint_interface_.registerHandle(prop_joint_handle);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);
  registerInterface(&velocity_joint_interface_);

  feedback_sub_ = nh_.subscribe(hw_feedback_topic, 1, &FuturakartHardware::feedbackCallback, this);

  // Realtime publisher, initializes differently from regular ros::Publisher
  cmd_drive_pub_.init(nh_, cmd_vel_topic, 1);


  //  nh_.param("robot_description", robot_description_, robot_description_);
  //  registerInterface(&actuator_position_interface_);
  //  registerInterface(&actuator_state_interface_);
  //  registerInterface(&actuator_velocity_interface_);
  //  transmission_loader_.reset(new transmission_interface::TransmissionInterfaceLoader(this, &transmissions_));
  //  transmission_loader_->load(robot_description_);

}
//*********************************************************************************************************************

void FuturakartHardware::read(ros::Time time, ros::Duration period)
{

  boost::mutex::scoped_lock feedback_msg_lock(feedback_msg_mutex_, boost::try_to_lock);
  if (feedback_msg_ && feedback_msg_lock)
  {
    direction_joint_.pos = feedback_msg_->theta;
    propulsion_joint_.vel = feedback_msg_->x;
  }

  //  // Read actuator_vel, actuator_pos from MBED
  //  if(transmissions_.get<transmission_interface::ActuatorToJointStateInterface>())
  //    transmissions_.get<transmission_interface::ActuatorToJointStateInterface>()->propagate();
}

//*********************************************************************************************************************

void FuturakartHardware::update(ros::Time time, ros::Duration period)
{

}

//*********************************************************************************************************************

void FuturakartHardware::write(ros::Time time, ros::Duration period)
{
  //  if(transmissions_.get<transmission_interface::JointToActuatorPositionInterface>())
  //    transmissions_.get<transmission_interface::JointToActuatorPositionInterface>()->propagate();
  //  if(transmissions_.get<transmission_interface::JointToActuatorVelocityInterface>())
  //    transmissions_.get<transmission_interface::JointToActuatorVelocityInterface>()->propagate();


  // Write cmd_vel to MBED
  if (cmd_drive_pub_.trylock())
  {
    cmd_drive_pub_.msg_.x = propulsion_joint_.cmd;
    cmd_drive_pub_.msg_.y = 0.0;
    cmd_drive_pub_.msg_.theta = direction_joint_.cmd;
    cmd_drive_pub_.unlockAndPublish();
  }
}

//*********************************************************************************************************************

void FuturakartHardware::feedbackCallback(const geometry_msgs::Pose2D::ConstPtr &msg)
{
  // Update the feedback message pointer to point to the current message. Block
  // until the control thread is not using the lock.
  boost::mutex::scoped_lock lock(feedback_msg_mutex_);
  feedback_msg_ = msg;

}

//*********************************************************************************************************************

}  // namespace futurakart_base
