#ifndef FUTURAKART_HARDWARE_H
#define FUTURAKART_HARDWARE_H

// Boost
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>

// Ros controls
#include <hardware_interface/robot_hw.h>

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "realtime_tools/realtime_publisher.h"


// Transmission mode : !!! NOT USED
//#include <hardware_interface/actuator_command_interface.h>
//#include <hardware_interface/actuator_state_interface.h>
//#include <transmission_interface/robot_transmissions.h>
//#include <transmission_interface/transmission_interface_loader.h>
// END Transmission mode : !!! NOT USED




namespace futurakart_base
{

class FuturakartHardware : public hardware_interface::RobotHW
{

public:
  FuturakartHardware();
  void read(ros::Time time, ros::Duration period);
  void update(ros::Time time, ros::Duration period);
  void write(ros::Time time, ros::Duration period);

private:

  void feedbackCallback(const geometry_msgs::Pose2D::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Subscriber feedback_sub_;
  realtime_tools::RealtimePublisher<geometry_msgs::Pose2D> cmd_drive_pub_;
  // This pointer is set from the ROS thread.
  geometry_msgs::Pose2D::ConstPtr feedback_msg_;
  boost::mutex feedback_msg_mutex_;

  std::string robot_description_;

  //diagnostic_updater::Updater diag;
  //diagnostic_updater::FrequencyStatus diag_freq;
  //double diag_freq_min;
  //double diag_freq_max;

  // Transmission mode : !!! NOT USED
  //	transmission_interface::RobotTransmissions transmissions_;
  //	boost::scoped_ptr<transmission_interface::TransmissionInterfaceLoader> transmission_loader_;
  //	hardware_interface::PositionActuatorInterface actuator_position_interface_;
  //	hardware_interface::ActuatorStateInterface actuator_state_interface_;
  //	hardware_interface::VelocityActuatorInterface actuator_velocity_interface_;
  // END Transmission mode : !!! NOT USED


  // Direction joint model stores data to exchange between hardware and hardware interface
  struct DirectionJoint
  {
    double pos;
    double cmd;
    DirectionJoint() : pos(0.0), cmd(0.0) {}
  }
  direction_joint_;

  // Propulstion joint model stores data to exchange between hardware and hardware interface
  struct PropulsionJoint
  {
    double vel;
    double cmd;
    PropulsionJoint() : vel(0.0), cmd(0.0) {}
  }
  propulsion_joint_;

  hardware_interface::PositionJointInterface position_joint_interface_;
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;

};

}  // namespace futurakart_base

#endif  // FUTURAKART_HARDWARE_H
