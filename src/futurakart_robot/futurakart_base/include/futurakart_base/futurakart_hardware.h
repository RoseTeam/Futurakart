#ifndef FUTURAKART_HARDWARE_H
#define FUTURAKART_HARDWARE_H

// Ros controls
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface_loader.h>


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
    void feedbackCallback(const futurakart_msgs::Feedback::ConstPtr& msg);

	ros::NodeHandle nh_;
	ros::NodeHandle nh_priv_;
	std::string robot_description_;
	//diagnostic_updater::Updater diag;
	//diagnostic_updater::FrequencyStatus diag_freq;
	//double diag_freq_min;
	//double diag_freq_max;

	transmission_interface::RobotTransmissions transmissions_;
	boost::scoped_ptr<transmission_interface::TransmissionInterfaceLoader> transmission_loader_;

	hardware_interface::PositionActuatorInterface actuator_position_interface_;
	hardware_interface::ActuatorStateInterface actuator_state_interface_;
	hardware_interface::VelocityActuatorInterface actuator_velocity_interface_;

};

}  // namespace futurakart_base

#endif  // FUTURAKART_HARDWARE_H
