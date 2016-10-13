
#include <boost/assign.hpp>
#include "futurakart_base/futurakart_hardware.h"

namespace futurakart_base
{

//*********************************************************************************************************************

FuturakartHardware::FuturakartHardware() :
    robot_description_("")
{
	nh.param("robot_description", robot_description_, robot_description_);
	registerInterface(&actuator_position_interface_);
	registerInterface(&actuator_state_interface_);
	registerInterface(&actuator_velocity_interface_);

	transmission_loader.reset(new transmission_interface::TransmissionInterfaceLoader(this, &transmissions_));
	transmission_loader->load(robot_description_);

}
//*********************************************************************************************************************

void FuturakartHardware::read(ros::Time time, ros::Duration period)
{
    // Read actuator_vel, actuator_pos from MBED   
    
	if(transmissions.get<transmission_interface::ActuatorToJointStateInterface>())
		transmissions.get<transmission_interface::ActuatorToJointStateInterface>()->propagate();
}

//*********************************************************************************************************************

void FuturakartHardware::update(ros::Time time, ros::Duration period)
{

}

//*********************************************************************************************************************

void FuturakartHardware::write(ros::Time time, ros::Duration period)
{
	if(transmissions.get<transmission_interface::JointToActuatorPositionInterface>())
		transmissions.get<transmission_interface::JointToActuatorPositionInterface>()->propagate();
	if(transmissions.get<transmission_interface::JointToActuatorVelocityInterface>())
		transmissions.get<transmission_interface::JointToActuatorVelocityInterface>()->propagate();

    // Write actuator_cmd_vel to MBED
    
}

//*********************************************************************************************************************

}  // namespace futurakart_base
