// STD
#include <string>

// Boost
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

// ROS & ros controls
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>


// Project
//#include "futurakart_base/futurakart_diagnostic_updater.h"
#include "futurakart_base/futurakart_hardware.h"


typedef boost::chrono::steady_clock time_source;

//*********************************************************************************************************************

void controlThread(ros::Rate rate, futurakart_base::FuturakartHardware* robot, controller_manager::ControllerManager* cm)
{
  time_source::time_point last_time = time_source::now();

  while (ros::ok())
  {
    // Calculate monotonic time elapsed
    time_source::time_point this_time = time_source::now();
    boost::chrono::duration<double> elapsed_duration = this_time - last_time;
    ros::Duration elapsed(elapsed_duration.count());
    last_time = this_time;
    ros::Time this_time_ros = ros::Time::now();

    robot->read(this_time_ros, elapsed);
    cm->update(this_time_ros, elapsed);
    robot->update(this_time_ros, elapsed);
    robot->write(this_time_ros, elapsed);
    rate.sleep();

    boost::this_thread::interruption_point();
  }
}

//*********************************************************************************************************************

int main(int argc, char* argv[])
{
  // Initialize ROS node.
  ros::init(argc, argv, "futurakart_node");
  ros::NodeHandle controller_nh("");

  // Create instance of RobotHW and Controller manager
  futurakart_base::FuturakartHardware futurakart;
  controller_manager::ControllerManager cm(&futurakart, controller_nh);

  // Create a thread to control robot :
  boost::thread(boost::bind(controlThread, ros::Rate(50), &futurakart, &cm));

  // Create diagnostic updater, to update itself on the ROS thread.
  //futurakart_base::FuturakartDiagnosticUpdater futurakart_diagnostic_updater;

  // Foreground ROS spinner for ROS callbacks, including rosserial, diagnostics
  ros::spin();

  return 0;
}
