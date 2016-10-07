
#include <string>
#include <sys/types.h>
#include <ifaddrs.h>

#include "boost/algorithm/string/predicate.hpp"
#include "diagnostic_updater/update_functions.h"
#include "futurakart_base/futurakart_diagnostic_updater.h"


namespace futurakart_base
{

//*********************************************************************************************************************

FuturakartDiagnosticUpdater::FuturakartDiagnosticUpdater()
{
    setHardwareID("unknown");

    add("General", this, &FuturakartDiagnosticUpdater::generalDiagnostics);
    add("Battery", this, &FuturakartDiagnosticUpdater::batteryDiagnostics);
    add("User voltage supplies", this, &FuturakartDiagnosticUpdater::voltageDiagnostics);
    add("Current consumption", this, &FuturakartDiagnosticUpdater::currentDiagnostics);
    add("Power consumption", this, &FuturakartDiagnosticUpdater::powerDiagnostics);

    // The arrival of this message runs the update() method and triggers the above callbacks.
    status_sub_ = nh_.subscribe("status", 5, &FuturakartDiagnosticUpdater::statusCallback, this);

    // Publish whether the wireless interface has an IP address every second.
    ros::param::param<std::string>("~wireless_interface", wireless_interface_, "wlan0");
    ROS_INFO_STREAM("Checking for wireless connectivity on interface: " << wireless_interface_);
    wifi_connected_pub_ = nh_.advertise<std_msgs::Bool>("wifi_connected", 1);
    wireless_monitor_timer_ = nh_.createTimer(ros::Duration(1.0), &FuturakartDiagnosticUpdater::wirelessMonitorCallback, this);
}

//*********************************************************************************************************************

void FuturakartDiagnosticUpdater::generalDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    stat.add("MCU uptime (s)", last_status_->mcu_uptime.toSec());
    stat.addf("Motor drivers energized", "%s", last_status_->drivers_active ? "true" : "false");

    if (last_status_->driver_external_stop_present)
    {
        if (last_status_->driver_external_stop_stopped)
        {
            stat.add("External stop", "Present, asserting stop");
        }
        else
        {
            stat.add("External stop", "Present, not asserting stop");
        }
    }
    else
    {
        stat.add("External stop", "Absent");
    }

    if (last_status_->driver_external_stop_stopped)
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "External stop device asserting motor stop.");
    }
    else if (!last_status_->drivers_active)
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Motor drivers not energized.");
    }
    else
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "System OK.");
    }
}

//*********************************************************************************************************************

void FuturakartDiagnosticUpdater::batteryDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    stat.add("Battery Voltage (V)", last_status_->measured_battery);

    if (last_status_->measured_battery > 30.0)
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery overvoltage.");
    }
    else if (last_status_->measured_battery < 1.0)
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery voltage not detected, check BATT fuse.");
    }
    else if (last_status_->measured_battery < 20.0)
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery critically under voltage.");
    }
    else if (last_status_->measured_battery < 24.0)
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Battery low voltage.");
    }
    else
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Battery OK.");
    }
}

//*********************************************************************************************************************

void FuturakartDiagnosticUpdater::voltageDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    stat.add("12V Supply (V)", last_status_->measured_12v);
    stat.add("5V Supply (V)", last_status_->measured_5v);

    if (last_status_->measured_12v > 12.5 || last_status_->measured_5v > 5.5)
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
            "User supply overvoltage. Accessories may be damaged.");
    }
    else if (last_status_->measured_12v < 1.0 || last_status_->measured_5v < 1.0)
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "User supplies absent. Check tray fuses.");
    }
    else if (last_status_->measured_12v < 11.0 || last_status_->measured_5v < 4.0)
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Voltage supplies undervoltage. Check loading levels.");
    }
    else
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "User supplies OK.");
    }
}

//*********************************************************************************************************************

void FuturakartDiagnosticUpdater::currentDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    stat.add("Drive current (A)", last_status_->drive_current);
    stat.add("User current (A)", last_status_->user_current);
    stat.add("Computer current (A)", last_status_->computer_current);
    stat.add("Total current (A)", last_status_->total_current);

    if (last_status_->total_current > 32.0)
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Current draw critical.");
    }
    else if (last_status_->total_current > 20.0)
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Current draw warning.");
    }
    else if (last_status_->total_current > 10.0)
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Current draw requires monitoring.");
    }
    else
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Current draw nominal.");
    }
}

//*********************************************************************************************************************

void FuturakartDiagnosticUpdater::powerDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    stat.add("Total power consumption (Wh)", last_status_->total_power_consumed);

    if (last_status_->total_power_consumed > 260.0)
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Power consumed exceeds capacity of standard battery.");
    }
    else if (last_status_->total_power_consumed > 220.0)
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Power consumed approaches capacity of standard battery.");
    }
    else
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Battery OK.");
    }
}

//*********************************************************************************************************************

void FuturakartDiagnosticUpdater::statusCallback(const futurakart_msgs::Status::ConstPtr& status)
{
    // Fresh data from the MCU, publish a diagnostic update.
    last_status_ = status;
    setHardwareID(last_status_->hardware_id);
    update();
}

//*********************************************************************************************************************

void FuturakartDiagnosticUpdater::wirelessMonitorCallback(const ros::TimerEvent& te)
{
    std_msgs::Bool wifi_connected_msg;
    wifi_connected_msg.data = false;

    // Get system structure of interface IP addresses.
    struct ifaddrs* ifa_head;
    if (getifaddrs(&ifa_head) != 0)
    {
        ROS_WARN("System call getifaddrs returned error code. Unable to detect network interfaces.");
        return;
    }

    // Iterate structure looking for the wireless interface.
    struct ifaddrs* ifa_current = ifa_head;
    while (ifa_current != NULL)
    {
        if (strcmp(ifa_current->ifa_name, wireless_interface_.c_str()) == 0)
        {
            int family = ifa_current->ifa_addr->sa_family;
            if (family == AF_INET || family == AF_INET6)
            {
                wifi_connected_msg.data = true;
                break;
            }
        }

        ifa_current = ifa_current->ifa_next;
    }

    // Free structure, publish result message.
    freeifaddrs(ifa_head);
    wifi_connected_pub_.publish(wifi_connected_msg);
}

//*********************************************************************************************************************

}  // namespace futurakart_base
