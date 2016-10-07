#ifndef FUTURAKART_DIAGNOSTIC_UPDATER_H
#define FUTURAKART_DIAGNOSTIC_UPDATER_H

#include <string>

#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/publisher.h"
#include "futurakart_msgs/Status.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"

namespace futurakart_base
{

class FuturakartDiagnosticUpdater : private diagnostic_updater::Updater
{

public:
    FuturakartDiagnosticUpdater();

    void generalDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void batteryDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void voltageDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void currentDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void powerDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

    void statusCallback(const futurakart_msgs::Status::ConstPtr& status);
    void wirelessMonitorCallback(const ros::TimerEvent& te);

private:
    ros::NodeHandle nh_;
    ros::Timer timer_;
    ros::Subscriber status_sub_;
    futurakart_msgs::Status::ConstPtr last_status_;

    std::string wireless_interface_;
    ros::Timer wireless_monitor_timer_;
    ros::Publisher wifi_connected_pub_;

};

}  // namespace futurakart_base

#endif  // FUTURAKART_DIAGNOSTIC_UPDATER_H
