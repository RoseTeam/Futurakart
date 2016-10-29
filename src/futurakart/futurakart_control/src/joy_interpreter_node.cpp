#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;
  ros::ServiceClient client;

  //joy_interpreter::ChangeState srv;
 

  int linear_, angular_;
  double l_scale_, a_scale_;
  bool start_stop_state;
  
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  
};


TeleopTurtle::TeleopTurtle():  linear_(1),  angular_(0), a_scale_(0.4), l_scale_(0.6)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);

 // client = nh_.serviceClient<joy_interpreter::ChangeState>("change_state_rose_bot");

  //srv.request.state = 0;
  //srv.request.start_stop = 0;
  start_stop_state = false;

}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  vel.angular.z = a_scale_*joy->axes[angular_];
  vel.linear.x = l_scale_*joy->axes[linear_];
  vel_pub_.publish(vel);

  static bool last_state_button3 = false;

  if(joy->buttons[3])
  { 
	  /*if(last_state_button3 == false){
		  start_stop_state = !start_stop_state;
		  srv.request.state = start_stop_state;
		  if (client.call(srv))
		  {
		    ROS_INFO("Start sent");
		  }
	   }*/
	last_state_button3 = true;		
  }
  else {
   last_state_button3 = false;
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ps3_twist");
  ROS_INFO("Launching PS3 Controller node !!");
  TeleopTurtle teleop_turtle;

  ros::spin();
}
