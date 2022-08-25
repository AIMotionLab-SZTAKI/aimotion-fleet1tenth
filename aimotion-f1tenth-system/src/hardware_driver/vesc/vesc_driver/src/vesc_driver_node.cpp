#include <ros/ros.h>
#include <unistd.h>
#include <signal.h>

#include "vesc_driver/vesc_driver.h"


void WaitSigintHandler(int sig)
{
  // sleeps for 1 second before shutdown to let higher level processes finish first
  sleep(1);
  ros::shutdown();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "vesc_driver_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  signal(SIGINT, WaitSigintHandler);

  vesc_driver::VescDriver vesc_driver(nh, private_nh);

  ros::spin();

  return 0;
}
