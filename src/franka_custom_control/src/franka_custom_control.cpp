#include "ros/ros.h"
#include "std_msgs/String.h"
#include <franka/robot.h>
//#include <franka_hw/franka_hw.h>


//franka_hw::FrankaHW franka_control;
franka::Robot robot;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  robot.stop();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle public_node_handle;
  ros::NodeHandle node_handle("~");
  /**if (!franka_control.init(public_node_handle, node_handle)) {
    ROS_ERROR("franka_control_node: Failed to initialize FrankaHW class. Shutting down!");
    return 1;
  }**/

  ros::Subscriber sub = public_node_handle.subscribe("chatter", 1000, chatterCallback);

  //robot = franka_control.robot();

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
