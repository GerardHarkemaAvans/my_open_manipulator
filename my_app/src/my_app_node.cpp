#include "my_app/behavior/behavior.h"

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "behavior_node");
  ros::NodeHandle node_handle("");

  behavior my_behavior;

  while (ros::ok())
  {
    ros::spinOnce();
    if(my_behavior.getStatus()) break;
  }

  return 0;
}
