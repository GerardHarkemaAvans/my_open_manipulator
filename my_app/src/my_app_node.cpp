#include "my_app/behavior/behavior.h"

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "behavior_node");
  ros::NodeHandle node_handle("");
  std::cout << "my_app started" << std::endl;
  behavior my_behavior;

  my_behavior.onEnter();

  while (ros::ok())
  {
    ros::spinOnce();
    if(my_behavior.execute()){
      my_behavior.onExit();
      break;
    }
  }

  return 0;
}
