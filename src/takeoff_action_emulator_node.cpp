#include "upo_actions_emulator.hpp"

using namespace UPOActionsEmulators;

int main(int argc, char** argv)
{
  //One for each robot
  ros::init(argc, argv, "makeplan_emulator_node");
  ros::NodeHandle nh;

  TakeOffActionEmulator takeoffROS(nh);

  ros::Rate r(10);

  while(ros::ok())
  {
	takeoffROS.process();

	ros::spinOnce();
	r.sleep();
  }

  return 0;
}
