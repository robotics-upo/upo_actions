#include "upo_actions_emulator.hpp"

using namespace UPOActionsEmulators;

int main(int argc, char** argv)
{
  //One for each robot
  ros::init(argc, argv, "makeplan_emulator_node");
  ros::NodeHandle nh;

  MakePlanActionEmulator plannerROS(nh);

  ros::Rate r(10);

  while(ros::ok())
  {
	plannerROS.process();

	ros::spinOnce();
	r.sleep();
  }

  return 0;
}
