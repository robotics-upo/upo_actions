#include "upo_actions_emulator.hpp"

using namespace UPOActionsEmulators;

int main(int argc, char** argv)
{
  //One for each robot
  ros::init(argc, argv, "windowdetection_emulator_node");
  ros::NodeHandle nh;

  WindowDetectionActionEmulator windowDetectorROS(nh);

  ros::Rate r(10);

  while(ros::ok())
  {
	windowDetectorROS.process();

	ros::spinOnce();
	r.sleep();
  }

  return 0;
}
