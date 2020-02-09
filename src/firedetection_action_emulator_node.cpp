#include "firedetection_action_emulator.hpp"

using namespace UPOActionsEmulators;

int main(int argc, char** argv)
{
  //One for each robot
  ros::init(argc, argv, "firedetection_emulator_node");
  ros::NodeHandle nh;

  FireDetectionActionEmulator fireDetectionROS(nh);

  ros::Rate r(10);

  while(ros::ok())
  {
	fireDetectionROS.process();

	ros::spinOnce();
	r.sleep();
  }

  return 0;
}
