#ifndef FIRE_DETECTION_ACTION_EMULATOR_HPP
#define FIRE_DETECTION_ACTION_EMULATOR_HPP

//STD
#include <random>

// ros
#include <ros/ros.h>


#include <actionlib/server/simple_action_server.h>
#include <upo_actions/FireDetectionAction.h>
#include <upo_actions/FireDetection3DAction.h>


namespace UPOActionsEmulators
{


class FireDetectionActionEmulator
{

public:

 FireDetectionActionEmulator(ros::NodeHandle nh): fdas_(nh, "firedetection", false), fd3das_(nh, "firedetection3D", false)
{

	ros::NodeHandle nhp("~");
	float duration_of_action;
  	//Get parameters from configuration file
  	nhp.param<float>("success_percentaje", success_rate_,1.0);
	nhp.param<float>("duration", duration_of_action,10.0);

	action_duration_ = ros::Duration(duration_of_action);

	//Start action servers
	//register the goal and feeback callbacks
  	fdas_.registerGoalCallback(boost::bind(&FireDetectionActionEmulator::fireDetectionGoalCB, this));
  	fdas_.registerPreemptCallback(boost::bind(&FireDetectionActionEmulator::fireDetectionPreemptCB, this));

	fd3das_.registerGoalCallback(boost::bind(&FireDetectionActionEmulator::fireDetection3DGoalCB, this));
  	fd3das_.registerPreemptCallback(boost::bind(&FireDetectionActionEmulator::fireDetection3DPreemptCB, this));

  	fdas_.start();
	fd3das_.start();

}




void process(void)
{
	upo_actions::FireDetectionFeedback fdas_feedback_;
  	upo_actions::FireDetectionResult fdas_result_;


	//ROS_INFO("Processing...");
	if (fdas_.isActive())
	{
		ros::Duration fdas_current_duration_ = ros::Time::now() - fdas_start_time_;
		fdas_feedback_.current_time.data = fdas_current_duration_;
		fdas_.publishFeedback(fdas_feedback_);

		//If we arrive here, fire was not detected, set cancelled
		if(fdas_current_duration_ > fdas_duration_)
		{
			fdas_result_.fire_found = false;
			fdas_.setAborted(fdas_result_);
		}
		else if (success_rate_>random_number_ && fdas_current_duration_ > action_duration_)
		{
			fdas_result_.fire_found = true;
			fdas_.setSucceeded(fdas_result_);
		}


	}

	upo_actions::FireDetection3DFeedback fd3das_feedback_;
  	upo_actions::FireDetection3DResult fd3das_result_;

	// This action and the previous one cannot be at the same time active
	if (fd3das_.isActive())
	{
		ros::Duration fd3das_current_duration_ = ros::Time::now() - fd3das_start_time_;
		fd3das_feedback_.current_time.data = fd3das_current_duration_;
		fd3das_.publishFeedback(fd3das_feedback_);

		//If we arrive here, fire was not detected, set cancelled
		if(fd3das_current_duration_ > fd3das_duration_)
		{
			fd3das_result_.fire_found = false;
			fd3das_.setAborted(fd3das_result_);
		}else if (success_rate_>random_number_ && fd3das_current_duration_ > action_duration_)
		{

			fd3das_result_.fire_found = true;
			fd3das_result_.fire_position.x = 2.0;
			fd3das_result_.fire_position.y = 5.0;
			fd3das_result_.fire_position.z = 1.0;

			fd3das_result_.errorX.data = 1.0;
			fd3das_result_.errorY.data = 1.0;
			fd3das_result_.errorZ.data = 1.0;
					
			fd3das_.setSucceeded(fd3das_result_);

		}

	}

}

	
private:

  //Defined success rate of the node
  float success_rate_;
  //Random number to emulate it
  float random_number_;

  //Duration of the action. After this time, if the random_number is over the success_rate_ the action will succeed. It will fail otherwise
  ros::Duration action_duration_;
  

 //Fire detection action server: returns succeeded if it detects a fire is in the image
  actionlib::SimpleActionServer<upo_actions::FireDetectionAction> fdas_;
  ros::Duration fdas_duration_;
  ros::Time fdas_start_time_;


  //Fire detection 3D action server: returns succeeded if it localizes a fire with enough precision
  actionlib::SimpleActionServer<upo_actions::FireDetection3DAction> fd3das_;
  ros::Duration fd3das_duration_;
  ros::Time fd3das_start_time_;


//! Fire detection actions
void fireDetectionGoalCB()
  {
    // reset helper variables
    fdas_start_time_ = ros::Time::now();
   
    // accept the new goal
    fdas_duration_ = fdas_.acceptNewGoal()->detect_time.data;

	//TODO: improve this
    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(0.0,1.0);
    random_number_ = distribution(generator);

    ROS_INFO("firedetection emulator: Goal Received");
  

    //Preempts received for the new goal between checking if isNewGoalAvailable or invocation of a goal callback and the acceptNewGoal call will not trigger a preempt callback. This means, isPreemptRequested should be called after accepting the goal even for callback-based implementations to make sure the new goal does not have a pending preempt request.

   if(fdas_.isPreemptRequested())
   {
	upo_actions::FireDetectionResult fdas_result_;
    	fdas_result_.fire_found = false;

    	fdas_.setPreempted(fdas_result_);
    }


   //Abort current task in the other action server, if any
   if (fd3das_.isActive())
   {
        upo_actions::FireDetection3DResult f3das_result_;
    	f3das_result_.fire_found = false;

	fd3das_.setAborted(f3das_result_);

   }

  }


void fireDetectionPreemptCB()
  {
    //ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    ROS_INFO("firedetection emulator: Preempted");

    //Return a failed result
    upo_actions::FireDetectionResult fdas_result_;
    fdas_result_.fire_found = false;

    fdas_.setPreempted(fdas_result_);
  }

//! Fire 3D detection actions
void fireDetection3DGoalCB()
  {

    ROS_INFO("firedetection3D emulator: Goal Received");

    //TODO: improve this
    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(0.0,1.0);
    random_number_ = distribution(generator);

    // reset helper variables
    fd3das_start_time_ = ros::Time::now();
   
    // accept the new goal
    fd3das_duration_ = fd3das_.acceptNewGoal()->detect_time.data;

    //Preempts received for the new goal between checking if isNewGoalAvailable or invocation of a goal callback and the acceptNewGoal call will not trigger a preempt callback. This means, isPreemptRequested should be called after accepting the goal even for callback-based implementations to make sure the new goal does not have a pending preempt request.
   if(fd3das_.isPreemptRequested())
   {
	upo_actions::FireDetection3DResult f3das_result_;
    	f3das_result_.fire_found = false;
    	fd3das_.setPreempted(f3das_result_);
   }

    //Abort current task in the other action server, if any
    if (fdas_.isActive())
    {
	upo_actions::FireDetectionResult fdas_result_;
    	fdas_result_.fire_found = false;

	fdas_.setAborted(fdas_result_);

    }

  }


void fireDetection3DPreemptCB()
  {
    //ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    ROS_INFO("firedetection3D emulator: Preempted");

    upo_actions::FireDetection3DResult f3das_result_;
    f3das_result_.fire_found = false;

    fd3das_.setPreempted(f3das_result_);
  }


};

}


#endif



