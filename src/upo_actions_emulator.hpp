#ifndef UPO_ACTIONS_EMULATOR_HPP
#define UPO_ACTIONS_EMULATOR_HPP

//STD
#include <random>

// ros
#include <ros/ros.h>


#include <actionlib/server/simple_action_server.h>
#include <upo_actions/MakePlanAction.h>
#include <upo_actions/FireExtinguishAction.h>
#include <upo_actions/WindowDetectionAction.h>
#include <upo_actions/TakeOffAction.h>
#include <upo_actions/LandingAction.h>

namespace UPOActionsEmulators
{

//TODO: This can be done better by creating some base classes depending on the abstract classes from actionlib

class MakePlanActionEmulator
{

public:

 MakePlanActionEmulator(ros::NodeHandle nh): action_(nh, "Make_Plan", false)
{
	ros::NodeHandle nhp("~");
	float duration_of_action;
  	//Get parameters from configuration file
  	nhp.param<float>("success_percentaje", success_rate_,1.0);
	nhp.param<float>("duration", duration_of_action,10.0);

	action_duration_ = ros::Duration(duration_of_action);

	//Start action servers
	//register the goal and feeback callbacks
  	action_.registerGoalCallback(boost::bind(&MakePlanActionEmulator::goalCB, this));
  	action_.registerPreemptCallback(boost::bind(&MakePlanActionEmulator::preemptCB, this));

  	action_.start();

}


void process(void)
{
	upo_actions::MakePlanFeedback action_feedback_;
  	upo_actions::MakePlanResult action_result_;

	//ROS_INFO("Processing...");
	if (action_.isActive())
	{
		ros::Duration action_current_duration_ = ros::Time::now() - action_start_time_;

		//Return the adequate feedback, if any		
		//fdas_feedback_.current_time.data = fdas_current_duration_;
		//fdas_.publishFeedback(fdas_feedback_);

		//
		if(action_current_duration_ > action_duration_)
		{
			action_result_.replan_number.data = 0;
			action_result_.emergency_stop_times.data = 0;
			action_result_.time_spent.data = action_current_duration_;
			action_result_.not_possible = false;

			if(success_rate_>random_number_)
			{
				action_result_.finished = true;
				action_.setSucceeded(action_result_);
			}else
			{
				action_result_.finished = false;
				action_.setAborted(action_result_);
			}
		}

	}

}

	
private:

  //Defined success rate of the node
  float success_rate_;
  //Random number to emulate it
  float random_number_;

  
  actionlib::SimpleActionServer<upo_actions::MakePlanAction> action_;
  //Duration of the action. After this time, if the random_number is over the success_rate_ the action will succeed. It will fail otherwise
  ros::Duration action_duration_;
  ros::Time action_start_time_;



//! Goal received
void goalCB()
  {
    // reset helper variables
    action_start_time_ = ros::Time::now();
   
    // accept the new goal
   // action_duration_ = action_.acceptNewGoal()->detect_time.data;
    action_.acceptNewGoal();

	//TODO: improve this
    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(0.0,1.0);
    random_number_ = distribution(generator);

    ROS_INFO("Make_Plan emulator: Goal Received");
  

    //Preempts received for the new goal between checking if isNewGoalAvailable or invocation of a goal callback and the acceptNewGoal call will not trigger a preempt callback. This means, isPreemptRequested should be called after accepting the goal even for callback-based implementations to make sure the new goal does not have a pending preempt request.

   if(action_.isPreemptRequested())
	action_.setPreempted();


  }


void preemptCB()
  {
    //ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    action_.setPreempted();
  }


};



class FireExtinguishActionEmulator
{

public:

 FireExtinguishActionEmulator(ros::NodeHandle nh): action_(nh, "blanket_fire_extinguisher", false)
{
	ros::NodeHandle nhp("~");
	float duration_of_action;
  	//Get parameters from configuration file
  	nhp.param<float>("success_percentaje", success_rate_,1.0);
	nhp.param<float>("duration", duration_of_action,10.0);

	action_duration_ = ros::Duration(duration_of_action);

	//Start action servers
	//register the goal and feeback callbacks
  	action_.registerGoalCallback(boost::bind(&FireExtinguishActionEmulator::goalCB, this));
  	action_.registerPreemptCallback(boost::bind(&FireExtinguishActionEmulator::preemptCB, this));

  	action_.start();

}


void process(void)
{
	upo_actions::FireExtinguishFeedback action_feedback_;
  	upo_actions::FireExtinguishResult action_result_;

	//ROS_INFO("Processing...");
	if (action_.isActive())
	{
		ros::Duration action_current_duration_ = ros::Time::now() - action_start_time_;

		//Return the adequate feedback, if any		
		//fdas_feedback_.current_time.data = fdas_current_duration_;
		//fdas_.publishFeedback(fdas_feedback_);

		//
		if(action_current_duration_ > action_duration_)
		{

			if(success_rate_>random_number_)
			{
				action_result_.fire_finded = true;
				action_.setSucceeded(action_result_);
			}else
			{
				action_result_.fire_finded = false;
				action_.setAborted(action_result_);
			}
		}

	}

}

	
private:

  //Defined success rate of the node
  float success_rate_;
  //Random number to emulate it
  float random_number_;
  

  actionlib::SimpleActionServer<upo_actions::FireExtinguishAction> action_;
  //Duration of the action. After this time, if the random_number is over the success_rate_ the action will succeed. It will fail otherwise
  ros::Duration action_duration_;
  ros::Time action_start_time_;

  



//! Goal received
void goalCB()
  {
    // reset helper variables
    action_start_time_ = ros::Time::now();
   
    // accept the new goal

    action_.acceptNewGoal();

	//TODO: improve this
    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(0.0,1.0);
    random_number_ = distribution(generator);

    ROS_INFO("Make_Plan emulator: Goal Received");
  

    //Preempts received for the new goal between checking if isNewGoalAvailable or invocation of a goal callback and the acceptNewGoal call will not trigger a preempt callback. This means, isPreemptRequested should be called after accepting the goal even for callback-based implementations to make sure the new goal does not have a pending preempt request.

   if(action_.isPreemptRequested())
   {

	action_.setPreempted();
   }

  }


void preemptCB()
  {
    //ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
   

    action_.setPreempted();
    
  }


};



class WindowDetectionActionEmulator
{

public:

 WindowDetectionActionEmulator(ros::NodeHandle nh): action_(nh, "window_detector", false)
{
	ros::NodeHandle nhp("~");
	float duration_of_action;
  	//Get parameters from configuration file
  	nhp.param<float>("success_percentaje", success_rate_,1.0);
	nhp.param<float>("duration", duration_of_action,10.0);

	action_duration_ = ros::Duration(duration_of_action);

	//Start action servers
	//register the goal and feeback callbacks
  	action_.registerGoalCallback(boost::bind(&WindowDetectionActionEmulator::goalCB, this));
  	action_.registerPreemptCallback(boost::bind(&WindowDetectionActionEmulator::preemptCB, this));

  	action_.start();

}


void process(void)
{
	upo_actions::WindowDetectionFeedback action_feedback_;
  	upo_actions::WindowDetectionResult action_result_;

	//ROS_INFO("Processing...");
	if (action_.isActive())
	{
		ros::Duration action_current_duration_ = ros::Time::now() - action_start_time_;		

		//Return the adequate feedback, if any		
		action_feedback_.current_time.data = action_current_duration_;
		action_.publishFeedback(action_feedback_);

		//If we arrive here, fire was not detected, set cancelled
		if(action_current_duration_ > detection_duration_)
		{
			action_result_.window_found = false;
			action_.setAborted(action_result_);
		}
		else if (action_current_duration_ > action_duration_)
		{
			if(success_rate_>random_number_)
			{
				action_result_.window_found = true;
				action_.setSucceeded(action_result_);
			}else
			{
				action_result_.window_found = false;
				action_.setAborted(action_result_);
			}
		}

	}

}

	
private:

  //Defined success rate of the node
  float success_rate_;
  //Random number to emulate it
  float random_number_;
  

  actionlib::SimpleActionServer<upo_actions::WindowDetectionAction> action_;
  //Duration of the action. After this time, if the random_number is over the success_rate_ the action will succeed. It will fail otherwise
  ros::Duration action_duration_;
  ros::Time action_start_time_;

  ros::Duration detection_duration_;


//! Goal received
void goalCB()
  {
    // reset helper variables
    action_start_time_ = ros::Time::now();
   
    // accept the new goal
    detection_duration_ = action_.acceptNewGoal()->detect_time.data;

	//TODO: improve this
    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(0.0,1.0);
    random_number_ = distribution(generator);

    ROS_INFO("Window_detection emulator: Goal Received");
  

    //Preempts received for the new goal between checking if isNewGoalAvailable or invocation of a goal callback and the acceptNewGoal call will not trigger a preempt callback. This means, isPreemptRequested should be called after accepting the goal even for callback-based implementations to make sure the new goal does not have a pending preempt request.

    if(action_.isPreemptRequested())
   {
	upo_actions::WindowDetectionResult action_result_;
    	action_result_.window_found = false;

	action_.setPreempted(action_result_);
   }


  }


void preemptCB()
  {
    //ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    upo_actions::WindowDetectionResult action_result_;
    action_result_.window_found = false;

    action_.setPreempted(action_result_);
  }


};



class TakeOffActionEmulator
{

public:

 TakeOffActionEmulator(ros::NodeHandle nh): action_(nh, "TakeOff", false)
{
	ros::NodeHandle nhp("~");
	float duration_of_action;
  	//Get parameters from configuration file
  	nhp.param<float>("success_percentaje", success_rate_,1.0);
	nhp.param<float>("duration", duration_of_action,10.0);

	action_duration_ = ros::Duration(duration_of_action);

	//Start action servers
	//register the goal and feeback callbacks
  	action_.registerGoalCallback(boost::bind(&TakeOffActionEmulator::goalCB, this));
  	action_.registerPreemptCallback(boost::bind(&TakeOffActionEmulator::preemptCB, this));

  	action_.start();

}


void process(void)
{
	upo_actions::TakeOffFeedback action_feedback_;
  	upo_actions::TakeOffResult action_result_;

	//ROS_INFO("Processing...");
	if (action_.isActive())
	{
		ros::Duration action_current_duration_ = ros::Time::now() - action_start_time_;

		//Return the adequate feedback, if any		
		//fdas_feedback_.current_time.data = fdas_current_duration_;
		//fdas_.publishFeedback(fdas_feedback_);

		//
		if(action_current_duration_ > action_duration_)
		{
			action_result_.extra_info = "Took off";

			if(success_rate_>random_number_)
			{
				action_result_.success = true;
				action_.setSucceeded(action_result_);
			}else
			{
				action_result_.success = false;
				action_.setAborted(action_result_);
			}
		}

	}

}

	
private:

  //Defined success rate of the node
  float success_rate_;
  //Random number to emulate it
  float random_number_;

  
  actionlib::SimpleActionServer<upo_actions::TakeOffAction> action_;
  //Duration of the action. After this time, if the random_number is over the success_rate_ the action will succeed. It will fail otherwise
  ros::Duration action_duration_;
  ros::Time action_start_time_;



//! Goal received
void goalCB()
  {
    // reset helper variables
    action_start_time_ = ros::Time::now();
   
    // accept the new goal
   // action_duration_ = action_.acceptNewGoal()->detect_time.data;
    action_.acceptNewGoal();

	//TODO: improve this
    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(0.0,1.0);
    random_number_ = distribution(generator);

    ROS_INFO("TakeOff emulator: Goal Received");
  

    //Preempts received for the new goal between checking if isNewGoalAvailable or invocation of a goal callback and the acceptNewGoal call will not trigger a preempt callback. This means, isPreemptRequested should be called after accepting the goal even for callback-based implementations to make sure the new goal does not have a pending preempt request.

   if(action_.isPreemptRequested())
	action_.setPreempted();


  }


void preemptCB()
  {
    //ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    action_.setPreempted();
  }


};


class LandingActionEmulator
{

public:

 LandingActionEmulator(ros::NodeHandle nh): action_(nh, "Landing", false)
{
	ros::NodeHandle nhp("~");
	float duration_of_action;
  	//Get parameters from configuration file
  	nhp.param<float>("success_percentaje", success_rate_,1.0);
	nhp.param<float>("duration", duration_of_action,10.0);

	action_duration_ = ros::Duration(duration_of_action);

	//Start action servers
	//register the goal and feeback callbacks
  	action_.registerGoalCallback(boost::bind(&LandingActionEmulator::goalCB, this));
  	action_.registerPreemptCallback(boost::bind(&LandingActionEmulator::preemptCB, this));

  	action_.start();

}


void process(void)
{
	upo_actions::LandingFeedback action_feedback_;
  	upo_actions::LandingResult action_result_;

	//ROS_INFO("Processing...");
	if (action_.isActive())
	{
		ros::Duration action_current_duration_ = ros::Time::now() - action_start_time_;

		//Return the adequate feedback, if any		
		//fdas_feedback_.current_time.data = fdas_current_duration_;
		//fdas_.publishFeedback(fdas_feedback_);

		//
		if(action_current_duration_ > action_duration_)
		{
			action_result_.extra_info = "Took off";

			if(success_rate_>random_number_)
			{
				action_result_.success = true;
				action_.setSucceeded(action_result_);
			}else
			{
				action_result_.success = false;
				action_.setAborted(action_result_);
			}
		}

	}

}

	
private:

  //Defined success rate of the node
  float success_rate_;
  //Random number to emulate it
  float random_number_;

  
  actionlib::SimpleActionServer<upo_actions::LandingAction> action_;
  //Duration of the action. After this time, if the random_number is over the success_rate_ the action will succeed. It will fail otherwise
  ros::Duration action_duration_;
  ros::Time action_start_time_;



//! Goal received
void goalCB()
  {
    // reset helper variables
    action_start_time_ = ros::Time::now();
   
    // accept the new goal
   // action_duration_ = action_.acceptNewGoal()->detect_time.data;
    action_.acceptNewGoal();

	//TODO: improve this
    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(0.0,1.0);
    random_number_ = distribution(generator);

    ROS_INFO("Landing emulator: Goal Received");
  

    //Preempts received for the new goal between checking if isNewGoalAvailable or invocation of a goal callback and the acceptNewGoal call will not trigger a preempt callback. This means, isPreemptRequested should be called after accepting the goal even for callback-based implementations to make sure the new goal does not have a pending preempt request.

   if(action_.isPreemptRequested())
	action_.setPreempted();


  }


void preemptCB()
  {
    //ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    action_.setPreempted();
  }


};



}


#endif



