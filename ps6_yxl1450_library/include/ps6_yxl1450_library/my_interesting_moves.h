// my_interesting_moves.h header file
/// yangliu28, Oct, 2015.
/// Include this file in "my_interesting_moves.cpp", and in any main that uses this library.
/// This class presents three interesting (I hope so) moves of baxter

#ifndef MY_INTERESTING_MOVES_H_
#define MY_INTERESTING_MOVES_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_traj_streamer/baxter_traj_streamer.h>
#include <baxter_traj_streamer/trajAction.h>
#include <math.h>

using namespace std;
const Vectorq7x1 LOWER_LIMIT_DEG = (Vectorq7x1() <<	-141.0, -123.0, -173.5, -3.0, -175.25, -90.0, -175.25).finished();
const Vectorq7x1 UPPER_LIMIT_DEG = (Vectorq7x1() <<	51.0, 60.0, 173.5, 150.0, 175.25, 120.0, 175.25).finished();
const Vectorq7x1 LOWER_LIMIT_RAD = LOWER_LIMIT_DEG * M_PI / 180.0;
const Vectorq7x1 UPPER_LIMIT_RAD = UPPER_LIMIT_DEG * M_PI / 180.0;

// define a class, including a constructor, member variables and member functions
class MyInterestingMoves
{
public:
	MyInterestingMoves(ros::NodeHandle* nodehandle); // constructor

	/**
	 * Move 1: all 7 joints of right arm will be set to zero.
	 * This is for the test of joint origin
	 */
	void all_joint_1_origin();

	/**
	 * Move 2: all 7 joints will go to its upper limit.
	 * This is for the test of upper limit
	 */
	void all_joint_2_upper_limit();

    /**
     * Move 3: all 7 joints will go to its lower limit.
     * This is for the test of lower limit
     */
    void all_joint_3_lower_limit();

private:
	void send_joints_as_goal(Vectorq7x1 q_end_pose);
	// callback for the action client

	ros::NodeHandle nh_;
	int g_count;

	// need an action client to talk to the action server
	actionlib::SimpleActionClient<baxter_traj_streamer::trajAction> action_client;
};

#endif
