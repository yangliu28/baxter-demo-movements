// Copyright [2015] <Copyright yangliu28>
// my_interesting_moves: implementation of class MyInterestingMoves

#include <ps6_yxl1450_library/my_interesting_moves.h>
#include <vector>

MyInterestingMoves::MyInterestingMoves(ros::NodeHandle* nodehandle):
action_client("trajActionServer", true)
{
    nh_ = *nodehandle;  // dereference the pointer and pass the value
    g_count = 0;

    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0));  // wait for up to 5 seconds
    // retry connecting if not connected
    if (!server_exists)
    {
        ROS_WARN("could not connect to server; will wait forever");
        return;  // bail out; optionally, could print a warning message and retry
    }
    server_exists = action_client.waitForServer();  // wait forever
    // if here, then connected to the server
    ROS_INFO("connected to action server");
}

// my interesting move 1
void MyInterestingMoves::all_joint_1_origin()
{
    Vectorq7x1 origin_pose;
    origin_pose << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;  // set all joints to zero
    send_joints_as_goal(origin_pose);
}

// my interesting move 2
void MyInterestingMoves::all_joint_2_upper_limit()
{
    Vectorq7x1 upper_limit_pose;
    upper_limit_pose = UPPER_LIMIT_RAD;  // UPPER_LIMIT_RAD is defined in the header file
    send_joints_as_goal(upper_limit_pose);
}

// my interesting move 3
void MyInterestingMoves::all_joint_3_lower_limit()
{
    Vectorq7x1 lower_limit_pose;
    lower_limit_pose = LOWER_LIMIT_RAD;  // UPPER_LIMIT_RAD is defined in the header file
    send_joints_as_goal(lower_limit_pose);
}

void MyInterestingMoves::send_joints_as_goal(Vectorq7x1 q_end_pose)
{
    cout << "instantiating a baxter traj streamer" << endl;
    Baxter_traj_streamer baxter_traj_streamer(&nh_);  // instantiate a Baxter_traj_streamer object
    // warm up the joint-state callbacks
    cout << "warming up callbacks..." << endl;
    for (int i = 0; i < 100; i++)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    // some variables for the goal preparation
    Vectorq7x1 q_vec_right_arm;  // container for current right arm joints
    Eigen::VectorXd q_in_vecxd;  // vector to be push in des_path
    std::vector<Eigen::VectorXd> des_path;  // destination path
    trajectory_msgs::JointTrajectory des_trajectory;  // trajectory message
    baxter_traj_streamer::trajGoal goal;

    // get current right-arm pose and destination pose, push to des_path
    cout << "getting current right-arm pose:" << endl;
    q_vec_right_arm =  baxter_traj_streamer.get_qvec_right_arm();
    cout << "r_arm state:" << q_vec_right_arm.transpose() << endl;
    q_in_vecxd = q_vec_right_arm;  // q_in_vecxd will be reused
    des_path.push_back(q_in_vecxd);  // push for the first time
    cout << "getting ending right-arm pose:" << endl;
    cout << "ending pose:" << q_end_pose.transpose() << endl;
    q_in_vecxd = q_end_pose;  // the end pose
    des_path.push_back(q_in_vecxd);  // push for the second time, to define a trajectory

    // prepare the goal
    cout << "stuffing traj: " << endl;
    baxter_traj_streamer.stuff_trajectory(des_path, des_trajectory);  // from vector to trajectory message
    goal.trajectory = des_trajectory;
    g_count = g_count + 1;  // increment by 1 for every invoke
    goal.traj_id = g_count;  // record number of goals been sent
    ROS_INFO("sending traj_id %d", g_count);

    // send out the goal, wait and check result
    action_client.sendGoal(goal);  // send out the goal
    bool finished_before_timeout = action_client.waitForResult(ros::Duration(20.0));
    if (!finished_before_timeout)
    {
        ROS_WARN("giving up waiting on result for goal number %d", g_count);
        return;
    }
    else
    {
        ROS_INFO("finished before timeout");
    }
}

