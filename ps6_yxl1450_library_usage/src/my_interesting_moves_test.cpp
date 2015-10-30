// Copyright [2015] <Copyright yangliu28>

// test for the library ps6_yxl1450_library

#include <ros/ros.h>
#include <ps6_yxl1450_library/my_interesting_moves.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_interesting_moves_test");
    ros::NodeHandle nh;

    // initialize an object for class my_interesting_moves
    MyInterestingMoves test_move(&nh);

    // test move 1
    ROS_INFO_STREAM(endl << endl << endl << "move 1 testing...");
    test_move.all_joint_1_origin();

    // test move 2
    ROS_INFO_STREAM(endl << endl << endl << "move 2 testing...");
    test_move.all_joint_2_upper_limit();

    // test move 3
    ROS_INFO_STREAM(endl << endl << endl << "move 3 testing...");
    test_move.all_joint_3_lower_limit();
}

