//============================================================================
// Name        : maxon_epos2_node.cpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 26.04.2018
// Copyright   : BSD 3-Clause
// Description : Node for maxon_epos2, initialization of ROS.
//		 		 Install EPOS2 Linux Library from Maxon first!
//============================================================================

#include <ros/ros.h>
#include "maxon_epos2/EposController.hpp"

int main(int argc, char** argv)
{
	/*
	 * We have a publisher, so we can not use ros::spin(), but spinOnce and sleep in a while loop instead.
	 * spinOnce is needed for subscriber callback.
	 */

	ros::init(argc, argv, "maxon_epos2");
	ros::NodeHandle nodeHandle("~");
	ros::Rate loop_rate(10);

	//create class Object "EposController", this also initializes EPOS2
	maxon_epos2::EposController EposController(nodeHandle);

	//publish until node gets interrupted
	while (ros::ok()){
		EposController.publisher_loop();
		ros::spinOnce();
		loop_rate.sleep();
	}

	//if node is interrupted, close device
	EposController.close_device();

	return 0;
}
