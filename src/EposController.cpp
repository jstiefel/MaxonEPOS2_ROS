//============================================================================
// Name        : EposController.cpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 26.04.2018
// Copyright   : BSD 3-Clause
// Description : Class providing the control and ROS interface for Maxon EPOS2
//				 (subscribers, parameters, timers, etc.).
//		 		 Install EPOS2 Linux Library from Maxon first!
//============================================================================

#include "maxon_epos2/EposController.hpp"

namespace maxon_epos2 {

EposController::EposController(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }

  //Initialize device:
  if((epos_device_.initialization())==MMC_FAILED) ROS_ERROR("Device initialization");
  //Start position mode during homing callback function:
  //if((epos_device_.startPositionMode())==MMC_FAILED) ROS_ERROR("Starting position mode failed");


  publisher_ = nodeHandle_.advertise<maxon_epos2::epos_motor_info>(publisherTopic_, 10);
  homing_service_ = nodeHandle_.advertiseService("epos_homing_service", &EposController::homingCallback, this);
  service_ = nodeHandle_.advertiseService(serviceName_, &EposController::serviceCallback, this);


  ROS_INFO("Successfully launched EPOS Controller node.");
}

EposController::~EposController()
{
}

bool EposController::readParameters()
{
  if (!nodeHandle_.getParam("publisher_topic", publisherTopic_)) return false;
  if (!nodeHandle_.getParam("service_name", serviceName_)) return false;
  return true;
}

bool EposController::homingCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response){
	ROS_INFO("Requested homing service");
	//Home device:
	ROS_INFO("Homing...");
	if((epos_device_.homing())==MMC_FAILED) ROS_ERROR("Device homing failed");
	else{
		//Start position mode:
		ROS_INFO("Start position mode");
		if((epos_device_.startPositionMode())==MMC_FAILED) ROS_ERROR("Starting position mode failed");
		response.success = MMC_SUCCESS;
	}
	return true;
}

bool EposController::serviceCallback(maxon_epos2::epos_motor_service::Request& request, maxon_epos2::epos_motor_service::Response& response){
	ROS_INFO_STREAM("Requested position" << request.position_setpoint);
	if(!epos_device_.setPosition(request.position_setpoint)) ROS_ERROR("setPosition failed");
	response.success = true;
	if((epos_device_.getPosition(&response.position)) == MMC_FAILED) ROS_ERROR("getPosition failed for service");
	if((epos_device_.getVelocity(&response.velocity)) == MMC_FAILED) ROS_ERROR("getVelocity failed for service");
	return true;
}

void EposController::publisher_loop(){
	if((epos_device_.deviceOpenedCheck()) == MMC_SUCCESS)
	{

		epos_device_.getPosition(&motor.position);
		epos_device_.getVelocity(&motor.velocity);
// ****only output these for DEBUGGING******
//		if((epos_device_.getPosition(&motor.position)) == MMC_FAILED) ROS_ERROR("getPosition failed for message");
//		if((epos_device_.getVelocity(&motor.velocity)) == MMC_FAILED) ROS_ERROR("getVelocity failed for message");
		publisher_.publish(motor);
	}
	else{
		//****only for DEBUGGING****
//		ROS_INFO("Nothing to publish.");
	}
}

void EposController::close_device(){
	  if((epos_device_.closeDevice()) == MMC_FAILED) ROS_ERROR("Device closing failed");
}

} /* namespace */
