//============================================================================
// Name        : EposController.hpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 26.04.2018
// Copyright   : BSD 3-Clause
// Description : Class providing the control and ROS interface for Maxon EPOS2.
//		 		 Install EPOS2 Linux Library from Maxon first!
//============================================================================

#pragma once

// STD
#include <string>

#include "maxon_epos2/EposCommunication.hpp"

// ROS
#include <ros/ros.h>

#include "maxon_epos2/epos_motor_info.h"
#include "maxon_epos2/epos_motor_service.h"
#include <std_srvs/Trigger.h>

namespace maxon_epos2 {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class EposController
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  EposController(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~EposController();

  void publisher_loop();
  void close_device();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();
  bool homingCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  bool serviceCallback(maxon_epos2::epos_motor_service::Request& request, maxon_epos2::epos_motor_service::Response& response);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic publisher.
  ros::Publisher publisher_;

  //! ROS service server
  ros::ServiceServer service_;
  ros::ServiceServer homing_service_;

  //! ROS publisher topic name
  std::string publisherTopic_;

  //! ROS service server name
  std::string serviceName_;

  //! Device object
  EposCommunication epos_device_;

  //! Create variable for publishing motor info
  maxon_epos2::epos_motor_info motor;
};

} /* namespace */
