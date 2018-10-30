//============================================================================
// Name        : EposCommunication.hpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 26.04.2018
// Copyright   : BSD 3-Clause
// Description : Class providing the communication functions for Maxon EPOS2.
//		 		 Install EPOS2 Linux Library from Maxon first!
//============================================================================

#pragma once

//Define some parameters:
#ifndef MMC_SUCCESS
	#define MMC_SUCCESS 1
#endif

#ifndef MMC_FAILED
	#define MMC_FAILED 0
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
	#define MMC_MAX_LOG_MSG_SIZE 512
#endif
//--

#ifndef EPOSCOMMUNICATION_H_
#define EPOSCOMMUNICATION_H_
// Include headers of Maxon EPOS library libCmdEpos.so:
#include "maxon_epos2/Definitions.h"
#include "maxon_epos2/WinTypes.h"

#include <iostream>
//STD
#include <string>
#include <sstream>
#include <getopt.h>
#include "stdint.h"
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <ros/ros.h>
#include <cmath>
#include <unistd.h>
#include <list>
#include <sys/types.h>
#include <sys/times.h>
#include <sys/time.h>

namespace maxon_epos2 {

/*!
 * Class containing the algorithmic part of the package.
 */
class EposCommunication
{
 public:
  /*!
   * Constructor.
   */
  EposCommunication();

  /*!
   * Destructor.
   */
  virtual ~EposCommunication();

  int 	initialization();
  bool 	deviceOpenedCheck();
  int 	homing();
  int 	startPositionMode();
  int 	setPosition(float position_setpoint);
  int 	getPosition(float* pPositionIs);
  int 	getVelocity(float* pVelocityIs);
  int 	closeDevice();

 private:
  //define HANDLE
  typedef void* HANDLE;
  typedef int BOOL;

  //Variables:
  bool deviceOpenedCheckStatus = MMC_FAILED;
  bool homingCompletedStatus = MMC_FAILED;
  void* g_pKeyHandle;
  unsigned short g_usNodeId;
  int g_baudrate;
  std::string g_deviceName;
  std::string g_protocolStackName;
  std::string g_interfaceName;
  std::string g_portName;
  const std::string g_programName = "EPOS2 Controller";
  
  //Functions:
  void  LogError(std::string functionName, int p_lResult, unsigned int p_ulErrorCode);
  void  LogInfo(std::string message);
  void  SeparatorLine();
  void  PrintHeader();
  void  PrintSettings();
  void  SetDefaultParameters();
  int	SetPositionProfile(unsigned int* p_pErrorCode);
  int 	SetHomingParameter(unsigned int* p_pErrorCode);
  int	SetSensor(unsigned int* p_pErrorCode);
  int   OpenDevice(unsigned int* p_pErrorCode);
  int   CloseDevice(unsigned int* p_pErrorCode);
  int   PrepareEpos(unsigned int* p_pErrorCode);
  int	PositionMode(unsigned int* p_pErrorCode);
  int 	HomingMode(unsigned int* p_pErrorCode);
  int	ActivateProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode);
  int 	ActivateHomingMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode);
  int 	FindHome(unsigned int* p_pErrorCode);
  int 	HomingSuccess(bool* homing_success, unsigned int* p_pErrorCode);
  int	SetPosition(long position_setpoint, unsigned int* p_pErrorCode);
  int	PrintAvailablePorts(char* p_pInterfaceNameSel);
  int   PrintAvailableInterfaces();
  int   PrintDeviceVersion();
  int	PrintAvailableProtocols();
  int	GetPosition(int* pPositionIsCounts, unsigned int* p_pErrorCode);
  int	GetVelocity(int* pVelocityIsCounts, unsigned int* p_pErrorCode);
  float countsTomm(int* counts);
  int 	mmToCounts(float mm);


}; /* Class */


#endif /* EPOSCOMMUNICATION_H_ */

} /* namespace */
