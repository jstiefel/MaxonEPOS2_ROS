//============================================================================
// Name        : EposCommunication.cpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 26.04.2018
// Copyright   : BSD 3-Clause
// Description : Class providing the communication functions for Maxon EPOS2.
//		 		 Install EPOS2 Linux Library from Maxon first!
//============================================================================

#include "maxon_epos2/EposCommunication.hpp"

namespace maxon_epos2 {

EposCommunication::EposCommunication()
{
	g_pKeyHandle = 0; //set adress to zero
	g_usNodeId = 1;
	g_baudrate = 0;
}

EposCommunication::~EposCommunication()
{
}

void EposCommunication::LogError(std::string functionName, int p_lResult, unsigned int p_ulErrorCode)
{
	std::cerr << g_programName << ": " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< std::endl;
}

void EposCommunication::LogInfo(std::string message)
{
	std::cout << message << std::endl;
}

void EposCommunication::SeparatorLine()
{
	const int lineLength = 65;
	for(int i=0; i<lineLength; i++)
	{
		std::cout << "-";
	}
	std::cout << std::endl;
}

void EposCommunication::PrintHeader()
{
	SeparatorLine();

	LogInfo("Initializing EPOS2 Communication Library");

	SeparatorLine();
}

void EposCommunication::PrintSettings()
{
	std::stringstream msg;

	msg << "default settings:" << std::endl;
	msg << "node id             = " << g_usNodeId << std::endl;
	msg << "device name         = '" << g_deviceName << "'" << std::endl;
	msg << "protocal stack name = '" << g_protocolStackName << "'" << std::endl;
	msg << "interface name      = '" << g_interfaceName << "'" << std::endl;
	msg << "port name           = '" << g_portName << "'"<< std::endl;
	msg << "baudrate            = " << g_baudrate;

	LogInfo(msg.str());

	SeparatorLine();
}

void EposCommunication::SetDefaultParameters()
{

	/* Options:
	 * node id: default 1 (not ROS node!)
	 * device name: EPOS2, EPOS4, default: EPOS4
	 * protocol stack name: MAXON_RS232, CANopen, MAXON SERIAL V2, default: MAXON SERIAL V2
	 * interface name: RS232, USB, CAN_ixx_usb 0, CAN_kvaser_usb 0,... default: USB
	 * port name: COM1, USB0, CAN0,... default: USB0
	 * baudrate: 115200, 1000000,... default: 1000000
	 */

	//USB
	g_usNodeId = 1;
	g_deviceName = "EPOS2"; 
	g_protocolStackName = "MAXON SERIAL V2"; 
	g_interfaceName = "USB"; 
	g_baudrate = 1000000; 

	//get the port name:
	int lStartOfSelection = 1;
	int lMaxStrSize = 255;
	char* pPortNameSel = new char[lMaxStrSize];
	int lEndOfSelection = 0;
	unsigned int ulErrorCode = 0;
	VCS_GetPortNameSelection((char*)g_deviceName.c_str(), (char*)g_protocolStackName.c_str(), (char*)g_interfaceName.c_str(), lStartOfSelection, pPortNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode);
	g_portName = pPortNameSel;
	ROS_INFO_STREAM("Port Name: " << g_portName);

}

int EposCommunication::SetPositionProfile(unsigned int* p_pErrorCode)
{
	//to use set variables below first!
	int lResult = MMC_SUCCESS;
	unsigned int profile_velocity = 700; //default: 300
	unsigned int profile_acceleration = 100; //default: 50
	unsigned int profile_deceleration = 100; //default: 50

	if(VCS_SetPositionProfile(g_pKeyHandle, g_usNodeId, profile_velocity, profile_acceleration, profile_deceleration, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_SetPositionProfile", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	return lResult;
}

int EposCommunication::SetHomingParameter(unsigned int* p_pErrorCode)
{
	//to use set variables below first!
	int lResult = MMC_SUCCESS;
	unsigned int homing_acceleration = 20;
	unsigned int speed_switch = 50;
	unsigned int speed_index = 50;
	int home_offset = 0;
	unsigned short current_threshold = 0;
	int home_position = 0;
	if(VCS_SetHomingParameter(g_pKeyHandle, g_usNodeId, homing_acceleration, speed_switch, speed_index, home_offset, current_threshold, home_position, p_pErrorCode) == MMC_FAILED)
	{
		lResult = MMC_FAILED;
		LogError("VCS_SetHomingParameter", lResult, *p_pErrorCode);
	}

	return lResult;
}

int EposCommunication::SetSensor(unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;

	if(VCS_SetSensorType(g_pKeyHandle, g_usNodeId, 1, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_SetSensorType", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	if(VCS_SetIncEncoderParameter(g_pKeyHandle, g_usNodeId, 256, 0, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_SetIncEncoderParameter", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	return lResult;
}

int EposCommunication::OpenDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	char* pDeviceName = new char[255];
	char* pProtocolStackName = new char[255];
	char* pInterfaceName = new char[255];
	char* pPortName = new char[255];

	strcpy(pDeviceName, g_deviceName.c_str());
	strcpy(pProtocolStackName, g_protocolStackName.c_str());
	strcpy(pInterfaceName, g_interfaceName.c_str());
	strcpy(pPortName, g_portName.c_str());

	LogInfo("Open device...");

	g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

	if(g_pKeyHandle!=0 && *p_pErrorCode == 0)
	{
		unsigned int lBaudrate = 0;
		unsigned int lTimeout = 0;

		if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=MMC_FAILED)
		{
			if(VCS_SetProtocolStackSettings(g_pKeyHandle, g_baudrate, lTimeout, p_pErrorCode)!=MMC_FAILED)
			{
				if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=MMC_FAILED)
				{
					if(g_baudrate==(int)lBaudrate)
					{
						lResult = MMC_SUCCESS;
					}
				}
			}
		}
	}
	else
	{
		g_pKeyHandle = 0;
		ROS_ERROR("Opening device failed.");
	}

	delete []pDeviceName;
	delete []pProtocolStackName;
	delete []pInterfaceName;
	delete []pPortName;

	return lResult;
}

int EposCommunication::CloseDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	*p_pErrorCode = 0;

	if(VCS_SetDisableState(g_pKeyHandle, g_usNodeId, p_pErrorCode) == MMC_FAILED)
	{
		lResult = MMC_FAILED;
	}

	LogInfo("Close device");

	if(VCS_CloseDevice(g_pKeyHandle, p_pErrorCode)!=MMC_FAILED && *p_pErrorCode == 0)
	{
		lResult = MMC_SUCCESS;
	}

	return lResult;
}

int EposCommunication::PrepareEpos(unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	BOOL oIsFault = 0; //0 is not in fault state

	if(VCS_GetFaultState(g_pKeyHandle, g_usNodeId, &oIsFault, p_pErrorCode ) == MMC_FAILED)
	{
		LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	ROS_INFO_STREAM("Debug 1: FaultState:" << oIsFault);

	if(oIsFault)
	{
		std::stringstream msg;
		msg << "clear fault, node = '" << g_usNodeId << "'";
		LogInfo(msg.str());

		if(VCS_ClearFault(g_pKeyHandle, g_usNodeId, p_pErrorCode) == MMC_FAILED)
		{
			LogError("VCS_ClearFault", lResult, *p_pErrorCode);
			lResult = MMC_FAILED;
		}
	}

	BOOL oIsEnabled = 0;

	if(VCS_GetEnableState(g_pKeyHandle, g_usNodeId, &oIsEnabled, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}


	if(!oIsEnabled)
	{
		if(VCS_SetEnableState(g_pKeyHandle, g_usNodeId, p_pErrorCode) == MMC_FAILED)
		{
			LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
			lResult = MMC_FAILED;
		}
		else{
			VCS_GetEnableState(g_pKeyHandle, g_usNodeId, &oIsEnabled, p_pErrorCode);
			ROS_INFO_STREAM("SetEnableState should be 1:" <<  oIsEnabled);
		}
	}

	return lResult;
}

int EposCommunication::PositionMode(unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;

	lResult = ActivateProfilePositionMode(g_pKeyHandle, g_usNodeId, p_pErrorCode);

	if(lResult != MMC_SUCCESS)
	{
		LogError("ActivateProfilePositionMode", lResult, *p_pErrorCode);
	}

	return lResult;
}

int EposCommunication::HomingMode(unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;

	lResult = ActivateHomingMode(g_pKeyHandle, g_usNodeId, p_pErrorCode);

	if(lResult != MMC_SUCCESS)
	{
		LogError("ActivateHomingMode", lResult, *p_pErrorCode);
	}

	return lResult;
}

int EposCommunication::ActivateProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	std::stringstream msg;

	msg << "set profile position mode, node = " << p_usNodeId;
	LogInfo(msg.str());

	if(VCS_ActivateProfilePositionMode(p_DeviceHandle, p_usNodeId, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_ActivateProfilePositionMode", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	else {
		ROS_INFO("VCS_ActivateProfilePositionMode successfull.");
	}
	return lResult;
}

int EposCommunication::ActivateHomingMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	std::stringstream msg;

	msg << "set homing mode, node = " << p_usNodeId;
	LogInfo(msg.str());

	if(VCS_ActivateHomingMode(p_DeviceHandle, p_usNodeId, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_ActivateHomingMode", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	return lResult;
}

int EposCommunication::FindHome(unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	signed char homing_method = 1; //method 1: negative limit switch

	if(VCS_FindHome(g_pKeyHandle, g_usNodeId, homing_method, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_ActivateHomingMode", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	return lResult;
}

int EposCommunication::HomingSuccess(bool* homing_success, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	*homing_success = MMC_FAILED;
	unsigned int timeout = 6000000; //timeout in ms, should be shorter after testing

	if(VCS_WaitForHomingAttained(g_pKeyHandle, g_usNodeId, timeout, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_WaitForHomingAttained", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	int pHomingAttained;
	int pHomingError;

	if(VCS_GetHomingState(g_pKeyHandle, g_usNodeId, &pHomingAttained, &pHomingError, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_GetHomingState", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	if(pHomingAttained == MMC_SUCCESS & pHomingError == 0)
	{
		*homing_success = MMC_SUCCESS;
	}

	return lResult;
}

int EposCommunication::SetPosition(long position_setpoint, unsigned int* p_pErrorCode)
{
	// absolute position, starts immediately
	int lResult = MMC_SUCCESS;
	std::stringstream msg;

	msg << "move to position = " << position_setpoint << ", node = " << g_usNodeId;
	LogInfo(msg.str());

	if(VCS_MoveToPosition(g_pKeyHandle, g_usNodeId, position_setpoint, 1, 1, p_pErrorCode) == MMC_FAILED)
		{
			LogError("VCS_MoveToPosition", lResult, *p_pErrorCode);
			lResult = MMC_FAILED;
		}
	else{
		ROS_INFO("Movement executed.");
	}

	return lResult;
}

int EposCommunication::PrintAvailablePorts(char* p_pInterfaceNameSel)
{
	int lResult = MMC_FAILED;
	int lStartOfSelection = 1;
	int lMaxStrSize = 255;
	char* pPortNameSel = new char[lMaxStrSize];
	int lEndOfSelection = 0;
	unsigned int ulErrorCode = 0;

	do
	{
		if(!VCS_GetPortNameSelection((char*)g_deviceName.c_str(), (char*)g_protocolStackName.c_str(), p_pInterfaceNameSel, lStartOfSelection, pPortNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode))
		{
			lResult = MMC_FAILED;
			LogError("GetPortNameSelection", lResult, ulErrorCode);
			break;
		}
		else
		{
			lResult = MMC_SUCCESS;
			printf("            port = %s\n", pPortNameSel);
		}

		lStartOfSelection = 0;
	}
	while(lEndOfSelection == 0);

	return lResult;
}

int EposCommunication::PrintAvailableInterfaces()
{
	int lResult = MMC_FAILED;
	int lStartOfSelection = 1;
	int lMaxStrSize = 255;
	char* pInterfaceNameSel = new char[lMaxStrSize];
	int lEndOfSelection = 0;
	unsigned int ulErrorCode = 0;

	do
	{
		if(!VCS_GetInterfaceNameSelection((char*)g_deviceName.c_str(), (char*)g_protocolStackName.c_str(), lStartOfSelection, pInterfaceNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode))
		{
			lResult = MMC_FAILED;
			LogError("GetInterfaceNameSelection", lResult, ulErrorCode);
			break;
		}
		else
		{
			lResult = MMC_SUCCESS;

			printf("interface = %s\n", pInterfaceNameSel);

			PrintAvailablePorts(pInterfaceNameSel);
		}

		lStartOfSelection = 0;
	}
	while(lEndOfSelection == 0);

	SeparatorLine();

	delete[] pInterfaceNameSel;

	return lResult;
}

int EposCommunication::PrintDeviceVersion()
{
	int lResult = MMC_FAILED;
	unsigned short usHardwareVersion = 0;
	unsigned short usSoftwareVersion = 0;
	unsigned short usApplicationNumber = 0;
	unsigned short usApplicationVersion = 0;
	unsigned int ulErrorCode = 0;

	if(VCS_GetVersion(g_pKeyHandle, g_usNodeId, &usHardwareVersion, &usSoftwareVersion, &usApplicationNumber, &usApplicationVersion, &ulErrorCode))
	{
		printf("%s Hardware Version    = 0x%04x\n      Software Version    = 0x%04x\n      Application Number  = 0x%04x\n      Application Version = 0x%04x\n",
				g_deviceName.c_str(), usHardwareVersion, usSoftwareVersion, usApplicationNumber, usApplicationVersion);
		lResult = MMC_SUCCESS;
	}

	return lResult;
}

int EposCommunication::PrintAvailableProtocols()
{
	int lResult = MMC_FAILED;
	int lStartOfSelection = 1;
	int lMaxStrSize = 255;
	char* pProtocolNameSel = new char[lMaxStrSize];
	int lEndOfSelection = 0;
	unsigned int ulErrorCode = 0;

	do
	{
		if(!VCS_GetProtocolStackNameSelection((char*)g_deviceName.c_str(), lStartOfSelection, pProtocolNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode))
		{
			lResult = MMC_FAILED;
			LogError("GetProtocolStackNameSelection", lResult, ulErrorCode);
			break;
		}
		else
		{
			lResult = MMC_SUCCESS;

			printf("protocol stack name = %s\n", pProtocolNameSel);
		}

		lStartOfSelection = 0;
	}
	while(lEndOfSelection == 0);

	SeparatorLine();

	delete[] pProtocolNameSel;

	return lResult;
}

int EposCommunication::GetPosition(int* pPositionIsCounts, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;

	if(VCS_GetPositionIs(g_pKeyHandle, g_usNodeId, pPositionIsCounts, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_GetPositionIs", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	return lResult;
}

int EposCommunication::GetVelocity(int* pVelocityIsCounts, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;

	if(VCS_GetVelocityIs(g_pKeyHandle, g_usNodeId, pVelocityIsCounts, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_GetVelocityIs", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	return lResult;
}

//public functions:

int EposCommunication::initialization(){
	int lResult = MMC_SUCCESS;
	unsigned int ulErrorCode = 0;

	//Print Header:
	PrintHeader();

	//Set Default Parameters:
	SetDefaultParameters();

	//Print Settings:
	PrintSettings();

	//Open device:
	if((lResult = OpenDevice(&ulErrorCode))==MMC_FAILED)
	{
		LogError("OpenDevice", lResult, ulErrorCode);
	}
	else {
		deviceOpenedCheckStatus = MMC_SUCCESS; //used to forbid other functions as getPosition and getVelocity if device is not opened
	}

	//Set Sensor parameters:
	if((lResult = SetSensor(&ulErrorCode))==MMC_FAILED)
	{
		LogError("SetSensor", lResult, ulErrorCode);
	}

	//Set Position profile:
	if((lResult = SetPositionProfile(&ulErrorCode))==MMC_FAILED)
	{
		LogError("SetPositionProfile", lResult, ulErrorCode);
	}

	//Set Homing Parameter:
	if((lResult = SetHomingParameter(&ulErrorCode))==MMC_FAILED)
	{
		LogError("SetHomingParameter", lResult, ulErrorCode);
	}

	//Prepare EPOS controller:
	if((lResult = PrepareEpos(&ulErrorCode))==MMC_FAILED)
	{
		LogError("PrepareEpos", lResult, ulErrorCode);
	}

	LogInfo("Initialization successful");

	return lResult;
}

int EposCommunication::homing()
{
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;

	//Start homing mode:
	if((lResult = HomingMode(&ulErrorCode))==MMC_FAILED)
	{
		LogError("HomingMode", lResult, ulErrorCode);
	}

	//Find home:
	if((lResult = FindHome(&ulErrorCode))==MMC_FAILED)
	{
		LogError("FindHome", lResult, ulErrorCode);
	}

	//Check if successfull:
	bool homing_success_status = MMC_FAILED;
	if((lResult = HomingSuccess(&homing_success_status, &ulErrorCode))==MMC_FAILED)
	{
		LogError("HomingSuccess", lResult, ulErrorCode);
	}
	else{
		homingCompletedStatus = MMC_SUCCESS;
	}

	return homing_success_status;
}

int EposCommunication::startPositionMode()
{
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;

	//Start position mode:
	if((lResult = PositionMode(&ulErrorCode))==MMC_FAILED)
	{
		LogError("PositionMode", lResult, ulErrorCode);
	}
	else{
		ROS_INFO("PositionMode successfully started.");
	}

	return lResult;
}

bool EposCommunication::deviceOpenedCheck()
{
	return deviceOpenedCheckStatus;
}

int EposCommunication::setPosition(float position_setpoint){
	//Set position, call this function in service callback:
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;

	//Safety check setpoint and homing:
	if(homingCompletedStatus == MMC_SUCCESS & position_setpoint <= 0 & position_setpoint >= -45)
	{
		if((lResult = SetPosition(mmToCounts(position_setpoint), &ulErrorCode))==MMC_FAILED)
		{
			LogError("SetPosition", lResult, ulErrorCode);
			return lResult;
		}
		else{
			ROS_INFO("SetPosition executed.");
		}
	}
	return lResult;
}

int EposCommunication::getPosition(float* pPositionIs)
{
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;
	int pPositionIsCounts = 0;

	if((lResult = GetPosition(&pPositionIsCounts, &ulErrorCode))==MMC_FAILED)
	{
		LogError("getPosition", lResult, ulErrorCode);
		return lResult;
	}
	*pPositionIs = countsTomm(&pPositionIsCounts);

	//only for Debugging
	//ROS_INFO_STREAM("!!! pPositionIs: " << *pPositionIs << " pPositionIsCounts: " << pPositionIsCounts);
	return lResult;
}

int EposCommunication::getVelocity(float* pVelocityIs)
{
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;
	int pVelocityIsCounts;

	if((lResult = GetVelocity(&pVelocityIsCounts, &ulErrorCode))==MMC_FAILED)
	{
		LogError("getVelocity", lResult, ulErrorCode);
		return lResult;
	}
	*pVelocityIs = countsTomm(&pVelocityIsCounts);
	return lResult;
}

int EposCommunication::closeDevice(){
	//Close device:
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;
	if((lResult = CloseDevice(&ulErrorCode))==MMC_FAILED)
	{
		LogError("CloseDevice", lResult, ulErrorCode);
		return lResult;
	}
	return lResult;
}

float EposCommunication::countsTomm(int* counts){
	//1024 encoder counts per revolution
	//linear pitch R = 2.35mm
	float i_trans = 32/40.;
	float gpx_trans = 10/66.;
	float R = 2.35;

	float mm = 1/1024. * i_trans * gpx_trans * R * (*counts);

	return mm;
}

int EposCommunication::mmToCounts(float mm){
	float i_trans = 32/40.;
	float gpx_trans = 10/66.;
	float R = 2.35;

	int counts = 1024 * (1/i_trans) * (1/R) * (1/gpx_trans) * mm;
	ROS_INFO_STREAM("counts: " << counts);
	return counts;
}

	/* workflow:
	 * initialize, setPosition, closeDevice
	 */

} /* namespace */
