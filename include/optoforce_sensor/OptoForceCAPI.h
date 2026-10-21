#pragma once
#include "OptoForceAPI.h"
#include "OptoSimplePacket3D.h"
#include "OptoSimplePacket6D.h"
/*! \file OptoForceCAPI.h
\brief C binding for OptoForceAPI
\n

Example code:
\code
OptoDAQDescriptorHandle descriptor = OptoDAQDescriptorCreate("");
OptoDAQWatcherHandle daqWatcher = OptoDAQWatcherCreate();
OptoDAQWatcherStart(daqWatcher);

while (OptoDAQDescriptorIsValid(descriptor) == 0) { //Waiting for a DAQ to be connected to the computer
  OptoDAQWatcherGetFirstDAQ(daqWatcher, descriptor);  
}
printf("Address of connected DAQ: %s\r\n", OptoDAQDescriptorGetAddress(descriptor));
printf("Type name of connected DAQ: %s\r\n", OptoDAQDescriptorGetTypeName(descriptor));
printf("Serial number of connected DAQ: %s\r\n", OptoDAQDescriptorGetSerialNumber(descriptor));
OptoDAQHandle optoDAQ = OptoDAQCreate(descriptor, 1000);
OptoDAQDescriptorRelease(descriptor);
if (OptoDAQOpen(optoDAQ) == 0) {
  printf("Could not open DAQ...\r\n");
  OptoDAQRelease(optoDAQ);
  OptoDAQWatcherRelease(daqWatcher);
  return;
}
OptoConfigHandle config = OptoConfigCreate();
OptoConfigSetSpeed(config, 100);  //Speed will be 100 Hz
OptoConfigSetFilter(config, 4);
OptoConfigSetZeroing(config, 0);
if (OptoDAQSendConfig(optoDAQ, config) == 0) {
  printf("Could not send config...\r\n"); // This is not a fatal error but should not happen
}
OptoPacket3DHandle packet3D = OptoPacket3DCreate();
OptoPacket6DHandle packet6D = OptoPacket6DCreate();
int is3D = OptoDAQIs3D(optoDAQ);
for (unsigned int i = 0; i < 1000; ++i) {
  if (is3D) {  // This is a 3D packet
    OptoDAQGetLastPacket3D(optoDAQ, packet3D, 1);
      if (OptoPacket3DIsValid(packet3D) > 0) {
        int fx = OptoPacket3DGetFxInCounts(packet3D, 0);
        int fy = OptoPacket3DGetFyInCounts(packet3D, 0);
        int fz = OptoPacket3DGetFzInCounts(packet3D, 0);
        printf("Fx: %d; Fy: %d; Fz: %d\r\n", fx, fy, fz);
      }
  }
  else { // This is a 6D packet
    OptoDAQGetLastPacket6D(optoDAQ, packet6D, 1);
    if (OptoPacket6DIsValid(packet3D) > 0) {
      int fx = OptoPacket6DGetFxInCounts(packet6D);
      int fy = OptoPacket6DGetFyInCounts(packet6D);
      int fz = OptoPacket6DGetFzInCounts(packet6D);
      int tx = OptoPacket6DGetTxInCounts(packet6D);
      int ty = OptoPacket6DGetTyInCounts(packet6D);
      int tz = OptoPacket6DGetTzInCounts(packet6D);
      printf("Fx: %d; Fy: %d; Fz:%d; Tx:%d; Ty:%d; Tz:%d\r\n", fx, fy, fz, tx, ty, tz);
    }
  }
}
// Release instances
OptoDAQRelease(optoDAQ);
OptoDAQWatcherRelease(daqWatcher);
OptoPacket3DRelease(packet3D);
OptoPacket6DRelease(packet6D);
OptoConfigRelease(config);
\endcode
*/
#ifdef __cplusplus
extern "C" {
#endif
	/**
	* Handle of \ref OptoDAQDescriptor
	*/
	typedef	void* OptoDAQDescriptorHandle;
	/**
	* Handle of \ref OptoDebugPacket
	*/
	typedef	void* OptoDebugPacketHandle;
	/**
	* Handle of \ref OptoPacket3D
	*/
	typedef	void* OptoPacket3DHandle;
	/**
	* Handle of \ref OptoPacket6D
	*/
	typedef	void* OptoPacket6DHandle;
	/**
	* Handle of \ref OptoPackets3D
	*/
	typedef	void* OptoPackets3DHandle;
	/**
	* Handle of \ref OptoPackets6D
	*/
	typedef	void* OptoPackets6DHandle;
	/**
	* Handle of \ref OptoDAQWatcher
	*/
	typedef void* OptoDAQWatcherHandle;
	/**
	* Handle of \ref OptoConfig
	*/
	typedef void* OptoConfigHandle;
	/**
	* Handle of \ref OptoSensitivityReport
	*/
	typedef void* OptoSensitivityReportHandle;
	/**
	* Handle of \ref OptoSensitivity
	*/
	typedef void* OptoSensitivityHandle;
	/**
	* Handle of \ref OptoDAQ
	*/
	typedef void* OptoDAQHandle;
	/**
	* Retrieves the API's version string
	* @return the API's version string  (for example: "2.0.0.144"))
	*/
	OPTOFORCE_API	const char *					getVersionString();


	/**
	* Creates an \ref OptoDAQDescriptor instance and retrieves the handle of the created instance
	* @param [in] p_address The address of DAQ (like "COM3")
	* @return The handle of the created instance
	*/
	OPTOFORCE_API  OptoDAQDescriptorHandle			OptoDAQDescriptorCreate(const char * p_address);
	/**
	* Releases an \ref OptoDAQDescriptor instance
	* @param [in] p_OptoDAQDescriptor The instance which is intended to be released
	*/
	OPTOFORCE_API  void								OptoDAQDescriptorRelease(OptoDAQDescriptorHandle p_OptoDAQDescriptor);
	/**
	* Determines if an OptoDAQDescriptor instance is a USB connected device
	* @return a nonzero value if the DAQ is USB connected, zero otherwise (currently it always returns > 0)
	*/
	OPTOFORCE_API  int								OptoDAQDescriptorIsUSB(OptoDAQDescriptorHandle p_OptoDAQDescriptor);
	/**
	* Sets the type name of an \ref OptoDAQDescriptor instance. Currently acceptable names are "31", "34" and "61"
	* @param [in] p_OptoDAQDescriptor The handle of \ref OptoDAQDescriptor instance
	* @param [in] p_typeName A null-terminated string which holds the type name
	*/
	OPTOFORCE_API void								OptoDAQDescriptorSetTypeName(OptoDAQDescriptorHandle p_OptoDAQDescriptor, const char * p_typeName);
	/**
	* Sets the type name of an \ref OptoDAQDescriptor instance. Currently acceptable names are 31, 34 and 61
	* @param [in] p_OptoDAQDescriptor The handle of \ref OptoDAQDescriptor instance
	* @param [in] p_typeName An integer  which holds the type name
	*/
	OPTOFORCE_API void								OptoDAQDescriptorSetTypeNameAsInt(OptoDAQDescriptorHandle p_OptoDAQDescriptor, int p_typeName);
	/**
	* Sets the serial number of an \ref OptoDAQDescriptor instance. 
	* @param [in] p_OptoDAQDescriptor The handle of \ref OptoDAQDescriptor instance
	* @param [in] p_serialNumber A null-terminated string which holds the serial number
	*/
	OPTOFORCE_API void								OptoDAQDescriptorSetSerialNumber(OptoDAQDescriptorHandle p_OptoDAQDescriptor, const char * p_serialNumber);
	/**
	* Sets the serial number of an \ref OptoDAQDescriptor instance.
	* @param [in] p_OptoDAQDescriptor The handle of \ref OptoDAQDescriptor instance
	* @param [in] p_address A null-terminated string which holds the addres (like "COM3")
	*/
	OPTOFORCE_API void								OptoDAQDescriptorSetAddress(OptoDAQDescriptorHandle p_OptoDAQDescriptor, const char * p_address);
	/**
	* Sets the opening parameter of a DAQ. Currently it is used to set up the
	* baudrate of a DAQ. Its default value is 1000000
	*/
	OPTOFORCE_API void								OptoDAQDescriptorSetOpeningParameter(OptoDAQDescriptorHandle p_OptoDAQDescriptor, int p_openingParameter);
	/**
	* Retrieves the address of an \ref OptoDAQDescriptor instance
	* @param [in] p_OptoDAQDescriptor The handle of \ref OptoDAQDescriptor instance
	* @return the address of the \ref OptoDAQDescriptor instance as a null-terminated string if parameter is valid (not NULL), otherwise returns NULL
	*/
	OPTOFORCE_API const char *						OptoDAQDescriptorGetAddress(OptoDAQDescriptorHandle  p_OptoDAQDescriptor);
	/**
	* Retrieves the type name of an \ref OptoDAQDescriptor instance
	* @param [in] p_OptoDAQDescriptor The handle of \ref OptoDAQDescriptor instance
	* @return the type name of the \ref OptoDAQDescriptor instance as a null-terminated string if parameter is valid (not NULL), otherwise returns NULL
	*/
	OPTOFORCE_API const char *						OptoDAQDescriptorGetTypeName(OptoDAQDescriptorHandle p_OptoDAQDescriptor);
	/**
	* Retrieves the address of an \ref OptoDAQDescriptor instance
	* @param [in] p_OptoDAQDescriptor The handle of \ref OptoDAQDescriptor instance
	* @return the address of the \ref OptoDAQDescriptor instance as a null-terminated string if parameter is valid (not NULL), otherwise returns NULL
	*/
	OPTOFORCE_API const char *						OptoDAQDescriptorGetSerialNumber(OptoDAQDescriptorHandle p_OptoDAQDescriptor);
	/**
	* Retrieves the opening parameter of a DAQ (i.e. baudrate)
	* @return the set opening parameter, -1 if the given p_OptoDAQDescriptor is NULL
	*/
	OPTOFORCE_API int								OptoDAQDescriptorGetOpeningParameter(OptoDAQDescriptorHandle p_OptoDAQDescriptor);
	/**
	* Determines if an instance of \ref OptoDAQDescriptor holds the informations of a 3-axis DAQ
	* @return a nonzero value if the instance holds the informations of a 3-axis DAQ, otherwise returns 0
	*/
	OPTOFORCE_API int								OptoDAQDescriptorIs3D(OptoDAQDescriptorHandle p_OptoDAQDescriptor);
	/**
	* Determines if an instance of \ref OptoDAQDescriptor holds the informations of a 6-axis DAQ
	* @return a nonzero value if the instance holds the informations of a 6-axis DAQ, otherwise returns 0
	*/
	OPTOFORCE_API int								OptoDAQDescriptorIs6D(OptoDAQDescriptorHandle p_OptoDAQDescriptor);
	/**
	* Determines if an instance of \ref OptoDAQDescriptor is valid
	* @return a nonzero value if the instance is valid, otherwise returns 0
	*/
	OPTOFORCE_API int								OptoDAQDescriptorIsValid(OptoDAQDescriptorHandle p_OptoDAQDescriptor);


	/**
	* Retrieves raw signal of an \ref OptoDebugPacket instance
	* @param [in] p_OptoDebugPacket The handle of \ref OptoDebugPacket
	* @param [in] p_index The index of the raw signal (the value should be 0, 1, 2 or 3)
	* @return the value of raw signal
	*/
	OPTOFORCE_API int								OptoDebugPacketGetRawSignal(OptoDebugPacketHandle p_OptoDebugPacket, unsigned int p_index);
	/**
	* Retrieves compensated signal of an \ref OptoDebugPacket instance
	* @param [in] p_OptoDebugPacket The handle of \ref OptoDebugPacket
	* @param [in] p_index The index of the compensated signal (the value should be 0, 1, 2 or 3)
	* @return the value of compensated signal
	*/
	OPTOFORCE_API int								OptoDebugPacketGetCompensatedSignal(OptoDebugPacketHandle p_OptoDebugPacket, unsigned int p_index);
	/**
	* Retrieves static raw signal of an \ref OptoDebugPacket instance. (See \ref OptoDebugPacket::GetRawSignalStatic())
	* @param [in] p_OptoDebugPacket The handle of \ref OptoDebugPacket
	* @param [in] p_index The index of the raw signal (the value should be 0, 1, 2 or 3)
	* @return the value of static raw signal
	*/
	OPTOFORCE_API int								OptoDebugPacketGetRawSignalStatic(OptoDebugPacketHandle p_OptoDebugPacket, unsigned int p_index);
	/**
	* Retrieves static compensated signal of an \ref OptoDebugPacket instance. (See \ref OptoDebugPacket::GetCompensatedSignalStatic())
	* @param [in] p_OptoDebugPacket The handle of \ref OptoDebugPacket
	* @param [in] p_index The index of the compensated signal (the value should be 0, 1, 2 or 3)
	* @return the value of static compensated signal
	*/
	OPTOFORCE_API int								OptoDebugPacketGetCompensatedSignalStatic(OptoDebugPacketHandle p_OptoDebugPacket, unsigned int p_index);
	/**
	* Retrieves the temperature signal of an \ref OptoDebugPacket instance
	* @param [in] p_OptoDebugPacket The handle of \ref OptoDebugPacket
	* @return the value of temperature signal
	*/
	OPTOFORCE_API int								OptoDebugPacketGetTemp(OptoDebugPacketHandle p_OptoDebugPacket);
	/**
	* Retrieves the value of special Fx value, which is calculated from raw signals of 6-axis DAQ 
	* @param [in] p_OptoDebugPacket The handle of \ref OptoDebugPacket
	* @return the value of Fx in dimensionless counts 
	*/
	OPTOFORCE_API int								OptoDebugPacketGetFxAsCounts(OptoDebugPacketHandle p_OptoDebugPacket);
	/**
	* Retrieves the value of special Fy value, which is calculated from raw signals of 6-axis DAQ
	* @param [in] p_OptoDebugPacket The handle of \ref OptoDebugPacket
	* @return the value of Fy in dimensionless counts
	*/
	OPTOFORCE_API int								OptoDebugPacketGetFyAsCounts(OptoDebugPacketHandle p_OptoDebugPacket);
	/**
	* Retrieves the value of special Fz value, which is calculated from raw signals of 6-axis DAQ
	* @param [in] p_OptoDebugPacket The handle of \ref OptoDebugPacket
	* @return the value of Fz in dimensionless counts
	*/
	OPTOFORCE_API int								OptoDebugPacketGetFzAsCounts(OptoDebugPacketHandle p_OptoDebugPacket);
	/**
	* Retrieves the value of special Fx value, which is calculated from compensated signals of 6-axis DAQ
	* @param [in] p_OptoDebugPacket The handle of \ref OptoDebugPacket
	* @return the value of Fx in dimensionless counts
	*/
	OPTOFORCE_API int								OptoDebugPacketGetCompensatedFxAsCounts(OptoDebugPacketHandle p_OptoDebugPacket);
	/**
	* Retrieves the value of special Fy value, which is calculated from compensated signals of 6-axis DAQ
	* @param [in] p_OptoDebugPacket The handle of \ref OptoDebugPacket
	* @return the value of Fy in dimensionless counts
	*/
	OPTOFORCE_API int								OptoDebugPacketGetCompensatedFyAsCounts(OptoDebugPacketHandle p_OptoDebugPacket);
	/**
	* Retrieves the value of special Fz value, which is calculated from compensated signals of 6-axis DAQ
	* @param [in] p_OptoDebugPacket The handle of \ref OptoDebugPacket
	* @return the value of Fz in dimensionless counts
	*/
	OPTOFORCE_API int								OptoDebugPacketGetCompensatedFzAsCounts(OptoDebugPacketHandle p_OptoDebugPacket);
	/**
	* Determines if the OptoDebugPacket is a valid packet. Invalid packets should never be used
	* @param [in] p_OptoDebugPacket The handle of \ref OptoDebugPacket instance
	* @return a nonzero value if the packet is valid, otherwise returns 0
	*/
	OPTOFORCE_API int								OptoDebugPacketIsValid(OptoDebugPacketHandle p_OptoDebugPacket);

	/**
	* Creates an \ref OptoPacket3D instance and retrieves the handle of the created instance
	* @return The handle of the created instance
	*/
	OPTOFORCE_API OptoPacket3DHandle				OptoPacket3DCreate();
	/**
	* Releases an \ref OptoPacket3D instance
	* @param [in] p_OptoPacket3D The instance which is intended to be released
	*/
	OPTOFORCE_API void								OptoPacket3DRelease(OptoPacket3DHandle p_OptoPacket3D);
	/**
	* Retrieves the sample counter of an \ref OptoPacket3D instance
	* @param [in] p_OptoPacket3D The handle of \ref OptoPacket3D
	* @return the sample counter of the instance
	*/
	OPTOFORCE_API unsigned short					OptoPacket3DGetCounter(OptoPacket3DHandle p_OptoPacket3D);
	/**
	* Retrieves the Fx force as a dimensionless value
	* @param [in] p_OptoPacket3D The handle of \ref OptoPacket3D
	* @param [in] p_index The index of the requested value (should be 0, 1, 2 or 3)
	* @return Fx force as a dimensionless value
	*/
	OPTOFORCE_API int								OptoPacket3DGetFxInCounts(OptoPacket3DHandle p_OptoPacket3D, unsigned int p_index);
	/**
	* Retrieves the Fy force as a dimensionless value
	* @param [in] p_OptoPacket3D The handle of \ref OptoPacket3D
	* @param [in] p_index The index of the requested value (should be 0, 1, 2 or 3)
	* @return Fy force as a dimensionless value
	*/
	OPTOFORCE_API int								OptoPacket3DGetFyInCounts(OptoPacket3DHandle p_OptoPacket3D, unsigned int p_index);
	/**
	* Retrieves the Fz force as a dimensionless value
	* @param [in] p_OptoPacket3D The handle of \ref OptoPacket3D
	* @param [in] p_index The index of the requested value (should be 0, 1, 2 or 3)
	* @return Fz force as a dimensionless value
	*/
	OPTOFORCE_API int								OptoPacket3DGetFzInCounts(OptoPacket3DHandle p_OptoPacket3D, unsigned int p_index);
	/**
	* Retrieves the Fx force as Newton
	* @param [in] p_OptoPacket3D The handle of \ref OptoPacket3D
	* @param [in] p_index The index of the requested value (should be 0, 1, 2 or 3)
	* @return Fx force in Newton
	*/
	OPTOFORCE_API double							OptoPacket3DGetFxInNewton(OptoPacket3DHandle p_OptoPacket3D, unsigned int p_index);
	/**
	* Retrieves the Fy force as Newton
	* @param [in] p_OptoPacket3D The handle of \ref OptoPacket3D
	* @param [in] p_index The index of the requested value (should be 0, 1, 2 or 3)
	* @return Fy force as Newton (currently it returns with 0.0)
	*/
	OPTOFORCE_API double							OptoPacket3DGetFyInNewton(OptoPacket3DHandle p_OptoPacket3D, unsigned int p_index);
	/**
	* Retrieves the Fz force as Newton
	* @param [in] p_OptoPacket3D The handle of \ref OptoPacket3D
	* @param [in] p_index The index of the requested value (should be 0, 1, 2 or 3)
	* @return Fz force as Newton (currently it returns with 0.0)
	*/
	OPTOFORCE_API double							OptoPacket3DGetFzInNewton(OptoPacket3DHandle p_OptoPacket3D, unsigned int p_index);
	/**
	* Retrieves the status word of an \ref OptoPacket3D instance
	* @param [in] p_OptoPacket3D The handle of \ref OptoPacket3D
	* @return the status word of an \ref OptoPacket3D instance
	*/
	OPTOFORCE_API unsigned short					OptoPacket3DGetStatus(OptoPacket3DHandle p_OptoPacket3D);
	/**
	* Retrieves the handle of the \ref OptoDebugPacket of an \ref OptoPacket3D instance
	* @param [in] p_OptoPacket3D The handle of \ref OptoPacket3D
	* @param [in] p_index The index of the requested debug packet (value should be 0, 1, 2 or 3)
	* @return the handle of the \ref OptoDebugPacket of the packets' debug packet
	*/
	OPTOFORCE_API const OptoDebugPacketHandle		OptoPacket3DGetDebugPacket(OptoPacket3DHandle p_OptoPacket3D, unsigned int p_index);
	/**
	* Sets the timestamp of an \ref OptoPacket3D instance
	* @param [in] p_OptoPacket3D The handle of \ref OptoPacket3D
	* @param [in] p_timeStamp The timestamp in microseconds
	*/
	OPTOFORCE_API void								OptoPacket3DSetTimeStamp(OptoPacket3DHandle p_OptoPacket3D, long long p_timeStamp);
	/**
	* Retrieves the timestamp of an \ref OptoPacket3D instance
	* @param [in] p_OptoPacket3D The handle of \ref OptoPacket3D
	* @return the instance's timestamp in microseconds
	*/
	OPTOFORCE_API long long							OptoPacket3DGetTimeStamp(OptoPacket3DHandle p_OptoPacket3D);
	/**
	* Generates a more simple structure from an \ref OptoPacket3D instance
	* @param [in] p_OptoPacket3D The handle of \ref OptoPacket3D
	* @return the generated \ref OptoSimplePacket3D structure
	*/
	OPTOFORCE_API OptoSimplePacket3D				OptoPacket3DToSimplePacket(OptoPacket3DHandle p_OptoPacket3D);
	/**
	* Retrieves the size of an \ref OptoPacket3D instance (in the case of one-channel DAQ it returns 1 otherwise it returns 4) 
	* @param [in] p_OptoPacket3D The handle of \ref OptoPacket3D
	* @return 1 in the case of one-channel DAQ, otherwise 4 
	*/
	OPTOFORCE_API unsigned int						OptoPacket3DGetSize(OptoPacket3DHandle p_OptoPacket3D);
	/**
	* Determines if an \ref OptoPacket3D instance is valid
	* @param [in] p_OptoPacket3D The handle of \ref OptoPacket3D
	* @return a nonzero value if the instance is valid, otherwise 0
	*/
	OPTOFORCE_API int								OptoPacket3DIsValid(OptoPacket3DHandle p_OptoPacket3D);


	/**
	* Creates an \ref OptoPacket6D instance and retrieves the handle of the created instance
	* @return The handle of the created instance
	*/
	OPTOFORCE_API OptoPacket6DHandle				OptoPacket6DCreate();
	/**
	* Releases an \ref OptoPacket6D instance
	* @param [in] p_OptoPacket6D The instance which is intended to be released
	*/
	OPTOFORCE_API void								OptoPacket6DRelease(OptoPacket6DHandle p_OptoPacket6D);
	/**
	* Retrieves the sample counter of an \ref OptoPacket6D instance
	* @param [in] p_OptoPacket6D The handle of \ref OptoPacket6D
	* @return the sample counter of the instance
	*/
	OPTOFORCE_API unsigned short					OptoPacket6DGetCounter(OptoPacket6DHandle p_OptoPacket6D);
	/**
	* Retrieves the Fx force as a dimensionless value
	* @param [in] p_OptoPacket6D The handle of \ref OptoPacket6D
	* @return Fx force as a dimensionless value
	*/
	OPTOFORCE_API int								OptoPacket6DGetFxInCounts(OptoPacket6DHandle p_OptoPacket6D);
	/**
	* Retrieves the Fy force as a dimensionless value
	* @param [in] p_OptoPacket6D The handle of \ref OptoPacket6D
	* @return Fy force as a dimensionless value
	*/
	OPTOFORCE_API int								OptoPacket6DGetFyInCounts(OptoPacket6DHandle p_OptoPacket6D);
	/**
	* Retrieves the Fz force as a dimensionless value
	* @param [in] p_OptoPacket6D The handle of \ref OptoPacket6D
	* @return Fz force as a dimensionless value
	*/
	OPTOFORCE_API int								OptoPacket6DGetFzInCounts(OptoPacket6DHandle p_OptoPacket6D);
	/**
	* Retrieves the Tx torque as a dimensionless value
	* @param [in] p_OptoPacket6D The handle of \ref OptoPacket6D
	* @return Tx torque as a dimensionless value
	*/
	OPTOFORCE_API int								OptoPacket6DGetTxInCounts(OptoPacket6DHandle p_OptoPacket6D);
	/**
	* Retrieves the Ty torque as a dimensionless value
	* @param [in] p_OptoPacket6D The handle of \ref OptoPacket6D
	* @return Ty torque as a dimensionless value
	*/
	OPTOFORCE_API int								OptoPacket6DGetTyInCounts(OptoPacket6DHandle p_OptoPacket6D);
	/**
	* Retrieves the Tz torque as a dimensionless value
	* @param [in] p_OptoPacket6D The handle of \ref OptoPacket6D
	* @return Tz torque as a dimensionless value
	*/
	OPTOFORCE_API int								OptoPacket6DGetTzInCounts(OptoPacket6DHandle p_OptoPacket6D);
	/**
	* Retrieves the Fx force as Newton
	* @param [in] p_OptoPacket6D The handle of \ref OptoPacket6D
	* @return Fx force as Newton
	*/
	OPTOFORCE_API double							OptoPacket6DGetFxInNewton(OptoPacket6DHandle p_OptoPacket6D);
	/**
	* Retrieves the Fy force as Newton
	* @param [in] p_OptoPacket6D The handle of \ref OptoPacket6D
	* @return Fy force as Newton
	*/
	OPTOFORCE_API double							OptoPacket6DGetFyInNewton(OptoPacket6DHandle p_OptoPacket6D);
	/**
	* Retrieves the Fz force as Newton
	* @param [in] p_OptoPacket6D The handle of \ref OptoPacket6D
	* @return Fz force as Newton
	*/
	OPTOFORCE_API double							OptoPacket6DGetFzInNewton(OptoPacket6DHandle p_OptoPacket6D);
	/**
	* Retrieves the Tx torque as Newton meter
	* @param [in] p_OptoPacket6D The handle of \ref OptoPacket6D
	* @return Tx torque as Newton meter
	*/
	OPTOFORCE_API double							OptoPacket6DGetTxInNewtonMeter(OptoPacket6DHandle p_OptoPacket6D);
	/**
	* Retrieves the Ty torque as Newton meter
	* @param [in] p_OptoPacket6D The handle of \ref OptoPacket6D
	* @return Ty torque as Newton meter
	*/
	OPTOFORCE_API double							OptoPacket6DGetTyInNewtonMeter(OptoPacket6DHandle p_OptoPacket6D);
	/**
	* Retrieves the Tz torque as Newton meter
	* @param [in] p_OptoPacket6D The handle of \ref OptoPacket6D
	* @return Tz torque as Newton meter
	*/
	OPTOFORCE_API double							OptoPacket6DGetTzInNewtonMeter(OptoPacket6DHandle p_OptoPacket6D);
	/**
	* Retrieves the status word of an \ref OptoPacket6D instance
	* @param [in] p_OptoPacket6D The handle of \ref OptoPacket6D
	* @return the status word of an \ref OptoPacket6D instance
	*/
	OPTOFORCE_API unsigned short					OptoPacket6DGetStatus(OptoPacket6DHandle p_OptoPacket6D);
	/**
	* Retrieves the handle of the \ref OptoDebugPacket of an \ref OptoPacket6D instance
	* @param [in] p_OptoPacket6D The handle of \ref OptoPacket6D
	* @param [in] p_index The index of the requested debug packet (value should be 0, 1, 2 or 3)
	* @return the handle of the \ref OptoDebugPacket of the packets' debug packet
	*/
	OPTOFORCE_API const OptoDebugPacketHandle		OptoPacket6DGetDebugPacket(OptoPacket6DHandle p_OptoPacket6D, unsigned int p_index);
	/**
	* Sets the timestamp of an \ref OptoPacket6D instance
	* @param [in] p_OptoPacket6D The handle of \ref OptoPacket6D
	* @param [in] p_timeStamp The timestamp in microseconds
	*/
	OPTOFORCE_API void								OptoPacket6DSetTimeStamp(OptoPacket6DHandle p_OptoPacket6D, long long p_timeStamp);
	/**
	* Retrieves the timestamp of an \ref OptoPacket6D instance
	* @param [in] p_OptoPacket6D The handle of \ref OptoPacket3D
	* @return the instance's timestamp in microseconds
	*/
	OPTOFORCE_API long long							OptoPacket6DGetTimeStamp(OptoPacket6DHandle p_OptoPacket6D);
	/**
	* Generates a more simple structure from an \ref OptoPacket6D instance
	* @param [in] p_OptoPacket6D The handle of \ref OptoPacket6D
	* @return the generated \ref OptoSimplePacket3D structure
	*/
	OPTOFORCE_API OptoSimplePacket6D				OptoPacket6DToSimplePacket(OptoPacket6DHandle p_OptoPacket6D);
	/**
	* Determines if an \ref OptoPacket6D instance is valid
	* @param [in] p_OptoPacket6D The handle of \ref OptoPacket6D
	* @return a nonzero value if the instance is valid, otherwise 0
	*/
	OPTOFORCE_API int								OptoPacket6DIsValid(OptoPacket6DHandle p_OptoPacket6D);


	/**
	* Creates an \ref OptoPackets3D instance and retrieves the handle of the created instance
	* @param [in] p_Capacity the maximal capacity of the instance (See \ref OptoPackets3D for more information)
	* @return The handle of the created instance
	*/
	OPTOFORCE_API OptoPackets3DHandle				OptoPackets3DCreate(unsigned int p_Capacity);
	/**
	* Releases an \ref OptoPackets3D instance
	* @param [in] p_OptoPackets3D The instance which is intended to be released
	*/
	OPTOFORCE_API void								OptoPackets3DRelease(OptoPackets3DHandle p_OptoPackets3D);
	/**
	* Retrieves the requested \ref OptoPacket3D handle of an \ref OptoPackets3D instance
	* @param [in] p_OptoPackets3D The handle of \ref OptoPackets3D
	* @param [in] p_index The index of the requested packet
	* @return The requested handle of the \ref OptoPacket3D instance. The returned reference may be invalid, please
	* see \ref OptoPacket3DIsValid()
	*/
	OPTOFORCE_API const OptoPacket3DHandle			OptoPackets3DGetPacket(OptoPackets3DHandle p_OptoPackets3D, unsigned int p_index);
	/**
	* Clears the stored elements. The size of the container reduces to 0 but capacity remains the same. 
	* @param [in] p_OptoPackets3D The handle of \ref OptoPackets3D
	*/
	OPTOFORCE_API void								OptoPackets3DClear(OptoPackets3DHandle p_OptoPackets3D);
	/**
	* Resizes the container. 
	* @param [in] p_OptoPackets3D The handle of \ref OptoPackets3D
	* @param [in] p_newSize The intended new size of the container. 
	* - You can reduce the size of the container if you give smaller value than the current capacity
	* - You can release the memory used by the container if you give 0 as the new size
	* - The elements will not be erased but if the new capacity is smaller than the current size, elements that cannot be hold will be truncated from the right (i.e. from the bigger index) 
	* @return the new size of the instance
	*/
	OPTOFORCE_API unsigned int						OptoPackets3DResize(OptoPackets3DHandle p_OptoPackets3D, unsigned int p_newSize);
    /**
	* Retrieves the capacity of an \ref OptoPackets3D instance
	* @param [in] p_OptoPackets3D The handle of \ref OptoPackets3D
	* @return the capacity of the container
	*/
	OPTOFORCE_API unsigned int						OptoPackets3DGetCapacity(OptoPackets3DHandle p_OptoPackets3D);
	/**
	* Retrieves the current size of an \ref OptoPackets3D instance (the current number of elements the container holds)
	* @param [in] p_OptoPackets3D The handle of \ref OptoPackets3D
	* @return the current size of the instance
	*/
	OPTOFORCE_API unsigned int						OptoPackets3DGetSize(OptoPackets3DHandle p_OptoPackets3D);


	/**
	* Creates an \ref OptoPackets6D instance and retrieves the handle of the created instance
	* @param [in] p_Capacity the maximal capacity of the instance (See \ref OptoPackets6D for more information)
	* @return The handle of the created instance
	*/
	OPTOFORCE_API OptoPackets6DHandle				OptoPackets6DCreate(unsigned int p_Capacity);
	/**
	* Releases an \ref OptoPackets6D instance
	* @param [in] p_OptoPackets6D The instance which is intended to be released
	*/
	OPTOFORCE_API void								OptoPackets6DRelease(OptoPackets6DHandle p_OptoPackets6D);
	/**
	* Retrieves the requested \ref OptoPacket6D handle of an \ref OptoPackets6D instance
	* @param [in] p_OptoPackets6D The handle of \ref OptoPackets6D
	* @param [in] p_index The index of the requested packet
	* @return The requested handle of the \ref OptoPacket6D instance. The returned reference may be invalid, please
	* see \ref OptoPacket6DIsValid()
	*/
	OPTOFORCE_API const OptoPacket6DHandle			OptoPackets6DGetPacket(OptoPackets6DHandle p_OptoPackets6D, unsigned int p_index);
	/**
	* Clears the stored elements. The size of the container reduces to 0 but capacity remains the same.
	* @param [in] p_OptoPackets6D The handle of \ref OptoPackets6D
	*/
	OPTOFORCE_API void								OptoPackets6DClear(OptoPackets6DHandle p_OptoPackets6D);
	/**
	* Resizes the container.
	* @param [in] p_OptoPackets6D The handle of \ref OptoPackets6D
	* @param [in] p_newSize The intended new size of the container.
	* - You can reduce the size of the container if you give smaller value than the current capacity
	* - You can release the memory used by the container if you give 0 as the new size
	* - The elements will not be erased but if the new capacity is smaller than the current size, elements that cannot be hold will be truncated from the right (i.e. from the bigger index)
	* @return the new size of the instance
	*/
	OPTOFORCE_API unsigned int						OptoPackets6DResize(OptoPackets6DHandle p_OptoPackets6D, unsigned int p_newSize);
	/**
	* Retrieves the capacity of an \ref OptoPackets3D instance
	* @param [in] p_OptoPackets6D The handle of \ref OptoPackets6D
	* @return the capacity of the container
	*/
	OPTOFORCE_API unsigned int						OptoPackets6DGetCapacity(OptoPackets6DHandle p_OptoPackets6D);
	/**
	* Retrieves the current size of an \ref OptoPackets6D instance (the current number of elements the container holds)
	* @param [in] p_OptoPackets6D The handle of \ref OptoPackets6D
	* @return the current size of the instance
	*/
	OPTOFORCE_API unsigned int						OptoPackets6DGetSize(OptoPackets6DHandle p_OptoPackets6D);

	/**
	* Creates an \ref OptoDAQWatcher instance and retrieves the handle of the created instance
	* @return The handle of the created instance
	*/
	OPTOFORCE_API OptoDAQWatcherHandle				OptoDAQWatcherCreate();
	/**
	* Releases an \ref OptoDAQWatcher instance
	* @param [in] p_OptoDAQWatcher The instance which is intended to be released
	*/
	OPTOFORCE_API void								OptoDAQWatcherRelease(OptoDAQWatcherHandle p_OptoDAQWatcher);
	/**
	* Adds an address that should be not accessed by the \ref OptoDAQWatcher instance. 
	* This function can be used if you have other USB connected devices which must not 
	* be accessed by other applications. The unaccessable ports should be added before 
	* call of \ref OptoDAQWatcherStart()
	* @param [in] p_OptoDAQWatcher The handle of \ref OptoDAQWatcher
	* @param [in] p_Address a null-terminated string of the address of the device. (i.e. "COM3") 
	*/
	OPTOFORCE_API void								OptoDAQWatcherAddAddressToFilter(OptoDAQWatcherHandle p_OptoDAQWatcher, const char * p_Address);
	/**
	* Determines if there are any DAQs connected to the computer by USB.
	* @param [in] p_OptoDAQWatcher The handle of \ref OptoDAQWatcher
	* @return nonzero value if there are any DAQs connected to the computer by USB, otherwise 0. 
	*/
	OPTOFORCE_API int								OptoDAQWatcherHasConnectedDAQ(OptoDAQWatcherHandle p_OptoDAQWatcher);
	/**
	* Retrieves the connected DAQ's list
	* \n
	* Example:
	* \code
	* OptoDAQWatcherHandle daqWatcher = OptoDAQWatcherCreate();
	* OptoDAQWatcherStart(daqWatcher);
	* OptoDAQDescriptorHandle descriptors[16]; 
	* for (unisnged int i = 0; i < 16; ++i) {
	*   descriptors[i] = OptoDAQDescriptorCreate("");
	* }
	* unsigned int count = OptoDAQWatcherGetConnectedDAQs(daqWatcher, descriptors, 16, 1);
	* for (unsigned int i = 0; i < count; ++i) {
	*   printf("Address of connected DAQ is: %s\r\n", OptoDAQDescriptorGetAddress(descriptors[i]));
	* }
	* OptoDAQWatcherRelease(daqWatcher);
	* for (unsigned int i = 0; i < 16; ++i) {
	*   OptoDAQDescriptorRelease(descriptors[i]);
	* }
	* \endcode
	* @param [in] p_OptoDAQWatcher The handle of \ref OptoDAQWatcher
	* @param [out] p_descriptors The array of \ref OptoDAQDescriptor handles
	* @param [in] p_maxCount The maximal size of the p_descriptors array
	* @param [in] p_showOnlyNewest If this parameter is set to a nonzero value, 
	* the function only returns the most up-to-date list of descriptors: if no new DAQ 
	* is connected after the previous call of the function then no new item will be retrieved.
	* @return the size of the list of connected DAQs
	*/
	OPTOFORCE_API unsigned int						OptoDAQWatcherGetConnectedDAQs(OptoDAQWatcherHandle p_OptoDAQWatcher, OptoDAQDescriptorHandle * p_descriptors, unsigned int p_maxCount, int p_showOnlyNewest);
	/**
	* Retrieves the first DAQ found by the \ref OptoDAQWatcher instance. This is the most 
	* simple way of getting a descriptor if you have only one DAQ connected to the computer.
	* @param [in] p_OptoDAQWatcher The handle of \ref OptoDAQWatcher
	* @param [out] p_descriptor The descriptor that will hold the data of connected DAQ if any found
	*/
	OPTOFORCE_API int								OptoDAQWatcherGetFirstDAQ(OptoDAQWatcherHandle p_OptoDAQWatcher, OptoDAQDescriptorHandle p_descriptor);
	/**
	* Determines if there are any disconnected DAQs
	* @param [in] p_OptoDAQWatcher The handle of \ref OptoDAQWatcher
	* @return a nonzero if there are any DAQ that has been disconnected.
	*/
	OPTOFORCE_API int								OptoDAQWatcherHasDisconnectedDAQ(OptoDAQWatcherHandle p_OptoDAQWatcher);
	/**
	* Retrieves the disconnected DAQ's list. The usage is same as \ref OptoDAQWatcherGetConnectedDAQs()
	* @param [in] p_OptoDAQWatcher The handle of \ref OptoDAQWatcher
	* @param [out] p_descriptors The array of \ref OptoDAQDescriptor handles
	* @param [in] p_maxCount The maximal size of the p_descriptors array
	* @param [in] p_showOnlyNewest If this parameter is set to a nonzero value,
	* the function only returns the most up-to-date list of descriptors: if no new DAQ
	* is disconnected after the previous call of the function then no new item will be retrieved.
	* @return the size of the list of disconnected DAQs
	*/
	OPTOFORCE_API unsigned int						OptoDAQWatcherGetDisconnectedDAQs(OptoDAQWatcherHandle p_OptoDAQWatcher, OptoDAQDescriptorHandle * p_descriptors, unsigned int p_maxCount, int p_showOnlyNewest);
	/**
	* In most cases this function has not to be used. If you have a special
	* DAQ which baud rate is not the default, you must set the baudrate
	* using this function
	*/
	OPTOFORCE_API void								OptoDAQWatcherSetBaudRate(OptoDAQWatcherHandle p_OptoDAQWatcher, int p_baudRate);
	/**
	* Starts the instance's operation on a different thread. If you have limited CPU resources 
	* starting of the instance is not neccessary but in this case other function calls of OptoDAQWatcher will be blocking.
	* @param [in] p_OptoDAQWatcher The handle of \ref OptoDAQWatcher
	*/
	OPTOFORCE_API void								OptoDAQWatcherStart(OptoDAQWatcherHandle p_OptoDAQWatcher);
	/**
	* Stops the instance's operation. 
	* @param [in] p_OptoDAQWatcher The handle of \ref OptoDAQWatcher
	*/
	OPTOFORCE_API void								OptoDAQWatcherStop(OptoDAQWatcherHandle p_OptoDAQWatcher);

	/**
	* Creates an \ref OptoConfig instance and retrieves the handle of the created instance
	* @return The handle of the created instance
	*/
	OPTOFORCE_API OptoConfigHandle					OptoConfigCreate();
	/**
	* Releases an \ref OptoConfig instance
	* @param [in] p_OptoConfig The instance which is intended to be released
	*/
	OPTOFORCE_API void								OptoConfigRelease(OptoConfigHandle p_OptoConfig);
	/**
	* Sets the filter value of an \ref OptoConfig instance
	* @param [in] p_OptoConfig The handle of \ref OptoConfig
	* @param [in] p_Filter The value of filter to be set (See \ref OptoConfig::SetFilter for more information)
	*/
	OPTOFORCE_API void								OptoConfigSetFilter(OptoConfigHandle p_OptoConfig, int p_Filter);
	/**
	* Sets the speed value of an \ref OptoConfig instance
	* @param [in] p_OptoConfig The handle of \ref OptoConfig
	* @param [in] p_Speed The value of speed to be set (See \ref OptoConfig::SetSpeed for more information)
	*/
	OPTOFORCE_API void								OptoConfigSetSpeed(OptoConfigHandle p_OptoConfig, int p_Speed);
	/**
	* Sets the zeroing value of an \ref OptoConfig instance
	* @param [in] p_OptoConfig The handle of \ref OptoConfig
	* @param [in] p_zeroing The value of filter to be set (See \ref OptoConfig::SetZeroing for more information)
	*/
	OPTOFORCE_API void								OptoConfigSetZeroing(OptoConfigHandle p_OptoConfig, unsigned char p_zeroing); 
	/**
	* Retrieves an \ref OptoConfig instance's filter
	* @param [in] p_OptoConfig The handle of \ref OptoConfig	
	* @return Value of filter
	*/
	OPTOFORCE_API int								OptoConfigGetFilter(OptoConfigHandle p_OptoConfig);
	/**
	* Retrieves an \ref OptoConfig instance's speed
	* @param [in] p_OptoConfig The handle of \ref OptoConfig
	* @return Value of speed in Hz
	*/
	OPTOFORCE_API int								OptoConfigGetSpeed(OptoConfigHandle p_OptoConfig);
	/**
	* Retrieves an \ref OptoConfig instance's filter
	* @param [in] p_OptoConfig The handle of \ref OptoConfig
	* @return Value of filter in Hz
	*/
	OPTOFORCE_API double							OptoConfigGetFilterInHz(OptoConfigHandle p_OptoConfig);
	/**
	* Retrieves an \ref OptoConfig instance's hardware zeroing
	* @param [in] p_OptoConfig The handle of \ref OptoConfig
	* @return Value of zeroing (0 - off, 255 - on)
	*/
	OPTOFORCE_API unsigned char						OptoConfigGetZeroing(OptoConfigHandle p_OptoConfig);
	/**
	* Retrieves an \ref OptoConfig instance's mode
	* @param [in] p_OptoConfig The handle of \ref OptoConfig
	* @return > 0 if DAQ in compensated mode otherwise 0 
	*/
	OPTOFORCE_API unsigned char						OptoConfigGetMode(OptoConfigHandle p_OptoConfig);
	/**
	* Determines if an \ref OptoConfig instance is valid
	* @param [in] p_OptoConfig The handle of \ref OptoConfig
	* @return a nonzero value if the \ref OptoConfig instance is valid, otherwise 0
	*/
	OPTOFORCE_API int								OptoConfigIsValid(OptoConfigHandle p_OptoConfig);

	/**
	* Creates an \ref OptoSensitivity instance and retrieves the handle of the created instance
	* @param [in] p_NominalCapacity the value of nominal capacity
	* @param [in] p_Counts the value of counts
	* @return The handle of the created instance
	*/
	OPTOFORCE_API OptoSensitivityHandle				OptoSensitivityCreate(unsigned int p_NominalCapacity, unsigned int p_Counts);
	/**
	* Releases an \ref OptoSensitivity instance
	* @param [in] p_OptoSensitivity The instance which is intended to be released
	*/
	OPTOFORCE_API void								OptoSensitivityRelease(OptoSensitivityHandle p_OptoSensitivity);
	/**
	* Retrieves the nominal capacity of a \ref OptoSensitivity instance
	* @param [in] p_OptoSensitivity The handle of \ref OptoSensitivity
	* @return the nominal capacity
	*/
	OPTOFORCE_API unsigned int						OptoSensitivityGetNominalCapacity(OptoSensitivityHandle p_OptoSensitivity);
	/**
	* Retrieves the counts of a \ref OptoSensitivity instance
	* @param [in] p_OptoSensitivity The handle of \ref OptoSensitivity
	* @return the counts
	*/
	OPTOFORCE_API unsigned int						OptoSensitivityGetCounts(OptoSensitivityHandle p_OptoSensitivity);
	/**
	* Determines if an \ref OptoSensitivity instance is valid
	* @param [in] p_OptoSensitivity The handle of \ref OptoSensitivity
	* @return a noznero value if the instance is valid, otherwise 0
	*/
	OPTOFORCE_API int								OptoSensitivityIsValid(OptoSensitivityHandle p_OptoSensitivity);


	/**
	* Creates an \ref OptoSensitivityReport instance and retrieves the handle of the created instance
	* @return The handle of the created instance
	*/
	OPTOFORCE_API OptoSensitivityReportHandle		OptoSensitivityReportCreate();
	/**
	* Releases an \ref OptoSensitivityReport instance
	* @param [in] p_OptoSensitivityReport The instance which is intended to be released
	*/
	OPTOFORCE_API void								OptoSensitivityReportRelease(OptoSensitivityReportHandle p_OptoSensitivityReport);
	/**
	* Loads the sensitivity report from file (OSR format)
	* @param [in] p_OptoSensitivityReport The handle of \ref OptoSensitivityReport
	* @param [in] p_FileName A null-terminated string of the file name to be loaded
	* @return a nonzero value if the operation was successful, otherwise returns 0
	*/
	OPTOFORCE_API int								OptoSensitivityReportLoadFromFile(OptoSensitivityReportHandle p_OptoSensitivityReport, const char * p_FileName);
	/**
	* Determines if the sensitivity report is valid 
	* @param [in] p_OptoSensitivityReport The handle of \ref OptoSensitivityReport
	* return a nonzero value if the report is valid, otherwise 0
	*/
	OPTOFORCE_API int								OptoSensitivityReportIsValid(OptoSensitivityReportHandle p_OptoSensitivityReport);
	/**
	* Retrieves the handle of \ref OptoSensitivity which is associated to Fx - axis
	* @param [in] p_OptoSensitivityReport The handle of \ref OptoSensitivityReport
	* @return the handle of \ref OptoSensitivity which is associated to Fx-axis
	*/
	OPTOFORCE_API const OptoSensitivityHandle		OptoSensitivityReportGetFxSensitivity(OptoSensitivityReportHandle p_OptoSensitivityReport);
	/**
	* Retrieves the handle of \ref OptoSensitivity which is associated to Fy - axis
	* @param [in] p_OptoSensitivityReport The handle of \ref OptoSensitivityReport
	* @return the handle of \ref OptoSensitivity which is associated to Fy-axis
	*/
	OPTOFORCE_API const OptoSensitivityHandle		OptoSensitivityReportGetFySensitivity(OptoSensitivityReportHandle p_OptoSensitivityReport);
	/**
	* Retrieves the handle of \ref OptoSensitivity which is associated to Fz - axis
	* @param [in] p_OptoSensitivityReport The handle of \ref OptoSensitivityReport
	* @return the handle of \ref OptoSensitivity which is associated to Fz-axis
	*/
	OPTOFORCE_API const OptoSensitivityHandle		OptoSensitivityReportGetFzSensitivity(OptoSensitivityReportHandle p_OptoSensitivityReport);
	/**
	* Retrieves the handle of \ref OptoSensitivity which is associated to Tx - axis
	* @param [in] p_OptoSensitivityReport The handle of \ref OptoSensitivityReport
	* @return the handle of \ref OptoSensitivity which is associated to Tx-axis
	*/
	OPTOFORCE_API const OptoSensitivityHandle		OptoSensitivityReportGetTxSensitivity(OptoSensitivityReportHandle p_OptoSensitivityReport);
	/**
	* Retrieves the handle of \ref OptoSensitivity which is associated to Ty - axis
	* @param [in] p_OptoSensitivityReport The handle of \ref OptoSensitivityReport
	* @return the handle of \ref OptoSensitivity which is associated to Ty-axis
	*/
	OPTOFORCE_API const OptoSensitivityHandle		OptoSensitivityReportGetTySensitivity(OptoSensitivityReportHandle p_OptoSensitivityReport);
	/**
	* Retrieves the handle of \ref OptoSensitivity which is associated to Tz - axis
	* @param [in] p_OptoSensitivityReport The handle of \ref OptoSensitivityReport
	* @return the handle of \ref OptoSensitivity which is associated to Tz-axis
	*/
	OPTOFORCE_API const OptoSensitivityHandle		OptoSensitivityReportGetTzSensitivity(OptoSensitivityReportHandle p_OptoSensitivityReport);
	/**
	* Calculates Fx as Newton from counts
	* @param [in] p_OptoSensitivityReport The handle of \ref OptoSensitivityReport
	* @param [in] p_counts the counts value which is intended to be converted to Newton
	* @return the Fx value in Newton
	*/
	OPTOFORCE_API double							OptoSensitivityReportCalcFx(OptoSensitivityReportHandle p_OptoSensitivityReport, int p_counts);
	/**
	* Calculates Fy as Newton from counts
	* @param [in] p_OptoSensitivityReport The handle of \ref OptoSensitivityReport
	* @param [in] p_counts the counts value which is intended to be converted to Newton
	* @return the Fy value in Newton
	*/
	OPTOFORCE_API double							OptoSensitivityReportCalcFy(OptoSensitivityReportHandle p_OptoSensitivityReport, int p_counts);
	/**
	* Calculates Fz as Newton from counts
	* @param [in] p_OptoSensitivityReport The handle of \ref OptoSensitivityReport
	* @param [in] p_counts the counts value which is intended to be converted to Newton
	* @return the Fz value in Newton
	*/
	OPTOFORCE_API double							OptoSensitivityReportCalcFz(OptoSensitivityReportHandle p_OptoSensitivityReport, int p_counts);
	/**
	* Calculates Tx as Newton meter from counts
	* @param [in] p_OptoSensitivityReport The handle of \ref OptoSensitivityReport
	* @param [in] p_counts the counts value which is intended to be converted to Newton meter
	* @return the Tx value in Newton meter
	*/
	OPTOFORCE_API double							OptoSensitivityReportCalcTx(OptoSensitivityReportHandle p_OptoSensitivityReport, int p_counts);
	/**
	* Calculates Ty as Newton meter from counts
	* @param [in] p_OptoSensitivityReport The handle of \ref OptoSensitivityReport
	* @param [in] p_counts the counts value which is intended to be converted to Newton meter
	* @return the Ty value in Newton meter
	*/
	OPTOFORCE_API double							OptoSensitivityReportCalcTy(OptoSensitivityReportHandle p_OptoSensitivityReport, int p_counts);
	/**
	* Calculates Tz as Newton meter from counts
	* @param [in] p_OptoSensitivityReport The handle of \ref OptoSensitivityReport
	* @param [in] p_counts the counts value which is intended to be converted to Newton meter
	* @return the Tz value in Newton meter
	*/
	OPTOFORCE_API double							OptoSensitivityReportCalcTz(OptoSensitivityReportHandle p_OptoSensitivityReport, int p_counts);
	/**
	* Retrieves the number of axises of an \ref OptoSensitivityReport instance
	* @param [in] p_OptoSensitivityReport The handle of \ref OptoSensitivityReport
	* @return 6 if the report is for a 6-axis DAQ, 3 if the report is for a 3-axis DAQ, every other values are considered invalid
	*/
	OPTOFORCE_API unsigned int						OptoSensitivityReportGetAxisCount(OptoSensitivityReportHandle p_OptoSensitivityReport);
	/**
	* Sets the number of axises of an \ref OptoSensitivityReport
	* @param [in] p_OptoSensitivityReport The handle of \ref OptoSensitivityReport
	* @param [in] p_AxisCount The number of axises (currenty only 3 and 6 are valid values)
	*/
	OPTOFORCE_API void								OptoSensitivityReportSetAxisCount(OptoSensitivityReportHandle p_OptoSensitivityReport, unsigned int p_AxisCount);
	/**
	* Sets the \ref OptoSensitivity associated to Fx - axis
	* @param [in] p_OptoSensitivityReport The handle of \ref OptoSensitivityReport
	* @param [in] p_Sensitivity The \ref OptoSensitivity handle which is intended to be associated to Fx - axis
	*/
	OPTOFORCE_API void								OptoSensitivityReportSetFxSensitivity(OptoSensitivityReportHandle p_OptoSensitivityReport, const OptoSensitivityHandle p_Sensitivity);
	/**
	* Sets the \ref OptoSensitivity associated to Fy - axis
	* @param [in] p_OptoSensitivityReport The handle of \ref OptoSensitivityReport
	* @param [in] p_Sensitivity The \ref OptoSensitivity handle which is intended to be associated to Fy - axis
	*/
	OPTOFORCE_API void								OptoSensitivityReportSetFySensitivity(OptoSensitivityReportHandle p_OptoSensitivityReport, const OptoSensitivityHandle p_Sensitivity);
	/**
	* Sets the \ref OptoSensitivity associated to Fz - axis
	* @param [in] p_OptoSensitivityReport The handle of \ref OptoSensitivityReport
	* @param [in] p_Sensitivity The \ref OptoSensitivity handle which is intended to be associated to Fz - axis
	*/
	OPTOFORCE_API void								OptoSensitivityReportSetFzSensitivity(OptoSensitivityReportHandle p_OptoSensitivityReport, const OptoSensitivityHandle p_Sensitivity);
	/**
	* Sets the \ref OptoSensitivity associated to Tx - axis
	* @param [in] p_OptoSensitivityReport The handle of \ref OptoSensitivityReport
	* @param [in] p_Sensitivity The \ref OptoSensitivity handle which is intended to be associated to Tx - axis
	*/
	OPTOFORCE_API void								OptoSensitivityReportSetTxSensitivity(OptoSensitivityReportHandle p_OptoSensitivityReport, const OptoSensitivityHandle p_Sensitivity);
	/**
	* Sets the \ref OptoSensitivity associated to Ty - axis
	* @param [in] p_OptoSensitivityReport The handle of \ref OptoSensitivityReport
	* @param [in] p_Sensitivity The \ref OptoSensitivity handle which is intended to be associated to Ty - axis
	*/
	OPTOFORCE_API void								OptoSensitivityReportSetTySensitivity(OptoSensitivityReportHandle p_OptoSensitivityReport, const OptoSensitivityHandle p_Sensitivity);
	/**
	* Sets the \ref OptoSensitivity associated to Tz - axis
	* @param [in] p_OptoSensitivityReport The handle of \ref OptoSensitivityReport
	* @param [in] p_Sensitivity The \ref OptoSensitivity handle which is intended to be associated to Tz - axis
	*/
	OPTOFORCE_API void								OptoSensitivityReportSetTzSensitivity(OptoSensitivityReportHandle p_OptoSensitivityReport, const OptoSensitivityHandle p_Sensitivity);



	
	/**
	* Creates an \ref OptoDAQ instance and retrieves the handle of the created instance
	* @param [in] p_OptoDAQDescriptor The handle of an \ref OptoDAQDescriptor holds information
	* of the DAQ which is intended to be opened
	* @param [in] p_maxPacketCount The maximal number of packets (See the constructor of \ref OptoDAQ for more information)
	* @return The handle of the created instance
	*/
	OPTOFORCE_API OptoDAQHandle						OptoDAQCreate(OptoDAQDescriptorHandle p_OptoDAQDescriptor, unsigned int p_maxPacketCount);
	/**
	* Releases an \ref OptoDAQ instance
	* @param [in] p_OptoDAQ The instance which is intended to be released
	*/
	OPTOFORCE_API void								OptoDAQRelease(OptoDAQHandle p_OptoDAQ);
	/**
	* Opens DAQ. After successful call \ref OptoDAQ starts operating
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @return nonzero value if DAQ could be opened, otherwise returns 0
	*/
	OPTOFORCE_API int								OptoDAQOpen(OptoDAQHandle p_OptoDAQ);
	/**
	* Closes DAQ. After this call no new packets will be processed, the communication channel
	* will be closed (other applications can access the port)
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	*/
	OPTOFORCE_API void								OptoDAQClose(OptoDAQHandle p_OptoDAQ);
	/**
	* Determines if an \ref OptoDAQ instance is a 3-axis DAQ 
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @return a nonzero value if the connected DAQ is a 3-axis DAQ, otherwise 0
	*/
	OPTOFORCE_API int								OptoDAQIs3D(OptoDAQHandle p_OptoDAQ);
	/**
	* Determines if an \ref OptoDAQ instance is a 6-axis DAQ
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @return a nonzero value if the connected DAQ is a 6-axis DAQ, otherwise 0
	*/
	OPTOFORCE_API int								OptoDAQIs6D(OptoDAQHandle p_OptoDAQ);
	/**
	* Writes custom data to the DAQ.
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @param [in] p_Bytes Array of bytes that should be written to the DAQ 
	* @param [in] p_Length The length of the array 
	* @return a nonzero value if write was successful, otherwise 0.
	*/
	OPTOFORCE_API int								OptoDAQWrite(OptoDAQHandle p_OptoDAQ, const unsigned char * p_Bytes, unsigned int p_Length);
	/**
	* Retrieves the handle of \ref OptoDAQDescriptor associated to DAQ 
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	*/
	OPTOFORCE_API const OptoDAQDescriptorHandle		OptoDAQGetDescriptor(OptoDAQHandle p_OptoDAQ);
	/**
	* Sets the associated OptoDAQDescriptor. By calling this function \ref OptoDAQ will become invalid,
	* previous connection will be closed. To start using it again you have to call \ref OptoDAQOpen()
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @param [in] p_Descriptor The handle of \ref OptoDAQDescriptor
	*/
	OPTOFORCE_API void								OptoDAQSetDescriptor(OptoDAQHandle p_OptoDAQ, const OptoDAQDescriptorHandle p_Descriptor);
	/**
	* Sends config to the DAQ. (See \ref OptoConfig) 
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @param [in] p_Config The handle of \ref OptoConfig which is intended to send to DAQ
	* @return a nonzero value if send was successful, otherwise 0
	*/
	OPTOFORCE_API int								OptoDAQSendConfig(OptoDAQHandle p_OptoDAQ, const OptoConfigHandle p_Config);
	/**
	* Retrieves the config of an \ref OptoDAQ instance
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @return handle of instance of the current \ref OptoConfig of \ref OptoDAQ
	*/
	OPTOFORCE_API const OptoConfigHandle			OptoDAQGetConfig(OptoDAQHandle p_OptoDAQ);
	/**
	* Puts an instance of \ref OptoDAQ to debug mode
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @param [in] p_DebugMode nonzero value sets debug mode, zero sets non-debug mode. See \ref OptoDAQ::SetDebugMode for more information
	* @return a nonzero value if the intended mode is could be set, otherwise 0
	*/
	OPTOFORCE_API int								OptoDAQSetDebugMode(OptoDAQHandle p_OptoDAQ, int p_DebugMode);
	/**
	* Requests sensitivity report from the connected DAQ represented by an \ref OptoDAQ instance
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @return a nonzero value if the \ref OptoSensitivityReport could be obtained, otherwise 0
	*/
	OPTOFORCE_API int								OptoDAQRequestSensitivityReport(OptoDAQHandle p_OptoDAQ);
	/**
	* Sets the \ref OptoSensitivityReport of an \ref OptoDAQ instance
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @param [in] p_SensitivityReport The handle of the \ref OptoSensitivityReport
	*/
	OPTOFORCE_API void								OptoDAQSetSensitivityReport(OptoDAQHandle p_OptoDAQ, const OptoSensitivityReportHandle p_SensitivityReport);
	/**
	* Loads sensitivity report from file (OSR format)
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @param [in] p_FileName A null-terminated string of the file name to be loaded
	* @return a nonzero value if the operation was successful, otherwise returns 0
	*/
	OPTOFORCE_API int								OptoDAQLoadSensitivityReport(OptoDAQHandle p_OptoDAQ, const char * p_FileName);
	/**
	* Retrieves current sensitivity report of an instance of \ref OptoDAQ (to get a valid sensitivity report 
	* you should first load it from file or request it from the DAQ using \ref OptoDAQLoadSensitivityReport() or 
	* \ref OptoDAQRequestSensitivityReport()) 
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @return the handle of current \ref OptoSensitivityReport used by \ref OptoDAQ instance 
	*/
	OPTOFORCE_API const OptoSensitivityReportHandle	OptoDAQGetSensitivityReport(OptoDAQHandle p_OptoDAQ);
	/**
	* Retrieves \ref OptoPacket3D packets from an instance of \ref OptoDAQ
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @param [in] p_Packets The handle of an instance of \ref OptoPackets3D
	* @param [in] p_Block (0 - non-blocking mode, nonzero - blocking mode), see \ref OptoDAQ::OptoDAQGetPackets3D
	*/
	OPTOFORCE_API void								OptoDAQGetPackets3D(OptoDAQHandle p_OptoDAQ, OptoPackets3DHandle p_Packets, int p_Block);
	/**
	* Retrieves \ref OptoPacket6D packets from an instance of \ref OptoDAQ
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @param [in] p_Packets The handle of an instance of \ref OptoPackets3D
	* @param [in] p_Block (0 - non-blocking mode, nonzero - blocking mode), see \ref OptoDAQ::OptoDAQGetPackets6D
	*/
	OPTOFORCE_API void								OptoDAQGetPackets6D(OptoDAQHandle p_OptoDAQ, OptoPackets6DHandle p_Packets, int p_Block);
	/**
	* Retrieves last valid \ref OptoPacket3D from an instance of \ref OptoDAQ. 
	* The internal buffer will be cleared after this call
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @param [in] p_Packet The handle of \ref OptoPacket3D which will hold the result
	* @param [in] p_Block (0 - non-blocking mode, nonzero - blocking mode), see \ref OptoDAQ::GetLastPacket3D
	*/
	OPTOFORCE_API int								OptoDAQGetLastPacket3D(OptoDAQHandle p_OptoDAQ, OptoPacket3DHandle p_Packet, int p_Block);
	/**
	* Retrieves last valid \ref OptoPacket6D from an instance of \ref OptoDAQ.
	* The internal buffer will be cleared after this call
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @param [in] p_Packet The handle of \ref OptoPacket6D which will hold the result
	* @param [in] p_Block (0 - non-blocking mode, nonzero - blocking mode), see \ref OptoDAQ::GetLastPacket6D
	*/
	OPTOFORCE_API int								OptoDAQGetLastPacket6D(OptoDAQHandle p_OptoDAQ, OptoPacket6DHandle p_Packet, int p_Block);
	/**
	* Clears the internal buffer of the instance of an \ref OptoDAQ
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	*/
	OPTOFORCE_API void								OptoDAQClearPackets(OptoDAQHandle p_OptoDAQ);
	/**
	* Retrieves the number of \ref OptoPacket3D packets currently stored by an instance of \ref OptoDAQ
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @return the number of \ref OptoPacket3D packets currently stored in the internal buffer of the \ref OptoDAQ 
	*/
	OPTOFORCE_API unsigned int						OptoDAQGetPackets3DCount(OptoDAQHandle p_OptoDAQ);
	/**
	* Retrieves the number of \ref OptoPacket6D packets currently stored by an instance of \ref OptoDAQ
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @return the number of \ref OptoPacket6D packets currently stored in the internal buffer of the \ref OptoDAQ
	*/
	OPTOFORCE_API unsigned int						OptoDAQGetPackets6DCount(OptoDAQHandle p_OptoDAQ);
	/**
	* Retrieves the number of packets dropped by an instance of \ref OptoDAQ. For more information
	* see \ref OptoDAQ::GetDroppedPacketsCount()
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @return the number of dropped packets
	*/
	OPTOFORCE_API unsigned int						OptoDAQGetDroppedPacketsCount(OptoDAQHandle p_OptoDAQ);
	/**
	* Retrieves the number of sensors connected to an instance of \ref OptoDAQ
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @return the number of sensors connected to the DAQ (currently 1 or 4 considered valid values)
	*/
	OPTOFORCE_API unsigned int						OptoDAQGetSensorCount(OptoDAQHandle p_OptoDAQ);
	/**
	* Retrieves the number of checksum errors of an instance of \ref OptoDAQ
	* The value should be zero if the DAQ is operating properly. 
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @return the number of checksum errors
	*/
	OPTOFORCE_API unsigned int						OptoDAQGetChecksumErrorCount(OptoDAQHandle p_OptoDAQ);
	/**
	* Zeroes the given channel of an instance of \ref OptoDAQ. For more information see \ref OptoDAQ::Zero()
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @param [in] p_Channel the index of the channel
	* @return a nonzero value if operation was successful, otherwise returns 0
	*/
	OPTOFORCE_API int								OptoDAQZero(OptoDAQHandle p_OptoDAQ, unsigned int p_Channel);
	/**
	* Zeroes all of the channels of an instance of \ref OptoDAQ. For more information see \ref OptoDAQ::ZeroAll
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @return a nonzero value if operation was successful, otherwise returns 0
	*/
	OPTOFORCE_API int								OptoDAQZeroAll(OptoDAQHandle p_OptoDAQ);
	/**
	* Unzeroes the given channel of an instance of \ref OptoDAQ. For more information see \ref OptoDAQ::Unzero()
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @param [in] p_Channel the index of the channel
	* @return a nonzero value if operation was successful, otherwise returns 0
	*/
	OPTOFORCE_API int								OptoDAQUnzero(OptoDAQHandle p_OptoDAQ, unsigned int p_Channel);
	/**
	* Unzeroes all of the channels of an instance of \ref OptoDAQ. For more information see \ref OptoDAQ::UnzeroAll
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @return a nonzero value if operation was successful, otherwise returns 0
	*/
	OPTOFORCE_API int								OptoDAQUnzeroAll(OptoDAQHandle p_OptoDAQ);
	/**
	* Determines if an instance of \ref OptoDAQ is valid. For more information see \ref OptoDAQ::IsValid()
	* @param [in] p_OptoDAQ The handle of \ref OptoDAQ
	* @return a nonzero value if DAQ is valid, otherwise returns 0
	*/
	OPTOFORCE_API int								OptoDAQIsValid(OptoDAQHandle  p_OptoDAQ);
#ifdef __cplusplus
}
#endif