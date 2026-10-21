#pragma once
#include <cstddef>
#include <vector>
#include "OptoForceAPI.h"
#include "OptoDAQDescriptor.h"
#include "OptoPacket3D.h"
#include "OptoPacket6D.h"
#include "OptoPackets3D.h"
#include "OptoPackets6D.h"
#include "OptoConfig.h"
#include "OptoSimpleRawPacket.h"
#include "OptoSensitivityReport.h"
/**
* This class represents a DAQ connected to the computer.
* \n
* Using this class you are able to
* - Communicate with connected DAQs
* - Set up speed/filter/mode of connected DAQs
* - Set zeroing/unzeroing
* - Write raw data to the DAQ
* - Read 3-axis and 6-axis packets
* - etc
* \note
* An \ref OptoDAQ instance can be accessed from multiple different threads (SendConfig, GetPackets, etc)
* \attention
* Please keep in mind the following:
*  - You should pass OptoDAQ instances to functions as reference or pointer, never by value!
*  - You should not use the assignment operator after you called \ref OptoDAQ::Open(): (i.e. no a = b)
*  - Because of the above you should keep your instances in containers as pointers (std::vector<OptoDAQ *>)

* \n
* Simple usage:
* \code
* OptoDAQWatcher daqWatcher;
* daqWatcher.Start();
* OptoDAQ *optoDAQ = NULL;
* while (true) {
*   OptoDAQDescriptor descriptor;
*   if (daqWatcher.GetFirstDAQ(&descriptor) == true) {
*      optoDAQ = new OptoDAQ(descriptor, 1000);
*      break;
*   }
* }
* OptoPackets3D packets3D(1000);
* OptoPackets6D packets6D(1000);
* while (optoDAQ->IsValid()) {
*    if (optoDAQ->Is3D()) {
*       packets3D.Clear();
*       optoDAQ->GetPackets3D(packets3D, false);
*       for (std::size_t i = 0; i < packets3D.GetSize(); ++i) {
*           OptoPacket3D packet = packets3D.GetPacket(i);
*           // Here you can process the actual packet;
*       }
*    }
*    if (optoDAQ->Is6D()) {
*       packets6D.Clear();
*       optoDAQ->GetPackets6D(packets6D, false);
*       for (std::size_t i = 0; i < packets6D.GetSize(); ++i) {
*          OptoPacket6D packet = packets6D.GetPacket(i);
*          // Here you can process the actual packet
*       }
*    }
* }
* delete optoPacket; // release used memory
* \endcode
*/
class OPTOFORCE_API OptoDAQ
{
public:
	/**
	* Default constructor
	*/
	OptoDAQ();
	/**
	* Constructor
	* @param [in] p_maximalPacketCount This is the maximal count of \ref OptoPacket3D or \ref OptoPacket6D
	* packets will be hold by the OptoDAQ instance. 
	* \n
	* Explanation: if you set this value to 1000 and the speed of the DAQ is set to 100 then the object can hold
	* packets of the last 10 seconds. After the count of stored packets reaches the limit, the oldest
	* packets will be dropped.
	* @param [in] p_descriptor The OptoDAQDescriptor which identifies the DAQ (See \ref OptoDAQWatcher)
	* Note: You can use custom \ref OptoDAQDescriptor to initialize an \ref OptoDAQ instance, but in this case
	* you must setup the type-name of the DAQ!
	* \n
	* Example of custom OptoDAQDescriptor usage:
	* \code
	* OptoDAQDescriptor myDescriptor;
	* myDescriptor.SetAddress("COM3");
	* myDescriptor.SetTypeName("31"); //This is a must! Currently acceptable values are:
	*                                 // "31": 3-axis-1-channel DAQ
	*                                 // "34": 3-axis-multichannel DAQ
	*                                 // "64": 6-axis DAQ
	* OptoDAQ myDAQ(myDescriptor, 1000);
	* myDAQ.Open(); // This will open COM3 port assuming it is a 3-axis-1-channel DAQ
	* \endcode
	*/
	OptoDAQ(const OptoDAQDescriptor & p_descriptor, std::size_t p_maximalPacketCount = 10000);
	/**
	* Copy constructor
	*/
	OptoDAQ(OptoDAQ & p_other);
	/**
	* Assignment operator
	*/
	OptoDAQ & operator = (const OptoDAQ & p_other);
	/**
	* Destructor
	*/
	virtual ~OptoDAQ();
public:
	/**
	* Opens DAQ. After successful call DAQ starts operating.
	* @return true if DAQ could be opened otherwise false
	*/
	bool					Open();
	/**
	* Closes DAQ. After this call no new packets will be processed, the 
	* communication channel will be closed (other applications can access the port)
	*/
	void					Close();
	/**
	* Writes custom data to the DAQ.
	* @param [in] p_bytes Array of bytes that should be written to the DAQ
	* @param [in] p_length The length of the array
	* @return true if write was successful otherwise false
	*/
	bool					Write(const unsigned char * p_bytes, std::size_t p_length);
	/**
	* Retrieves the \ref OptoDAQDescriptor associated to DAQ
	* @return The associated \ref OptoDAQDescriptor
	*/
	const OptoDAQDescriptor	&			GetOptoDAQDescriptor() const;
	/**
	* Sets the associated \ref OptoDAQDescriptor. By calling this function \ref OptoDAQ will become
	* invalid, previous connection will be closed. To start using it again you have to call \ref
	* OptoDAQ::Open()
	* @param [in] p_descriptor The descriptor has to be associated to the \ref OptoDAQ
	*/
	void							SetOptoDAQDescriptor(const OptoDAQDescriptor & p_descriptor);
	/**
	* Sends config to the DAQ. (See \ref OptoConfig)
	* @param [in] p_config The config intended to send to the DAQ
	* @return true if config sent successfully otherwise false
	*/
	bool							SendConfig(const OptoConfig & p_config);
	/**
	* Retrieves OptoDAQ's current config
	* @return the current config of OptoDAQ
	*/
	const OptoConfig    &			GetConfig();
	/**
	* Sets debug mode
	* @param [in] p_debugMode If the flag is set to true OptoDAQ will start to operate in debug mode otherwise
	* it will operate in normal mode. Default mode is non-debug mode.
	* @return true if intended debug mode could be set.
	* \n
	* Warning: if you set 3-axis-multi channel or 6-axis DAQ to operate in debug mode, 1000 Hz speed cannot be reached.
	*/
	bool							SetDebugMode(bool p_debugMode);
	/**
	* Loads 6-axis' DAQ sensitivity report file (.OSR). If sensitivity report is loaded it is applied to all of 
	* received 6-axis packets so N/mNm values can be obtained.
	* @return true if OSR file could be loaded otherwise false (if file could not be found or corrupted)
	*/
	bool							LoadSensitivityReport(const char * p_fileName);
	/**
	* Requests sensitivity report from the DAQ
	* @return true if sensitivity report could be obtained otherwise false
	*/
	bool							RequestSensitivityReport();
	/**
	* Retrieves current sensitivity report of the DAQ (to get a valid sensitivity report you should firts load it from file
	* or request it from the DAQ by calling \ref OptoDAQ::LoadSensitivityReport() or \ref OptoDAQ::RequestSensitivityReport() )
	* @return the current \ref OptoSensitivityReport used by \ref OptoDAQ instance
	*/
	const OptoSensitivityReport	&	GetSensitivityReport();
	/**
	* Sets custom sensitivity report
	* @param [in] p_sensitivityReport the custom \ref OptoSensitivityReport which was loaded from file or created manually.
	*/
	void							SetSensitivityReport(const OptoSensitivityReport & p_sensitivityReport);
	/**
	* Retrieves last valid packet from DAQ. The internal buffer will be cleared after this call.
	* @param [out] p_packet the last valid \ref OptoPacket3D. The packet could be an invalid packet if read timeout occurs.
	* @param [in] p_block determines if call should be blocking. If set to true the call will be block until new
	* packet received from the DAQ. The timeout of the call in ms is calculated by this formula: 
	* 1 / DAQ's speed in Hz + delta where delta is >> 1;
	* \n
	* Example to reach ~1000 Hz loop
	* \code
	* daq.SendConfig(OptoConfig(1000, 4, 0, 0)); // Set speed of DAQ to 1000 Hz
	* while (true) {
	*	OptoPacket3D packet;
	*   daq.GetLastPacket(&packet, true);
	* }
	* \endcode
	* @return true if packet received from DAQ before timeout otherwise false
	*/
	bool							GetLastPacket3D(OptoPacket3D * p_packet, bool p_block = false);
	/**
	* Retrieves packets from DAQ.
	* \n
	* Simple usage (non-blocking, see \ref OptoPackets3D too for details):
	* \code
	* OptoPackets3D packets(1000);
	* while (true) {
	*	packets.Clear(); //Clear previous results
	*   daq.GetPackets3D(packets, false);
	*   std::size_t packetCount = packets.GetSize(); // packetCount can be zero if no new packets received from DAQ
	* }
	* \endcode
	* Simple usage (blocking):
	* \code
	* OptoPackets3D packets(10); // Note: the capacity is only 10
	* while (true) {
	*	packets.Clear(); //Clear previous results
	*	daq.GetPackets3D(packets, true); // This call will block until 10 packets received from DAQ
	*	std::size_t packetCount = packets.GetSize();  // If no timeout occured packetCount must be 10
	* }
	* \endcode
	* @param [out] p_packets The \ref OptoPackets3D structure which will hold the result. The function call will
	* erase as many elements from the internal buffer as many could be moved to p_packets.
	* @param [in] p_block determines if call should be blocking. If p_block is true then function will
	* block until as many packets received from DAQ as many free space is available in p_packets
	*/
	void							GetPackets3D(OptoPackets3D * p_packets, bool p_block = false);
	/**
	* Retrieves last valid packet from DAQ. The internal buffer will be cleared after this call.
	* @param [out] p_packet the last valid \ref OptoPacket6D. The packet could be an invalid packet if read timeout occurs.
	* @param [in] p_block determines if call should be blocking. If set to true the call will be block until new
	* packet received from the DAQ. The timeout of the call in ms is calculated by this formula:
	* 1 / DAQ's speed in Hz + delta where delta is >> 1;
	* \n
	* Example to reach ~1000 Hz loop:
	* \code
	* daq.SendConfig(OptoConfig(1000, 4, 0, 0)); // Set speed of DAQ to 1000 Hz
	* while (true) {
	*	OptoPacket6D packet;
	*   daq.GetLastPacket(&packet, true);
	* }
	* \endcode
	* @return true if packet received from DAQ before timeout otherwise false
	*/
	bool							GetLastPacket6D(OptoPacket6D * p_packet, bool p_block = false);

	/*
	* Retrieves a bootloader packet from the DAQ
	* In the most case, this function should not be used by an avarage user
	* @param [in|out] p_packet the packet that will contain the boot loader packet
	* @param [in] p_block if true, function call will block until packet received from DAQ or timeout
	* & return true if packet received from DAQ before timeout otherwise false
	*/
	bool							GetLastBootLoaderPacket(OptoSimpleRawPacket * p_packet, bool p_block = false);
	/**
	* Retrieves packets from DAQ.
	* \n
	* Simple usage (non-blocking, see \ref OptoPackets6D too for details):
	* \code
	* OptoPackets6D packets(1000);
	* while (true) {
	*	packets.Clear(); //Clear previous results
	*   daq.GetPackets6D(packets, false);
	*   std::size_t packetCount = packets.GetSize(); // packetCount can be zero if no new packets received from DAQ
	* }
	* \endcode
	* Simple usage (blocking):
	* \code
	* OptoPackets6D packets(10); // Note: the capacity is only 10
	* while (true) {
	*	packets.Clear(); //Clear previous results
	*	daq.GetPackets6D(packets, true); // This call will block until 10 packets received from DAQ
	*	std::size_t packetCount = packets.GetSize();  // If no timeout occured packetCount must be 10
	* }
	* \endcode
	* @param [out] p_packets The \ref OptoPackets6D structure which will hold the result. The function call will
	* erase as many elements from the internal buffer as many could be moved to p_packets.
	* @param [in] p_block determines if call should be blocking. If p_block is true then function will
	* block until as many packets received from DAQ as many free space is available in p_packets
	*/
	void							GetPackets6D(OptoPackets6D * p_packets, bool p_block = false);
	/**
	* Clears the internal buffer. It may be used after \ref OptoDAQ::Open() called or new configuration sent. 
	*/
	void							ClearPackets();
	/**
	* Retrieves the number of \ref OptoPacket3D packets currently stored in the internal buffer
	* @return the number of \ref OptoPacket3D packets currently stored in the internal buffer
	*/
	std::size_t						GetPackets3DCount() const;
	/**
	* Retrieves the number of \ref OptoPacket6D packets currently stored in the internal buffer
	* @return the number of \ref OptoPacket6D packets currently stored in the internal buffer
	*/
	std::size_t						GetPackets6DCount() const;
	/**
	* Retrieves the number of packets dropped from the internal buffer by reason of buffer overflow
	* @return the number of packets dropped from internal buffer
	*/
	std::size_t						GetDroppedPacketsCount() const;
	/**
	* Retrieves the number of sensors connected to the DAQ
	* @return the number of sensors connected to the DAQ
	*/
	std::size_t						GetSensorCount()		const;
	/**
	* Retrieves the number of checksum errors. The value should be zero if the DAQ is operating properly.
	* @return the number of checksum errors
	*/
	std::size_t						GetChecksumErrorCount() const;
	/**
	* Zeroes the given channel.
	* @param [in] p_channel The index of channel intended to be zeroed. Values can be 0, 1, 2 or 3.
	* @return true if zeroing was successful otherwise false 
	* Simple usage (on a 3-axis-multi channel DAQ):
	* \code
	* while (true) {
	*   OptoPacket3D packet;
	*   daq.GetLastPacket(&packet, true);
	*   std::cout<<packet.GetFxAsCounts(0)<<std::endl; // The output is a non-zero value like (12345)
	*   daq.Zero(0);
	*   daq.GetLastPacket(&packet, true);
	*   std::cout<<packet.GetFxAsCounts(0)<<std::endl; // The output is a near-to-zero value (like -1, 0, 1)
	* }
	* \endcode
	*/
	bool							Zero(std::size_t p_channel);
	/**
	* Zeroes all of the channels (in the case of 3-axis-multi channel DAQ all of the four channels)
	* @return true if zeroing was successful otherwise false
	*/
	bool							ZeroAll();
	/**
	* Unzeroes the given channel. Opposite function of \ref OptoDAQ::Zero.
	* @param [in] p_channel The index of channel intended to be unzeroed. Values can be 0, 1, 2 or 3.
	* @return true if unzeroing was successful otherwise false
	*/
	bool							Unzero(std::size_t p_channel);
	/**
	* Unzeroes all of the channels. Opposite function of \ref OptoDAQ::ZeroAll
	* @return true if unzeroing was successful otherwise false
	*/
	bool							UnzeroAll();
	/**
	* Determines if the connected DAQ is a 3-axis DAQ
	* @return true if the connected DAQ is a 3-axis DAQ otherwise false
	*/
	bool							Is3D() const;
	/**
	* Determines if the connected DAQ is a 6-axis DAQ
	* @return true if the connected DAQ is a 6-axis DAQ otherwise false
	*/
	bool							Is6D() const;
	/**
	* Determines if \ref OptoDAQ is in valid state. If a read timeout occured or DAQ has been disconnected
	* the function will return false.
	* \n
	* Simple usage:
	* \code
	* OptoDAQ optoDAQ(descriptor, 1000);
	* optoDAQ.Open();
	* while (optoDAQ.IsValid()) {
	*   OptoPacket3D packet;
	*   optoDAQ.GetLastPacket3D(&packet, true);
	*   std::cout<<"Fx: "<<packet.GetFxAsCounts()<<std::endl;
	* }
	* std::cout<<"Sorry, connection lost."<<std::endl;
	* \endcode
	* @return true if \ref OptoDAQ is in valid state otherwise false
	*/
	bool							IsValid() const;


	void							SetDeviceIndex(int p_deviceIndex);
	int								GetDeviceIndex() const;
	int								GetDeviceCount() const;
	int								GetGripperDistance() const;
	int								GetGripperForce() const;
	long long						GetInternalIdentifier() const;
	bool							SendGripperParams(int p_width, int p_force);
protected:
	void *m_impl;
};

