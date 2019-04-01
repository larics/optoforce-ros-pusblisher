#pragma once
#include <cstddef>
#include "OptoForceAPI.h"
#include "OptoDebugPacket.h"
#include "OptoSimplePacket3D.h"
#include "OptoSensitivityReport.h"
/**
* The class represents a packet of a 3-axis DAQ
*/
class OPTOFORCE_API OptoPacket3D
{
public:
	/**
	* Default constructor
	*/
	OptoPacket3D();
	/**
	* Constructor
	* This constructor is usually called by the internal API, so you can ignore it
	* @param [in] p_packetCounter the current counter of the packet given by the DAQ
	* @param [in] p_fX the Fx values of the packet in counts (size of the array must be 16)
	* @param [in] p_fY the Fy values of the packet in counts (size of the array must be 16)
	* @param [in] p_fZ the Fz values of the packet in counts (size of the array must be 16)
	* @param [in] p_status the status word of the current packet
	* @param [in] p_debugPackets the debug packets of the current packet (size of the array must be 16)
	* @param [in] p_size the size of the current packet (one-channel packet or multi-channel packet)
	*/
	OptoPacket3D(unsigned short p_packetCounter, const int * p_fX, const int * p_fY,  const int * p_fZ, unsigned short p_status, const OptoDebugPacket * p_debugPackets, std::size_t p_size = 1);
	/**
	* Copy constructor
	*/
	OptoPacket3D(const OptoPacket3D & p_Other);
	~OptoPacket3D();
public:
	/**
	* Returns the sample (packet) counter of the current packet as given by the DAQ
	* @return the sample (packet) counter of the current packet as given by the DAQ
	*/
	unsigned short			GetPacketCounter() const;
	/**
	* Returns the Fx force in a dimensionless value
	* @return the Fx force in a dimensionless value
	*/
	int						GetFxInCounts(std::size_t p_index) const;
	/**
	* Returns the Fy force in a dimensionless value
	* @return the Fy force in a dimensionless value
	*/
	int						GetFyInCounts(std::size_t p_index) const;
	/**
	* Returns the Fz force in a dimensionless value
	* @return the Fz force in a dimensionless value
	*/
	int						GetFzInCounts(std::size_t p_index) const;
	/**
	* Returns the Fx force in Newton
	* @return the Fx force in Newton
	*/
	double					GetFxInNewton(std::size_t p_index) const;
	/**
	* Returns the Fy force in Newton (currently always returns with 0.0)
	* @return the Fy force in Newton (currently always returns with 0.0)
	*/
	double					GetFyInNewton(std::size_t p_index) const;
	/**
	* Returns the Fz force in Newton (currently always returns with 0.0)
	* @return the Fz force in Newton (currently always returns with 0.0)
	*/
	double					GetFzInNewton(std::size_t p_index) const;
	/**
	* This function applies sensitivity report on the current packet. This function used
	* by the internal parts of the API
	* @param [in] p_report the sensitivity report
	*/
	void					ApplySensitivityReport(const OptoSensitivityReport & p_report);
	/**
	* Returns the status word of the current packet
	* @return the status word of the current packet
	*/
	unsigned short			GetStatus() const;
	/**
	* Returns the requested \ref OptoDebugPacket of the current packet
	* @param [in] p_index the index of the requested \ref OptoDebugPacket (in the case of a single-channel 
	* packet it should be 0, in the case of a multi-channel packet it should be 0, 1,2 or 3, see \ref GetSize())
	*/
	const OptoDebugPacket &	GetDebugPacket(std::size_t p_index) const;
	/**
	* Sets the time stamp of the packet.
	* @param [in] p_timeStamp the timestamp in microseconds
	*/
	void					SetTimeStamp(long long p_timeStamp);
	/**
	* Returns of the packet's timestamp
	* @return timestamp of the packet (the timestamp is generated when a packet is received from the DAQ)
	*/
	long long				GetTimeStamp() const;
	/**
	* Generates a more simple structure from the packet (See \ref OptoSimplePacket3D)
	* @return the more simple representation of the packet as a \ref OptoSimplePacket3D
	*/
	OptoSimplePacket3D		ToSimplePacket() const;
	/**
	* Returns the size of the packet (in the case of one-channel DAQ it returns 1 otherwise it return 4)
	* @return the size of the packet
	*/
	std::size_t				GetSize() const;
	/**
	* Determines if the packet is valid.
	* @return true if the packet is valid otherwise false. You should not use a packet
	* which is invalid.
	*/
	bool					IsValid() const;


private:
	OptoDebugPacket		m_debugPacket[16];
	OptoDebugPacket		m_invalidPacket;
	unsigned short		m_packetCounter;
	int					m_countsFx[16];
	int					m_countsFy[16];
	int					m_countsFz[16];
	double				m_fX[16];
	double				m_fY[16];
	double				m_fZ[16];
	unsigned short		m_status;
	bool				m_isValid;
	std::size_t			m_size;
	long long			m_timeStamp;
};

