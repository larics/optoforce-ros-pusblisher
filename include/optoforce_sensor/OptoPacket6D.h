#pragma once
#include <cstddef>
#include "OptoForceAPI.h"
#include "OptoDebugPacket.h"
#include "OptoSimplePacket6D.h"
#include "OptoSensitivityReport.h"

/**
* The class represents a packet of a 6-axis DAQ
*/
class OPTOFORCE_API OptoPacket6D
{
public:
	/**
	* Default constructor
	*/
	OptoPacket6D();
	/**
	* Constructor
	* This constructor is usually called by the internal API, so you can ignore it
	* @param [in] p_packetCounter the current counter of the packet given by the DAQ
	* @param [in] p_fX the Fx value of the packet in counts
	* @param [in] p_fY the Fy value of the packet in counts
	* @param [in] p_fZ the Fz value of the packet in counts
	* @param [in] p_tX the Tx value of the packet in counts
	* @param [in] p_tY the Ty value of the packet in counts
	* @param [in] p_tZ the Tz value of the packet in counts
	* @param [in] p_status the status word of the current packet
	* @param [in] p_debugPacket the debug packets of the current packet (size of the array must be 4)
	*/
	OptoPacket6D(unsigned short p_packetCounter, int p_fX, int p_fY, int p_fZ, int p_tX, int p_tY, int p_tZ, unsigned short p_status, const OptoDebugPacket * p_debugPacket,
				 int p_gripperDistance = -1, int p_gripperForce = -1, int p_distance = -1, bool p_distanceValid = false);
	/**
	* Copy constructor
	*/
	OptoPacket6D(const OptoPacket6D & p_other);
	/**
	* Destructor
	*/
	~OptoPacket6D();

	/**
	* Returns the sample (packet) counter of the current packet as given by the DAQ
	* @return the sample (packet) counter of the current packet as given by the DAQ
	*/
	unsigned short				GetPacketCounter() const;

	/**
	* Returns the Fx force in a dimensionless value
	* @return the Fx force in a dimensionless value
	*/
	int							GetFxInCounts() const;
	/**
	* Returns the Fy force in a dimensionless value
	* @return the Fy force in a dimensionless value
	*/
	int							GetFyInCounts() const;
	/**
	* Returns the Fz force in a dimensionless value
	* @return the Fz force in a dimensionless value
	*/
	int							GetFzInCounts() const;
	/**
	* Returns the Tx torque in a dimensionless value
	* @return the Tx torque in a dimensionless value
	*/
	int							GetTxInCounts() const;
	/**
	* Returns the Ty torque in a dimensionless value
	* @return the Ty torque in a dimensionless value
	*/
	int							GetTyInCounts() const;
	/**
	* Returns the Tz torque in a dimensionless value
	* @return the Tz torque in a dimensionless value
	*/
	int							GetTzInCounts() const;
	/**
	* Returns the Fx force in Newton
	* @return the Fx force in Newton. If no valid sensitivity report could be obtained from DAQ
	* or loaded from file, the function returns with 1234567.0
	*/
	double						GetFxInNewton() const;
	/**
	* Returns the Fy force in Newton
	* @return the Fy force in Newton. If no valid sensitivity report could be obtained from DAQ
	* or loaded from file, the function returns with 1234567.0
	*/
	double						GetFyInNewton() const;
	/**
	* Returns the Fz force in Newton
	* @return the Fz force in Newton. If no valid sensitivity report could be obtained from DAQ
	* or loaded from file, the function returns with 1234567.0
	*/
	double						GetFzInNewton() const;
	/**
	* Returns the Tx torque in Newton meter
	* @return the Tx torque in Newton meter. If no valid sensitivity report could be obtained from DAQ
	* or loaded from file, the function returns with 1234567.0 
	*/
	double						GetTxInNewtonMeter() const;
	/**
	* Returns the Ty torque in Newton meter
	* @return the Ty torque in Newton meter. If no valid sensitivity report could be obtained from DAQ
	* or loaded from file, the function returns with 1234567.0
	*/
	double						GetTyInNewtonMeter() const;
	/**
	* Returns the Tz torque in Newton meter
	* @return the Tz torque in Newton meter. If no valid sensitivity report could be obtained from DAQ
	* or loaded from file, the function returns with 1234567.0
	*/
	double						GetTzInNewtonMeter() const;
	/**
	* This function applies sensitivity report on the current packet. This function used
	* by the internal parts of the API
	* @param [in] p_report the sensitivity report
	*/
	void						ApplySensitivityReport(const OptoSensitivityReport & p_report);
	/**
	* Returns the status word of the current packet
	* @return the status word of the current packet
	*/
	unsigned short				GetStatus() const;
	/**
	* Returns the requested \ref OptoDebugPacket of the current packet
	* @param [in] p_index the index of the requested \ref OptoDebugPacket.\ref OptoPacket6D has 4 
	* OptoDebugPackets, p_index value should be 0, 1, 2 or 3.
	*/
	const OptoDebugPacket	  &	GetDebugPacket(std::size_t p_index) const;
	/**
	* Sets the time stamp of the packet.
	* @param [in] p_timeStamp the timestamp in microseconds
	*/
	void						SetTimeStamp(long long p_timeStamp);
	/**
	* Returns of the packet's timestamp
	* @return timestamp of the packet (the timestamp is generated when a packet is received from the DAQ)
	*/
	long long					GetTimeStamp() const;
	/**
	* Generates a more simple structure from the packet (See \ref OptoSimplePacket6D)
	* @return the more simple representation of the packet as a \ref OptoSimplePacket3D
	*/
	OptoSimplePacket6D			ToSimplePacket() const;
	/**
	* Determines if the packet is valid.
	* @return true if the packet is valid otherwise false. You should not use a packet
	* which is invalid.
	*/
	bool						IsValid() const;


	int							GetDistance() const;
	int							GetGripperDistance() const;
	int							GetGripperForce() const;
	bool						IsDistanceValid() const;

	

	void						SetQuaternion(double p_w, double p_x, double p_y, double p_z);
	void						SetGravity(double p_x, double p_y, double p_z);

	double						GetQw() const;
	double						GetQx() const;
	double						GetQy() const;
	double						GetQz() const;

	double							GetGx() const;
	double							GetGy() const;
	double							GetGz() const;


private:
	unsigned short m_packetCounter;
	unsigned short m_status;
	int m_countsFx;
	int m_countsFy;
	int m_countsFz;
	int m_countsTx;
	int m_countsTy;
	int m_countsTz;

	double m_fX;
	double m_fY;
	double m_fZ;
	double m_tX;
	double m_tY;
	double m_tZ;

	double m_qw;
	double m_qx;
	double m_qy;
	double m_qz;

	double    m_gx;
	double    m_gy;
	double	   m_gz;

	int		m_distance;
	int		m_gripperDistance;
	int		m_gripperForce;
	bool	m_distanceValid;

	bool m_isValid;
	unsigned  long long	m_timeStamp;
	OptoDebugPacket		m_debugPacket[4];
	OptoDebugPacket		m_invalidPacket;
};

