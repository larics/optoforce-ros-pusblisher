#pragma once
#include <cstddef>
#include "OptoForceAPI.h"
/**
* This class represents data received from DAQ in debug mode
*/
class OPTOFORCE_API OptoDebugPacket
{
public:
	/**
	* Constructor
	*/
	OptoDebugPacket();
	/**
	* Constructor
	* \n
	* This constructor is used by the API internally to construct debug packets
	* @param [in] p_s The raw signals (array, size must be 4)
	* @param [in] p_sc The compensated signals (array, size must be 4)
	* @param [in] p_temp The temperature signal
	*/
	OptoDebugPacket(const int * p_s, const int * p_sc, int p_temp);
	/**
	* Destructor
	*/
	virtual ~OptoDebugPacket();

	/**
	* Retrieves the requested raw signal
	* @param [in] p_index The index of the raw signal (value should be 0, 1, 2 or 3)
	* @return the requested raw signal's value
	*/
	int GetRawSignal(std::size_t p_index) const;
	/**
	* Retrieves the requested compensated signal
	* @param [in] p_index The index of the compensated signal (value should be 0, 1, 2 or 3)
	* @return the requested compensated signal's value
	*/
	int GetCompensatedSignal(std::size_t p_index) const; 
	/**
	* Retrieves the requested static raw signal. The value of these signals are never changed 
	* by the API even after zeroing.
	* @param [in] p_index The index of the static raw signal (value should be 0, 1, 2 or 3)
	* @return the requested static raw signal's value
	*/
	int GetRawSignalStatic(std::size_t p_index) const;
	/**
	* Retrieves the requested static compensated signal. The value of these are signals never changed
	* by the API even after zeroing.
	* @param [in] p_index The index of the static compensated signal (value should be 0, 1, 2 or 3)
	* @return the requested static compensated signal's value
	*/
	int GetCompensatedSignalStatic(std::size_t p_index) const;
	/**
	* Sets the  static raw signal's value at the given index
	* @param [in] p_index The index of the static raw signal (value should be 0, 1, 2 or 3)
	* @param [in] p_value The intended value of static raw signal
	*/
	void SetRawSignalStatic(std::size_t p_index, int p_value);
	/**
	* Sets the  static compensated signal's value at the given index
	* @param [in] p_index The index of the static compensated signal (value should be 0, 1, 2 or 3)
	* @param [in] p_value The intended value of static compensated signal
	*/
	void SetCompensatedSignalStatic(std::size_t p_index, int p_value);
	/**
	* Retrieves the value of \ref OptoDebugPacket's temperature signal
	* @return the value of the temperature signal
	*/
	int GetTemp() const;
	/**
	* Retrieves the value of special Fx value, which is calculated from raw signals of 6-axis DAQ
	* @return the value of Fx in dimensionless counts
	*/
	int			   GetFxAsCounts() const;
	/**
	* Retrieves the value of special Fy value, which is calculated from raw signals of 6-axis DAQ
	* @return the value of Fy in dimensionless counts
	*/
	int			   GetFyAsCounts() const;
	/**
	* Retrieves the value of special Fz value, which is calculated from raw signals of 6-axis DAQ
	* @return the value of Fz in dimensionless counts
	*/
	int			   GetFzAsCounts() const;
	/**
	* Retrieves the value of special Fx value, which is calculated from compensated signals of 6-axis DAQ
	* @return the value of Fx in dimensionless counts
	*/
	int			   GetCompensatedFxAsCounts() const;
	/**
	* Retrieves the value of special Fy value, which is calculated from compensated signals of 6-axis DAQ
	* @return the value of Fy in dimensionless counts
	*/
	int			   GetCompensatedFyAsCounts() const;
	/**
	* Retrieves the value of special Fz value, which is calculated from compensated signals of 6-axis DAQ
	* @return the value of Fz in dimensionless counts
	*/
	int			   GetCompensatedFzAsCounts() const;
	/**
	* Calculates the compensated and raw Fx, Fy, Fz values
	* @param [in] p_is6D flag that determines if the \ref OptoDebugPacket is a 6-axis debug packet. 
	* @param [in] p_compMode determines if it is a compensated packet
	*/
	void		   CalculateValues(bool p_is6D, bool p_compMode, int p_fx, int p_fy, int p_fz);

	/**
	* Determines if the \ref OptoDebugPacket is a valid packet. Invalid packets should never
	* be used
	* @return true if the packet is valid, otherwise false
	*/
	bool		   IsValid() const;
private:
	int		m_s[4];
	int		m_sc[4];
	int		m_sStatic[4];
	int		m_scStatic[4];
	int		m_temp;

	int		m_fxCounts;
	int		m_fyCounts;
	int		m_fzCounts;

	int		m_fxCCounts;
	int		m_fyCCounts;
	int		m_fzCCounts;

	bool	m_isValid;
};

