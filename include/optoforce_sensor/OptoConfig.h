#pragma once
#include "OptoForceAPI.h"
/**
* This class is can be used to setup the DAQ's
* -speed
* -filter
* -zeroing
*/
class OPTOFORCE_API OptoConfig
{
public:
	/**
	* Default constructor
	*/
	OptoConfig();
	/**
	* Constructor
	* @param [in] p_speed The intended speed of the DAQ
	* @param [in] p_filter The indended filter of the DAQ
	* - Values can be: (0 - no filtering; 1 - 500 Hz, 2 - 150 Hz, 3 - 50 Hz, 4 - 15 Hz (default), 5 - 5 Hz, 6 - 1.5 Hz)
	* @param [in] p_zeroing Turn on (255) or off (0) hardware zeroing
	* @param [in] p_mode The mode of the DAQ (cannot be set, it is maintained by the API)
	*/
	OptoConfig(int p_speed, int p_filter, unsigned char p_zeroing, unsigned char p_mode = 255);
	/**
	* Destructor
	*/
	virtual ~OptoConfig();
public:
	/**
	* Sets filter
	* @param [in] p_filter The indended filter of the DAQ
	* - Values can be: (0 - no filtering; 1 - 500 Hz, 2 - 150 Hz, 3 - 50 Hz, 4 - 15 Hz (default), 5 - 5 Hz, 6 - 1.5 Hz)
	*/
	void			SetFilter(int p_filter);
	/**
	* Sets speed
	* @param [in] p_speed The speed in Hz. Maximal value is 1000 minimal is 0. Not every speed values are valid,
	* to check out what will be the real speed, please see \ref OptoConfig::GetSpeed()
	*/
	void			SetSpeed(int p_speed);
	/**
	* Sets hardware zeroing
	* @param [in] p_zeroing Turn on (255) or off (0) hardware zeroing
	* \note
	* If you want to apply hardware zeroing more than one times during your
	* program's run, first you have send configuration with hardware zeroing
	* set to 0 then send another configuration with hardware zeroing set to
	* 255
	* \n
	* Example:
	* \code
	* OptoConfig optoConfig;
	* optoConfig.SetSpeed(100);
	* optoConfig.SetFilter(4);
	* optoConfig.SetZeroing(0);
	* daq.SendConfig(optoConfig);
	* optoConfig.SetZeroing(255);
	* daq.SendConfig(optoConfig);
	* daq.ClearPackets();  // To eliminate previous packets which could be not zeroed.
	* \endcode
	*/
	void			SetZeroing(unsigned char p_zeroing);
	/**
	* Retrieves the filter of DAQ
	* @return the filter of DAQ
	*/
	int				GetFilter() const;
	/**
	* Retrieves the filter of DAQ in Hz
	* @return the filter of DAQ in Hz
	*/
	double			GetFilterInHz();
	/**
	* Retrieves the speed if DAQ in Hz
	* @return the filter of DAQ in Hz
	*
	* \note
	* After setting the speed with \ref OptoConfig::SetSpeed you may check the real speed by 
	* call \ref OptoConfig::GetSpeed()
	* \n
	* Example:
	* \code
	* // Case 1
	* OptoConfig optoConfig;
	* optoConfig.SetSpeed(1000);
	* int speed = optoConfig.GetSpeed(); // Value of speed will be 1000
	* // Case 2
	* optoConfig.SetSpeed(667);
	* int speed = optoConfig.GetSpeed(); // Value of speed will be 1000 too
	* // Case 3
	* optoConfig.SetSpeed(500);
	* int speed = optoConfig.GetSpeed(); // Value of speed will be 500
	* \endcode
	* Explanation: the speed is sent on 1 byte. When the DAQ is configured to 1000 Hz, the value
	* of this byte is 1. The slowest speed is ~4 Hz in this case the value of this byte is 255.
	* So when the value of this byte is n the real speed is m {m, n}
	* {1, 1000; 2, 500; 3, 333; 4, 250; 5, 200; 6, ~166;...;10, 100; 20, 50; 100, 10;...;255, ~4} 
	*/
	int				GetSpeed() const;
	/**
	* Retrieves hardware zeroing of DAQ
	* @return 0 if hardware zeroing is off 255 if hardware zeroing is on
	*/
	unsigned char	GetZeroing() const;
	/**
	* Retrieves mode of DAQ
	* @return > 0 if DAQ in compensated mode otherwise 0
	*/
	unsigned char	GetMode() const;
	/**
	* Determines if \ref OptoConfig instance is valid
	* @return true if \ref OptoConfig instance is valid false otherwise
	*/
	bool			IsValid() const;
private:
	int				m_filter;
	int				m_speed;
	unsigned char	m_zeroing;
	unsigned char	m_mode;
};

