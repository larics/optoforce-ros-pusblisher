#pragma once
#include "OptoForceAPI.h"
/**
* This class represents a sensitivity for a given axis (Fx, Fy, etc)
*/
class OptoSensitivity
{
public:
	/**
	* Default construcotr
	*/
	OptoSensitivity();
	/**
	* Constructor
	* @param [in] p_nominalCapacity The intended nominal capacity
	* @param [in] p_counts The intended counts
	*/
	OptoSensitivity(unsigned int p_nominalCapacity, unsigned int p_counts);

	/**
	* Retrieves the nominal capacity
	* @return the nominal capacity
	*/
	unsigned int				GetNominalCapacity() const;
	/**
	* Retrieves the counts value
	* @return the counts value
	*/
	unsigned int				GetCounts() const;
	/**
	* Determines if the sensitivity is valid
	* @return true if the sensitivity is valid false otherwise
	*/
	bool						IsValid() const;

	virtual ~OptoSensitivity();
private:
	unsigned int				m_nominalCapacity;
	unsigned int				m_counts;
};

