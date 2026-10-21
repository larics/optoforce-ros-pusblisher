#pragma once
#include <cstddef>
#include "OptoForceAPI.h"
#include "OptoSensitivity.h"

/**
* This class represents a complete sensitivity report for a given sensor
*/
class OPTOFORCE_API OptoSensitivityReport
{
public:
	/**
	* Default construcot
	*/
	OptoSensitivityReport();
	/**
	* Copy constructor
	*/
	OptoSensitivityReport(const OptoSensitivityReport & p_other);
	/**
	* Constructor
	* @param [in] p_axisFx The sensitivity which is intented to be associated to Fx axis
	* @param [in] p_axisFy The sensitivity which is intended to be associated to Fy axis
	* @param [in] p_axisFz The sensitivity which is intended to be associated to Fz axis
	* @param [in] p_axisTx The sensitivity which is intended to be associated to Tx axis
	* @param [in] p_axisTy The sensitivity which is intended to be associated to Ty axis
	* @param [in] p_axisTz The sensitivity which is intended to be associated to Tz axis
	*/
	OptoSensitivityReport(const OptoSensitivity & p_axisFx, const OptoSensitivity & p_axisFy, const OptoSensitivity & p_axisFz,
						  const OptoSensitivity & p_axisTx, const OptoSensitivity & p_axisTy, const OptoSensitivity & p_axisTz);
	/**
	* Destructor
	*/
	virtual ~OptoSensitivityReport();
	/**
	* Loads the sensitivity report from file (OSR format)
	* @param [in] p_fileName A null-terminated string of the file name to be loaded
	* @return true if the operation succeded false otherwise
	*/
	bool	LoadFromFile(const char * p_fileName);
	/**
	* Determines if the sensitivity report is valid
	* @return true if the sensitivity report is valid false otherwise
	*/
	bool	IsValid() const;
	/**
	* Retrieves the sensitivity associated to Fx-axis
	* @return the sensitivity associated to Fx-axis
	*/
	const	OptoSensitivity &	GetFxSensitivity() const;
	/**
	* Retrieves the sensitivity associated to Fy-axis
	* @return the sensitivity associated to Fy-axis
	*/
	const	OptoSensitivity &	GetFySensitivity() const;
	/**
	* Retrieves the sensitivity associated to Fz-axis
	* @return the sensitivity associated to Fz-axis
	*/
	const	OptoSensitivity &	GetFzSensitivity() const;
	/**
	* Retrieves the sensitivity associated to Tx-axis
	* @return the sensitivity associated to Tx-axis
	*/
	const	OptoSensitivity &	GetTxSensitivity() const;
	/**
	* Retrieves the sensitivity associated to Ty-axis
	* @return the sensitivity associated to Ty-axis
	*/
	const	OptoSensitivity &	GetTySensitivity() const;
	/**
	* Retrieves the sensitivity associated to Tz-axis
	* @return the sensitivity associated to Tz-axis
	*/
	const	OptoSensitivity &	GetTzSensitivity() const;
	/**
	* Calculates Fx as Newton from counts
	* @param [in] p_counts the counts value which is intended to be converted to Newton
	* @return Fx as Newton
	*/
	double						CalcFx(int p_counts) const;
	/**
	* Calculates Fy as Newton from counts
	* @param [in] p_counts the counts value which is intended to be converted to Newton
	* @return Fy as Newton
	*/
	double						CalcFy(int p_counts) const;
	/**
	* Calculates Fz as Newton from counts
	* @param [in] p_counts the counts value which is intended to be converted to Newton
	* @return Fz as Newton
	*/
	double						CalcFz(int p_counts) const;
	/**
	* Calculates Tx as Newton meter from counts
	* @param [in] p_counts the counts value which is intended to be converted to Newton meter
	* @return Tx as Newton meter
	*/
	double						CalcTx(int p_counts) const;
	/**
	* Calculates Ty as Newton meter from counts
	* @param [in] p_counts the counts value which is intended to be converted to Newton meter
	* @return Ty as Newton meter
	*/
	double						CalcTy(int p_counts) const;
	/**
	* Calculates Tz as Newton meter from counts
	* @param [in] p_counts the counts value which is intended to be converted to Newton meter
	* @return Tz as Newton meter
	*/
	double						CalcTz(int p_counts) const;
	/**
	* Retrieves the number of axises of the \ref OptoSensitivityReport
	* @return 6 if the report is for a 6-axis DAQ, 3 if the report is for a 3-axis DAQ, every other values are considered invalid
	*/
	std::size_t					GetAxisCount() const;
	/**
	* Sets the number of axises of the \ref OptoSensitivityReport
	* @param [in] p_axisCount The number of axises (currenty only 3 and 6 are valid values)
	*/
	void						SetAxisCount(std::size_t p_axisCount);
	/**
	* Sets the \ref OptoSensitivity associated to Fx-axis
	* @param [in] p_sensitivity The sensitivity which is intended to be associated to Fx-axis
	*/
	void						SetFxSensitivity(const OptoSensitivity & p_sensitivity);
	/**
	* Sets the \ref OptoSensitivity associated to Fy-axis
	* @param [in] p_sensitivity The sensitivity which is intended to be associated to Fy-axis
	*/
	void						SetFySensitivity(const OptoSensitivity & p_sensitivity);
	/**
	* Sets the \ref OptoSensitivity associated to Fz-axis
	* @param [in] p_sensitivity The sensitivity which is intended to be associated to Fz-axis
	*/
	void						SetFzSensitivity(const OptoSensitivity & p_sensitivity);
	/**
	* Sets the \ref OptoSensitivity associated to Tx-axis
	* @param [in] p_sensitivity The sensitivity which is intended to be associated to Tx-axis
	*/
	void						SetTxSensitivity(const OptoSensitivity & p_sensitivity);
	/**
	* Sets the \ref OptoSensitivity associated to Ty-axis
	* @param [in] p_sensitivity The sensitivity which is intended to be associated to Ty-axis
	*/
	void						SetTySensitivity(const OptoSensitivity & p_sensitivity);
	/**
	* Sets the \ref OptoSensitivity associated to Tz-axis
	* @param [in] p_sensitivity The sensitivity which is intended to be associated to Tz-axis
	*/
	void						SetTzSensitivity(const OptoSensitivity & p_sensitivity);
	/**
	* Assignment operator
	*/
	OptoSensitivityReport & operator = (const OptoSensitivityReport & p_other);
private:
	void *  m_impl;
};

