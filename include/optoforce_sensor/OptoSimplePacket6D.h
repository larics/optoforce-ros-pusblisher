#pragma once
#include "OptoForceAPI.h"

/**
* This structure represents a 6-axis packet in a more
* simple form than a \ref OptoPacket6D object
*/

struct OPTOFORCE_API OptoSimplePacket6D
{
	/**
	* The sample counter of a packet
	*/
	int SampleCounter;
	/**
	* The timestamp of the packet
	*/
	long long TimeStamp;
	/**
	* Value of Fx in a dimensionless format
	*/
	int CountsFx;
	/**
	* Value of Fy in a dimensionless format
	*/
	int CountsFy;
	/**
	* Value of Fz in a dimensionless format
	*/
	int CountsFz;
	/**
	* Value of Tx in a dimensionless format
	*/
	int CountsTx;
	/**
	* Value of Ty in a dimensionless format
	*/
	int CountsTy;
	/**
	* Value of Tz in a dimensionless format
	*/
	int CountsTz;
	/**
	* Value of Fx in Newton
	*/
	double Fx;
	/**
	* Value of Fy in Newton
	*/
	double Fy;
	/**
	* Value of Fz in Newton
	*/
	double Fz;
	/**
	* Value of Tx in Newton meter
	*/
	double Tx;
	/**
	* Value of Ty in Newton meter
	*/
	double Ty;
	/**
	* Value of Tz in Newton meter
	*/
	double Tz;
	/**
	* Determines if the packet is valid (See \ref OptoPacket6D::IsValid())
	* Value greater than zero means that the packet is valid
	*/
	unsigned char IsValid;
};
