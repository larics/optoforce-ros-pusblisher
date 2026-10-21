#pragma once
#include "OptoForceAPI.h"
/**
* This structure represents a 3-axis packet in a more
* simple form than a \ref OptoPacket3D object
*/

struct OPTOFORCE_API OptoSimplePacket3D
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
	* Array of Fx values in a dimensionless format
	*/
	int CountsFx[16];
	/**
	* Array of Fy values in a dimensionless format
	*/
	int CountsFy[16];
	/**
	* Array of Fz values in a dimensionless format
	*/
	int CountsFz[16];
	/**
	* Array of Fx values in Newton
	*/
	double Fx[16];
	/**
	* Array of Fy values in Newton
	*/
	double Fy[16];
	/**
	* Array of Fz values in Newton
	*/
	double Fz[16];
	/**
	* Determines if the packet is valid (See \ref OptoPacket3D::IsValid())
	* Value greater than zero means that the packet is valid
	*/
	unsigned char IsValid;
};