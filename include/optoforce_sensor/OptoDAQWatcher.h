#pragma once
#include <cstddef>
#include "OptoForceAPI.h"
#include "OptoDAQDescriptor.h"


/**
* This class is a helper class to enumerate DAQs which are connected
* to the computer by USB. The given functions return with 
* \ref OptoDAQDescriptor instances which can be used to operate with
* \ref OptoDAQ objects.\n
* Simple usage:
* \code
* OptoDAQ daq;
* OptoDAQWatcher daqWatcher; 
* daqWatcher.Start(); //Starts the operation of the instance on a separate thread
* while (true) {
*	OptoDAQDescriptor descriptor;
*   if (GetFirstDAQ(&descriptor) == true) {  // The GetFirstDAQ retrieves the first DAQ which found by daqWatcher
*		std::cout<<descriptor.GetAddress()<<std::endl; 
*		std::cout<<descriptor.GetSerialNumber()<<std::endl;
*		std::cout<<descriptor.GetTypeName()<<std::endl;
*		daq.SetDescriptor(descriptor);
*		if (daq.Open() == true) {
*			std::cout<<"DAQ is now opened!"<<std::endl;
*		}
*		break;
*   }
* }
* if (daq.IsValid()) {
*	// Here you can use the functions of OptoDAQ (i.e. read packets, set config, etc.)
* }
* \endcode
*/
class OPTOFORCE_API OptoDAQWatcher
{
public:
	/**
	* Default constructor
	*/
					OptoDAQWatcher();
	/**
	* Destructor
	*/
	virtual			~OptoDAQWatcher();
	/*
	* In most cases this function has not to be used. If you have a special
	* DAQ which baud rate is not the default, you must set the baudrate
	* using this function
	*/
	void			SetBaudRate(int p_baudRate);
	/**
	* Determines if there are any DAQs connected to the computer by USB.
	* @return true if there are any DAQs connected to the computer by USB.
	*/
	bool			HasConnectedDAQ() const;
	/**
	* Returns the list of connected DAQs.
	* \n
	* Simple usage:
	* \code
	* const std::size_t maximalCount = 8;
	* OptoDAQDescriptor descriptors[maximalCount];
	* OptoDAQWatcher daqWatcher;
	* daqWatcher.Start();
	* std::size_t connectedCount = daqWatcher.GetConnectedDAQs(descriptors, maximalCount);
	* for (std::size_t i = 0; i < connectedCount; ++i) {
	*	std::cout<<"Freshly connected DAQ's address is: "<<descriptors[i].GetAddress()<<std::endl;
	* }
	* \endcode
	* @param [out] p_descriptors Array of OptoDAQDescriptors which will hold the list of connected DAQs
	* @param [in] p_maxCount the maximal count of elements which p_descriptors can hold.
	* @param [in] p_showOnlyNewest If this flag is set to true, the function only returns the most up-to-date
	* list of descriptors: if no new DAQ is connected after the previous call of the function then no new item will
	* be retrieved.
	* \n
	* Sample:
	* \code
	* std::size_t count = 0;
	* count = daqWatcher.GetConnectedDAQs(descriptors, maximalCount, true); // Value of count will be the number of connected DAQs
	* count = daqWatcher.GetConnectedDAQs(descriptors, maximalCount, true); // Value of count will be 0 if no new DAQ was connected.
	* \endcode
	* @return the size of the list of connected DAQs
	*/
	std::size_t		GetConnectedDAQs(OptoDAQDescriptor * p_descriptors, std::size_t p_maxCount, bool p_showOnlyNewest = true);
	/**
	* Retrieves the first DAQ found by the OptoDAQWatcher instance.
	* This is the most simple way of getting a descriptor if you have only one DAQ connected to the computer.
	* @param [in] p_descriptor the descriptor that will hold the data of connected DAQ if any found.
	* @return true if there is a connected DAQ otherwise false 
	*/
	bool			GetFirstDAQ(OptoDAQDescriptor * p_descriptor);
	/**
	* Determines if there are any disconnected DAQs
	* @return true if there are any DAQ that has been disconnected. 
	*/
	bool			HasDisconnectedDAQ() const;
	/**
	* Returns the list of disconnected DAQs.
	* \n
	* Simple usage:
	* \code
	* const std::size_t maximalCount = 8;
	* OptoDAQDescriptor descriptors[maximalCount];
	* OptoDAQWatcher daqWatcher;
	* daqWatcher.Start();
	* std::size_t connectedCount = daqWatcher.GetDisconnectedDAQs(descriptors, maximalCount);
	* for (std::size_t i = 0; i < connectedCount; ++i) {
	*	std::cout<<"Freshly disconnected DAQ's address is: "<<descriptors[i].GetAddress()<<std::endl;
	* }
	* \endcode
	* @param [out] p_descriptors Array of OptoDAQDescriptors which will hold the list of disconnected DAQs
	* @param [in] p_maxCount the maximal count of elements which p_descriptors can hold.
	* @param [in] p_showOnlyNewest If this flag is set to true, the function only returns the most up-to-date
	* list of descriptors: if no new DAQ is disconnected after the previous call of the function then no new item will
	* be retrieved.
	* \n
	* Sample:
	* \code
	* std::size_t count = 0;
	* count = daqWatcher.GetDisconnectedDAQs(descriptors, maximalCount, true); // Value of count will be the number of disconnected DAQs
	* count = daqWatcher.GetDisconnectedDAQs(descriptors, maximalCount, true); // Value of count will be 0 if no new DAQ was disconnected.
	* \endcode
	* @return the size of the list of disconnected DAQs
	*/
	std::size_t		GetDisconnectedDAQs(OptoDAQDescriptor * p_descriptors, std::size_t p_maxCount, bool p_showOnlyNewest = true);
	/**
	* Adds an address that should be not accessed by the OptoDAQWatcher instance. 
	* This function can be used if you have other USB connected devices which must not be accessed
	* by other applications. The unaccessable ports should be added before call of \ref OptoDAQWatcher::Start()
	* @param [in] p_address a null-terminated string of the address of the device. (i.e. "COM3")
	*/
	void			AddAddressToFilter(const char * p_address);
	/**
	* Starts the instance's operation on a different thread. If you have limited CPU resources starting of
	* the instance is not neccessary but in this case other function calls of OptoDAQWatcher will be blocking.
	*/
	void			Start();
	/**
	* Stops the instance's operation. 
	*/
	void			Stop();
private:
	void * m_impl;
};

