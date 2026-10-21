#pragma once
#include "OptoForceAPI.h"
#include "OptoPacket6D.h"

/**
* The class is a container of \ref OptoPacket6D objects.
* You can read 6D packets from the \ref OptoDAQ by using
* this container. You can read/write the container concurently (i.e. AddPacket/Clear/GetPacket/Resize
* can be called from different threads)
* Simple usage:
* \code
* OptoPackets6D packets(1000);  // The container can hold 1000 elements
* packets.Clear(); // Clear previous elements.
* daq.GetPackets6D(packets, false);  // false means we do not want a blocking call (OptoDAQ::GetPackets6D for more information)
* std::size_t readPacketsCount = packets.GetSize(); // The readPacketsCount now holds the overall number of read packets.
* OptoPacket6D packet = packets.GetPacket(0); // packet now holds the first packet.
* \endcode
*/
class OPTOFORCE_API OptoPackets6D
{
public:
	/**
	* Default constructor
	*/
	OptoPackets6D();
	/**
	* Constructor
	* @param [in] p_capacity The capacity of the container. This value sets the maximal number of
	* elements that OptoPackets6D can hold.
	*/
	OptoPackets6D(std::size_t p_capacity);
	/**
	* Copy constructor
	*/
	OptoPackets6D(const OptoPackets6D & p_other);
	/**
	* Assignment operator
	*/
	OptoPackets6D & operator = (const OptoPackets6D & p_other);
	/**
	* Destructor
	*/
	virtual ~OptoPackets6D();

	/**
	* Adds a packet to the container.
	* @param [in] p_packet the packet you want to add to the container.
	*/
	void					AddPacket(const OptoPacket6D & p_packet);
	/**
	* Retrieves a packet from a container.
	* @param [in] p_index The index of the packet you want to retrieve.
	* @return a valid packet if a valid packet stored at the given index
	* otherwise it returns an invalid packet. (See \ref OptoPacket6D and \ref OptoPacket6D::IsValid())
	*/
	const OptoPacket6D	&	GetPacket(std::size_t p_index) const;


	/**
	* Retrieves a packet from a container
	* @param [in] p_index The index of the packet you want to retrieve
	* @return a valid packet if a valid packet stored at the given index
	* otherwise it returns an invalid packet. (See \ref OptoPacket6D and \ref OptoPacket6D::IsValid())
	*/
	const OptoPacket6D & operator [] (std::size_t p_index) const;
	/**
	* Clears the stored elements. The size of the container reduces to 0
	* but capacity remains the same.
	*/
	void					Clear();
	/**
	* Resizes the container.
	* @param [in] p_newSize The intended new size of the container.
	*  - You can reduce the size of the container if you give smaller value than the current capacity
	*  - You can release the memory used by the container if you give 0 as the new size
	*  - The elements will not be erased but if the new capacity is smaller than the current size,
	*    elements that cannot be hold will be truncated from the right (i.e. from the bigger index)
	*  @return The new size of the container
	*/
	std::size_t				Resize(std::size_t p_newSize);

	/**
	* Retrieves the current capacity of the container
	* To calculate how many elemenst can be stored you can use the formula: GetCapacity() - GetSize()
	* @return The current capacity of the container
	*/
	std::size_t				GetCapacity() const;
	/**
	* Retrieves the current size of the container (the current number of elements the container holds)
	* @return The current size of the container.
	*/
	std::size_t				GetSize() const;

private:
	OptoPacket6D *	m_packets;
	std::size_t		m_capacity;
	std::size_t		m_size;
	std::size_t		m_sensorCount;
	OptoPacket6D	m_invalidPacket;
	mutable void  *	m_concurent;
};

