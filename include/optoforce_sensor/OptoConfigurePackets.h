#pragma once
#include <cstddef>
#include "OptoForceAPI.h"
#include "OptoConfigurePacket.h"
class OPTOFORCE_API OptoConfigurePackets
{
public:
	OptoConfigurePackets();
	virtual ~OptoConfigurePackets();
	bool						Load(const char * p_fileName);
	std::size_t					GetPacketCount() const;
	const OptoConfigurePacket & GetPacket(const char * p_packetName) const;
	const OptoConfigurePacket & operator [] (const char * p_packetName) const;
	const OptoConfigurePacket & GetPacket(std::size_t p_index) const;
	const OptoConfigurePacket & operator [] (std::size_t p_index) const;
private:
	void * m_impl;
};

