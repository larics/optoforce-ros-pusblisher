#pragma once
#include <cstddef>
#include "OptoForceAPI.h"
#include "OptoDAQ.h"
#include "OptoDAQDescriptor.h"
#include "OptoSimpleRawPacket.h"
#include "OptoConfigurePacket.h"
class OPTOFORCE_API OptoDAQConfigure : public OptoDAQ
{
public:
	OptoDAQConfigure();
	OptoDAQConfigure(const OptoDAQDescriptor & p_descriptor, std::size_t p_maximalPacketCount = 10000);
	virtual ~OptoDAQConfigure();
public:
	OptoSimpleRawPacket RequestPacket(unsigned int p_fromAddress, std::size_t p_length);
	OptoSimpleRawPacket GetLastBootLoaderPacket();
	bool				SendPacket(unsigned int p_toAddress, const unsigned char * p_data, std::size_t p_length);
	bool				ReadPacket(unsigned int p_fromAddress, std::size_t p_length);
	bool				SendPacket(const OptoConfigurePacket & p_packet);
	OptoConfigurePacket RequestPacket(const OptoConfigurePacket & p_packet);
};

