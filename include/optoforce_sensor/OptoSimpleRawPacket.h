#pragma once
#include <cstddef>
#include "OptoForceAPI.h"

class OPTOFORCE_API OptoSimpleRawPacket
{
public:
	OptoSimpleRawPacket();
	OptoSimpleRawPacket(const OptoSimpleRawPacket & p_other);
	OptoSimpleRawPacket & operator = (const OptoSimpleRawPacket & p_other);
	virtual ~OptoSimpleRawPacket();

	void					SetAddress(unsigned int p_address);
	void					SetPayload(const unsigned char * p_payload, std::size_t p_size);
	void					SetReadWrite(unsigned char p_readWrite);
	const unsigned char *	GetPayload() const;
	std::size_t				GetSize() const;
	unsigned char			GetReadWrite() const;
	unsigned int			GetAddress() const;
	bool					IsValid() const;

	unsigned short			ToUnsignedShort(std::size_t p_offset) const;
	short					ToSignedShort(std::size_t p_offset) const;
private:
	unsigned int	m_address;
	std::size_t		m_size;
	unsigned char	m_payload[512];
	unsigned char   m_readWrite;
};

