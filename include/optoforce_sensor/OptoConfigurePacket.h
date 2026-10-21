#pragma once
#include <cstddef>
#include <vector>
#include <map>
#include "OptoForceAPI.h"
#include "OptoConfigureParam.h"
#include "OptoSimpleRawPacket.h"
class OPTOFORCE_API OptoConfigurePacket
{
public:
	OptoConfigurePacket();
	OptoConfigurePacket(const OptoConfigurePacket & p_other);
	virtual ~OptoConfigurePacket();
public:
	void						SetAddress(unsigned int p_address);
	void						SetLength(std::size_t p_length);
	void						SetName(const char * p_name);
	void						SetHeader(const char * p_header);
	unsigned int				GetAddress() const;
	std::size_t					GetLength() const;
	const char *				GetName() const;
	const char *				GetHeader() const;
public:
	bool						AddParam(const OptoConfigureParam & p_param);
	OptoConfigureParam &		GetParam(const char * p_name);
	OptoConfigureParam &		operator [] (const char * p_name);
	OptoConfigureParam &		GetParam(std::size_t p_index);
	OptoConfigureParam &		operator [](std::size_t p_index);
	std::size_t					GetParamCount() const;
	bool						IsValid() const;
	bool operator				< (const OptoConfigurePacket & p_other) const;
	bool operator				== (const OptoConfigurePacket & p_other) const;
	OptoConfigurePacket	& operator = (const OptoConfigurePacket & p_other);
	bool						ParsePacket(const OptoSimpleRawPacket & p_rawPacket);
	std::size_t					ToBinary(unsigned char * p_result, std::size_t p_maxLength) const;
private:
	void * m_impl;
};

