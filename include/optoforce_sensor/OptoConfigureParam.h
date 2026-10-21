#pragma once
#include <cstddef>
#include "OptoForceAPI.h"
class OPTOFORCE_API OptoConfigureParam
{
public:
	enum ParamTypes {
		ptNone,
		ptByte,
		ptUbyte,
		ptShort,
		ptUshort,
		ptString
	};
	OptoConfigureParam();
	virtual ~OptoConfigureParam();
	void			SetName(const char * p_name);
	void			SetHeader(const char * p_header);
	void			SetSequence(unsigned int p_sequence);
	void			SetType(const char * p_type);
	void			SetValueNumber(int p_value);
	void			SetValueString(const char * p_value);

	const char *	GetName() const;
	const char *	GetHeader() const;
	unsigned int	GetSequence() const;
	ParamTypes 		GetType() const;
	std::size_t		GetLength() const;
	const char *	GetValueString() const;
	int				GetValueNumber() const;

	bool			operator < (const OptoConfigureParam & p_other) const;
	bool			operator == (const OptoConfigureParam & p_other) const;

	bool			IsValid() const;
private:
	char			m_name[256];
	char			m_header[256];
	unsigned int	m_sequence;
	char			m_type[256];
	int				m_numberValue;
	char			m_stringValue[128];
	std::size_t		m_stringLength;
	std::size_t		m_length;
	ParamTypes		m_paramType;
};

