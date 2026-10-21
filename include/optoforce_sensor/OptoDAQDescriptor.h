#pragma once
#include <string>
#include "OptoForceAPI.h"
/**
\class OptoDAQDescriptor
Descriptor object which represents the properties of a DAQ (like seial number, address, etc.)
**/
class OPTOFORCE_API OptoDAQDescriptor
{
public:
	/**
	* Default constructor
	*/
	OptoDAQDescriptor();
	/**
	* Copy constructor
	*/
	OptoDAQDescriptor(const OptoDAQDescriptor & p_other);
	/**
	* Constructor
	* @param p_address the address of DAQ which you want to access with the descriptor (i.e. "COM3")
	*/
	OptoDAQDescriptor(const char * p_address);
	/**
	* Destructor
	*/
	~OptoDAQDescriptor();
	/**
	* Access the address of the descriptor
	* @return the address of the descriptor as null-terminated string
	*/
	const char *		GetAddress() const;
	/**
	* Determines if a descriptor describes a USB DAQ
	* @return currently always returns true
	*/
	bool				IsUSB() const;

	/**
	* Sets the type name of the DAQ as a null-terminated string. Currently acceptable 
	* type-names are: "31", "34", "64"
	* @param p_typeName the type name
	*/
	void				SetTypeName(const char * p_typeName);
	/**
	* Sets the type name of the DAQ as an integer
	* @param p_typeName the type name
	*/
	void				SetTypeName(int p_typeName);
	/**
	* Sets the serial number of the DAQ as a null-terminated string
	* @param p_serialNumber the serial number of the DAQ
	*/
	void				SetSerialNumber(const char * p_serialNumber);
	/**
	* Sets the address of a DAQ as a null-terminated string (i.e. "COM3")
	*/
	void				SetAddress(const char * p_address);
	/**
	* Sets the opening parameter of a DAQ. Currently it is used to set up the
	* baudrate of a DAQ. Its default value is 1000000
	*/
	void				SetOpeningParameter(int p_parameter);

	/**
	* Sets the axis convention of a DAQ
	*/
	void				SetAxisConvention(int p_axisConvention);
	/**
	* Gets the type-name of the DAQ
	* @return the type name of the DAQ as a null-terminated string
	*/
	const char *		GetTypeName() const;
	/**
	* Gets the serial number of the DAQ
	* @return the serial number of the DAQ as a null-terminated string
	*/
	const char *		GetSerialNumber() const;
	/**
	* Retrieves the opening parameter of a DAQ (i.e. baudrate)
	*/
	int					GetOpeningParameter() const;
	/**
	* Determines if the DAQ is a 3-axis DAQ
	* @return true if the DAQ is a 3-axis DAQ otherwise it returns false
	*/
	bool				Is3D() const;
	/**
	* Determines if the DAQ is a 6-axis DAQ
	* @return true if the DAQ is a 6-axis DAQ otherwise it returns false
	*/
	bool				Is6D() const;
	/**
	*  Determines if the DAQ is in bootloader mode
	*  @return true if the DAQ is in bootloader mode otherwise it returns false
	*/
	bool				IsBootLoader() const;
	/**
	* Determines if the DAQ is using new axis convention mode
	* @return true if the DAQ is using new axis convention mode false otherwise
	*/
	bool				IsNewAxisConvention() const;
	/**
	* Determines if the current instance is valid
	* @return true if the current instance is valid (it has been initialized by an address and a valid type name)
	*/
	bool				IsValid() const;

	/**
	* Determines if an instance is smaller than an other by their address
	*/
	bool				operator < (const OptoDAQDescriptor & p_other) const;
    /**
	* Determines if two OptoDAQDescriptors are the same
	*/
	bool				operator == (const OptoDAQDescriptor & p_other) const;
	/**
	* Assignment operator
	*/
	OptoDAQDescriptor	&   operator = (const OptoDAQDescriptor & p_other);

	void				SetProtocolVersion(int p_protocolVersion);
	int					GetProtocolVersion() const;

private:
	char				m_address[256];
	char				m_typeName[64];
	char				m_serialNumber[128];
	int					m_protocolVersion;
	int					m_openingParameter;
	int					m_axisConvention;
};

