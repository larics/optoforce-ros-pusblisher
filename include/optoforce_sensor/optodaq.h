#ifndef OMD__OPTODAQ_H
#define OMD__OPTODAQ_H 
#include "optoforce_sensor/optopackage.h"
#include "optoforce_sensor/optopackage6d.h"
#include "optoforce_sensor/sensorconfig.h"
#include "optoforce_sensor/optoports.h"
class ReaderThread;
class OptoDAQ_private;
class OptoDAQ{
private:
    OptoDAQ_private *d_ptr;
public:
    OptoDAQ ();
    ~OptoDAQ ();
    SensorConfig getConfig();
    bool unzero(int number=0);
    void unzeroAll();
    bool zero(int number=0);
    void zeroAll();
    int getSize();
    bool isChksumOK();
    opto_version getVersion();
    OptoPackage getOffset(int sensor);
    int getBytesPerRead();
    bool open (OPort port, bool modeSetup=false, int baudRate=1000000);
    void close ();
    char* getPortName();
    bool isOpen ();
    bool isVirtual();
    int getSensorSize();
    int read(OptoPackage& package, int sensor, bool peek=false);
    int readArray(int (&arr)[8], bool peek=false);
    int readVirtualArray(int (&virtArray)[128], bool peek=false);
    int readVirtual(OptoPackage (&packArray)[16], bool peek=false);
    int readAll(OptoPackage *&buffer, bool peek=false);
    int read6D(OptoPackage6D& package, bool peek=false);
    int readAll6D(OptoPackage6D *&buffer, bool peek=false);
    int read6Axis(int (&axis6)[6], bool peek=false);
    int readAll6Axis(int *&axis6Array, bool peek=false);
    bool sendConfig (SensorConfig c);
};
#endif
