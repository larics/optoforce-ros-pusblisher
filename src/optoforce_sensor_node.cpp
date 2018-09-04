#include <iostream>
#include "optoforce_sensor/OptoDAQ.h"
#include "optoforce_sensor/OptoDAQDescriptor.h"
#include "optoforce_sensor/OptoPacket6D.h"
#include "optoforce_sensor/OptoDAQWatcher.h"
#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include <unistd.h>
#include <string.h>
#include <sstream>

void msSleep(unsigned long p_uMillisecs)
{
	usleep(p_uMillisecs * 1000);
}


int main(int argc, char **argv)
{
    int number_of_sensors, speed, filter, rate;
    double fx_gain, fy_gain, fz_gain;  // sensitivity gain in counts/N
    double tx_gain, ty_gain, tz_gain;  // sensitivity gain in count/Nm

    ros::init(argc, argv, "OForceSensorPublisher");
    ros::NodeHandle n("~");


    // get sensitivy gains from rosparam server
    n.param("fx_gain", fx_gain, 500.0);
    n.param("fy_gain", fy_gain, 500.0);
    n.param("fz_gain", fz_gain, 200.28);

    n.param("tx_gain", tx_gain, 0.0);
    n.param("ty_gain", ty_gain, 0.0);
    n.param("tz_gain", tz_gain, 0.0);

    n.param("speed", speed, int(1000));
    n.param("filter", filter, int(0));
    n.param("sensors", number_of_sensors, int(1));

    //Create an OptoDAQWatcher instance that can enumerate connected DAQs via USB
    OptoDAQWatcher watcher; 
    watcher.Start();  // Start the watcher on a different thread 
    OptoDAQDescriptor *descriptors = new OptoDAQDescriptor[number_of_sensors];  
    
    //Trying to get connected DAQs (max.: 16, it can be changed up to 64)
    std::size_t count = watcher.GetConnectedDAQs(descriptors, number_of_sensors, true);
    while (count == 0) {
        count = watcher.GetConnectedDAQs(descriptors, number_of_sensors, true);
    }
    
    //Show information about connected DAQs
    for (std::size_t i = 0; i < count; ++i) {
        std::cout << "Information about Connected DAQ (" << i + 1 << "):" << std::endl;
        std::cout << "Connected on port: "<<descriptors[i].GetAddress()<<std::endl;
        std::cout << "Protocol version: " << descriptors[i].GetProtocolVersion() << std::endl;
        std::cout << "S/N:" << descriptors[i].GetSerialNumber() << std::endl;
        std::cout << "Type name:" << descriptors[i].GetTypeName() << std::endl;
        std::cout << "-----------------------" << std::endl;
    }

    // Open all the connected DAQs
    OptoDAQ  * daqs = new OptoDAQ[count];
    for (std::size_t i = 0; i < count; ++i) {
        daqs[i].SetOptoDAQDescriptor(descriptors[i]);
        bool success = daqs[i].Open();
        if (success == false) {
            std::cout << i + 1 << ". DAQ could not be opened!" << std::endl;
            continue;
        }
        OptoConfig config = OptoConfig(speed, filter, 0);
        success = daqs[i].SendConfig(config); // Set up the speed to 100 Hz and filtering to 15 Hz
        if (success) {
            std::cout << i + 1 << ". DAQ successfully configured." << std::endl;
        }
        else {
            std::cout << i + 1 << ". DAQ could not be configured." << std::endl;
            continue;
        }
        daqs[i].RequestSensitivityReport(); // This call is a must
    }

    ros::Publisher *wrench_pub = new ros::Publisher[count]; 

    for (int i = 0; i < count; i++) {
        
        std::ostringstream s;
        s << i;
        std::string topic;
        topic  = "OptoForceWrench_" + std::string(s.str());
        *wrench_pub = n.advertise<geometry_msgs::WrenchStamped>(topic, 0);
        daqs[i].ZeroAll();
    }

    OptoPackets3D packets3D(1);
    OptoPackets6D packets6D(1);
    geometry_msgs::WrenchStamped wrench_msg; // Create msg

    while(ros::ok()) {
        ros::spinOnce();

        for (std::size_t i = 0; i < count; ++i) {

            if (daqs[i].Is3D()) {
                if (daqs[i].IsValid()) daqs[i].GetPackets3D(&packets3D, true);
                std::size_t size = packets3D.GetSize();

                for (std::size_t j = 0; j < size; ++j) {
                    OptoPacket3D p = packets3D.GetPacket(j);
                    if (p.IsValid()) {
                        wrench_msg.header.stamp = ros::Time::now();
                        wrench_msg.wrench.force.x = p.GetFxInCounts(0) / fx_gain;;
                        wrench_msg.wrench.force.y = p.GetFyInCounts(0) / fy_gain;;
                        wrench_msg.wrench.force.z = p.GetFzInCounts(0) / fz_gain;;
                        wrench_msg.wrench.torque.x = 0.0;
                        wrench_msg.wrench.torque.y = 0.0;
                        wrench_msg.wrench.torque.z = 0.0;
                        wrench_pub[i].publish(wrench_msg);
                    }
                }
                packets3D.Clear(); // Empty the container for the next DAQ
            }
            else if (daqs[i].Is6D()) {
                if (daqs[i].IsValid()) daqs[i].GetPackets6D(&packets6D, true);
                std::size_t size = packets6D.GetSize();

                for (std::size_t j = 0; j < size; ++j) {
                    OptoPacket6D p = packets6D.GetPacket(j);
                    if (p.IsValid()) {
                        wrench_msg.header.stamp = ros::Time::now();
                        wrench_msg.wrench.force.x = p.GetFxInCounts() / fx_gain;
                        wrench_msg.wrench.force.y = p.GetFyInCounts() / fy_gain;
                        wrench_msg.wrench.force.z = p.GetFzInCounts() / fz_gain;
                        wrench_msg.wrench.torque.x = p.GetTxInCounts() / tx_gain;
                        wrench_msg.wrench.torque.y = p.GetTyInCounts() / ty_gain;
                        wrench_msg.wrench.torque.z = p.GetTzInCounts() / tz_gain;
                        wrench_pub[i].publish(wrench_msg);
                    }
                }
                packets6D.Clear(); // Empty the container for the next DAQ
            }
            //std::cout<< daqs[i].GetConfig().GetSpeed()<<std::endl;
            //std::cout<< daqs[i].GetConfig().GetFilter()<<std::endl;
        }
        //loop_rate.sleep();
    }

    for (std::size_t i = 0; i < count; ++i) daqs[i].Close();

    delete[] descriptors, daqs, wrench_pub;
    return 0;
}
