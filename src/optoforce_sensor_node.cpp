#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include <unistd.h>
#include <string.h>
#include <time.h>

#include "optoforce_sensor/OptoDAQ.h"
#include "optoforce_sensor/OptoDAQDescriptor.h"
#include "optoforce_sensor/OptoPacket6D.h"
#include "optoforce_sensor/OptoDAQWatcher.h"

void msSleep(unsigned long p_uMillisecs)
{
    usleep(p_uMillisecs * 1000);
}


int main(int argc, char **argv)
{
        
    ros::init(argc, argv, "ONROBOT_senzor");

    ros::NodeHandle n;
    ros::NodeHandle private_node_handle_("~");

    //Creating wrench publisher
    ros::Publisher wrench_pub_raw = n.advertise<geometry_msgs::WrenchStamped>("/optoforce_node/OptoForceWrench_raw", 1);
    ros::Publisher wrench_pub = n.advertise<geometry_msgs::WrenchStamped>("/optoforce_node/OptoForceWrench", 1);

    /*
    Create an OptoDAQWatcher instance that can enumerate connected DAQs via USB
    */
    OptoDAQWatcher watcher;
    watcher.Start();  // Start the watcher on a different thread


    OptoDAQDescriptor descriptors[16];

    /*
    Trying to get connected DAQs (max.: 16, it can be changed up to 64)
    */
    std::size_t count = watcher.GetConnectedDAQs(descriptors, 16, true);
    while (count == 0) {
        count = watcher.GetConnectedDAQs(descriptors, 16, true);
    }


    /*
    Show information about connected DAQs
    */
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
  
        OptoConfig config;
        config.SetSpeed(1000);
        config.SetFilter(1);
        config.SetZeroing(0);


        success = daqs[i].SendConfig(config); // Set up the speed to 1000 Hz and filtering to 150 Hz
        std::cout << "Set speed is: " << config.GetSpeed() << std::endl;

        if (success) {
            std::cout << i + 1 << ". DAQ successfully configured." << std::endl;
        }
        else {
            std::cout << i + 1 << ". DAQ could not be configured." << std::endl;
            continue;
        }
        daqs[i].RequestSensitivityReport(); // This call is a must
    }


    // Create a container that can hold 10 6D packets
    OptoPackets6D packets(1);
    OptoPacket6D packet;

    //Set ROS rate to speed Hz
    int speed = 100;
    ros::Rate loop_rate(speed);
    ros::Time::init();

    geometry_msgs::WrenchStamped wrench_msg; // Create msg

    //Main ROS loop
    while(ros::ok())
    {
        if (daqs[0].IsValid()) 
        {
            //daqs[0].GetPackets6D(&packets, true); // blocking call, waits for 10 packets
            daqs[0].GetLastPacket6D(&packet, true);

        }
        // Show the captured packets Fx value in newtons
        std::size_t size = packets.GetSize(); // It should be 10.
        
        double Fx = 0;
        double Fy = 0;
        double Fz = 0;
        double Tx = 0;
        double Ty = 0;
        double Tz = 0;

        /*
        for (std::size_t j = 0; j < size; ++j) 
        {
            OptoPacket6D p;
            p = packets.GetPacket(j);
            if (p.IsValid()) 
            {
                Fx += p.GetFxInNewton();
                Fy += p.GetFyInNewton();
                Fz += p.GetFzInNewton();
                Tx += p.GetTxInNewtonMeter();
                Ty += p.GetTyInNewtonMeter();
                Tz += p.GetTzInNewtonMeter();

                //Fill msg
                wrench_msg.header.stamp = ros::Time::now();
                wrench_msg.wrench.force.x = Fx/size;
                wrench_msg.wrench.force.y = Fy/size;
                wrench_msg.wrench.force.z = Fz/size;
                wrench_msg.wrench.torque.x = Tx/size;
                wrench_msg.wrench.torque.y = Ty/size;
                wrench_msg.wrench.torque.z = Tz/size;
                
                wrench_pub.publish(wrench_msg);
            }
        }
        */


        Fx = packet.GetFxInNewton();
        Fy = packet.GetFyInNewton();
        Fz = packet.GetFzInNewton();
        Tx = packet.GetTxInNewtonMeter();
        Ty = packet.GetTyInNewtonMeter();
        Tz = packet.GetTzInNewtonMeter();

        //Fill msg
        wrench_msg.header.stamp = ros::Time::now();
        wrench_msg.wrench.force.x = Fx;
        wrench_msg.wrench.force.y = Fy;
        wrench_msg.wrench.force.z = Fz;
        wrench_msg.wrench.torque.x = Tx;
        wrench_msg.wrench.torque.y = Ty;
        wrench_msg.wrench.torque.z = Tz;
                
        wrench_pub.publish(wrench_msg);



        packets.Clear(); // Empty the container for the next DAQ

        ros::spinOnce();
        //loop_rate.sleep();

    }


    return 0;
}
