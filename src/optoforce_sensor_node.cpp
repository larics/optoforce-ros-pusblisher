#include <iostream>
#include "optoforce_sensor/opto.h"
#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include <unistd.h>

void msSleep(unsigned long p_uMillisecs)
{
	usleep(p_uMillisecs * 1000);
}


int main(int argc, char **argv)
{
        
    OptoDAQ daq;
    OptoPorts ports;
    msSleep(2500); // We wait some ms to be sure about OptoPorts enumerated PortList

    OPort* portlist=ports.listPorts(true);

    if (ports.getLastSize()>0)
    {
    	daq.open(portlist[0]);

        if (daq.getVersion()!=_95 && daq.getVersion() != _64) // It is a 3D sensor
        {
            OptoPackage pack3D;
            int size=daq.read(pack3D,false);	// Reading Sensor #0 (up to 16 Sensors)
            std::cout<<"x: "<<pack3D.x<<" y: "<<pack3D.y<<" z: "<<pack3D.z<<std::endl;
            std::cout<<"Sensor not supported in ROS yet"<<std::endl;
            //Support should be added later

        }
        else					  // It is a 6D sensor = the only one supported for now
        {
            OptoPackage6D pack6D;
            int size=daq.read6D(pack6D,false);
            //init ROS node
            std::cout<<"Found 6DOF sensor -> Starting ROS node"<<std::endl;
            ros::init(argc, argv, "OForceSensorPublisher");
            ros::NodeHandle n;
            //Creating wrench publisher
            ros::Publisher wrench_pub = n.advertise<geometry_msgs::Wrench>("OptoForceWrench", 1000);
            //Set ROS rate to 10 Hz
            ros::Rate loop_rate(100);
            //Main ROS loop
            while(ros::ok())
            {
                geometry_msgs::Wrench wrench_msg; // Create msg
                //Fill msg
                wrench_msg.force.x = pack6D.Fx; 
                wrench_msg.force.y = pack6D.Fy;
                wrench_msg.force.z = pack6D.Fz;
                wrench_msg.torque.x = pack6D.Tx;
                wrench_msg.torque.x = pack6D.Ty;
                wrench_msg.torque.x = pack6D.Tz;
                wrench_pub.publish(wrench_msg);
                ros::spinOnce();
                loop_rate.sleep();
                size = daq.read6D(pack6D,false);
		std::cout<<size<<std::endl;
            }
            //std::cout<<"Fx: "<<pack6D.Fx<<" Fy: "<<pack6D.Fy<<" Fz: "<<pack6D.Fz<<" ";
            //std::cout<<"Tx: "<<pack6D.Tx<<" Ty: "<<pack6D.Ty<<" Tz: "<<pack6D.Tz<<std::endl;
        }
        std::cout<<"Closing sensor connection"<<std::endl;
        daq.close();
    }
    else
    {
    	std::cout<<"No sensor available"<<std::endl;
    }
    return 0;
}
