#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"

// Global variables (also modified by the topic subscriber):
bool triggerSens=false;
struct timeval timeV;
unsigned int updateBySubs=0;
float simTime=0.0;

// Topic subscriber callbacks:
void sensorCallback(const std_msgs::Bool& sensTrigger)
{
    if (gettimeofday(&timeV,NULL)==0)
        updateBySubs=timeV.tv_sec;
    triggerSens=sensTrigger.data;
}

void simulationTimeCallback(const std_msgs::Float32& simTime)
{
    simTime=simTime.data;
}

// Main code:
int main(int argc,char* argv[])
{
    // The robot motor velocities and the sensor topic names are given in the argument list
    // (when CoppeliaSim launches this executable, CoppeliaSim will also provide the argument list)
    std::string left_motor_topic;
    std::string right_motor_topic;
    std::string sensor_topic;
    std::string simulation_time_topic;
    if (argc>=5)
    {
        left_motor_topic=argv[1];
        right_motor_topic=argv[2];
        sensor_topic=argv[3];
        simulation_time_topic=argv[4];
        left_motor_topic="/"+left_motor_topic;
        right_motor_topic="/"+right_motor_topic;
        sensor_topic="/"+sensor_topic;
        simulation_time_topic="/"+simulation_time_topic;
    }
    else
    {
        printf("Indicate following arguments: 'left_motor_topic right_motor_topic sensor_topic simulation_time_topic'!\n");
        sleep(5000);
        return 0;
    }

    // Create a ROS node. The name has a random component: 
    int _argc = 0;
    char** _argv = NULL;
    if (gettimeofday(&timeV,NULL)==0)
        updateBySubs=(timeV.tv_sec*1000+timeV.tv_usec/1000)&0x00ffffff;
    std::string nodeName("rosBubbleRob");
    std::string randId(boost::lexical_cast<std::string>(updateBySubs+int(999999.0f*(rand()/(float)RAND_MAX))));
    nodeName+=randId;       
    ros::init(_argc,_argv,nodeName.c_str());

    if(!ros::master::check())
        return(0);
    
    ros::NodeHandle node("~");  
    printf("rosBubbleRob2 just started with node name %s\n",nodeName.c_str());

    // 1. Let's subscribe to the sensor and simulation time stream
    ros::Subscriber subSensor=node.subscribe(sensor_topic.c_str(),1,sensorCallback);
    ros::Subscriber subSimulationTime=node.subscribe(simulation_time_topic.c_str(),1,simulationTimeCallback);

    // 2. Let's prepare publishers for the motor speeds:
    ros::Publisher leftMotorSpeedPub=node.advertise<std_msgs::Float32>(left_motor_topic.c_str(),1);
    ros::Publisher rightMotorSpeedPub=node.advertise<std_msgs::Float32>(right_motor_topic.c_str(),1);

    // 3. Finally we have the control loop:
    float driveBackStartTime=-99.0f;
    unsigned int currentTime;
    if (gettimeofday(&timeV,NULL)==0)
    {
        updateBySubs=timeV.tv_sec;
        currentTime=updateBySubs;
    }
    while (ros::ok())
    { // this is the control loop (very simple, just as an example)
        if (gettimeofday(&timeV,NULL)==0)
        {
            currentTime=timeV.tv_sec;
            if (currentTime-updateBySubs>9)
                break; // we didn't receive any sensor information for quite a while... we leave
        }
        float desiredLeftMotorSpeed;
        float desiredRightMotorSpeed;
        if (simTime-driveBackStartTime<3.0f)
        { // driving backwards while slightly turning:
            desiredLeftMotorSpeed=-3.1415*0.5;
            desiredRightMotorSpeed=-3.1415*0.25;
        }
        else
        { // going forward:
            desiredLeftMotorSpeed=3.1415;
            desiredRightMotorSpeed=3.1415;
            if (triggerSens)
                driveBackStartTime=simTime; // We detected something, and start the backward mode
            triggerSens=false;
        }

        // publish the motor speeds:
        std_msgs::Float32 d;
        d.data=desiredLeftMotorSpeed;
        leftMotorSpeedPub.publish(d);
        d.data=desiredRightMotorSpeed;
        rightMotorSpeedPub.publish(d);

        // handle ROS messages:
        ros::spinOnce();

        // sleep a bit:
        usleep(5000);
    }
    ros::shutdown();
    printf("rosBubbleRob just ended!\n");
    return(0);
}

