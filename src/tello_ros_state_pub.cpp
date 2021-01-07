#include "tello_ros_state.hpp"
#include "low_level_receiver.hpp"
#include <string>
#include <array>
#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Pose.h"
#include "ros/time.h"
#include "sensor_msgs/BatteryState.h"
#include <thread>

int main(int argc, char** argv)
{
    const std::string ip = "0.0.0.0";
    const int port = 8890;
    std::array<char, 1600> recvArr;

    ros::init(argc, argv, "tello_state");
    ros::NodeHandle n;

    std::map<Topic, ros::Publisher> pubMap;

    pubMap[Topic::Orientation] = n.advertise<geometry_msgs::Quaternion>("tello_orien", 1000);
    pubMap[Topic::Velocity] = n.advertise<geometry_msgs::Twist>("tello_vel", 1000);
    pubMap[Topic::Position] = n.advertise<geometry_msgs::Pose>("tello_pos", 1000);
    pubMap[Topic::Battery] = n.advertise<sensor_msgs::BatteryState>("tello_bat", 1000);
    pubMap[Topic::Acceleration] = n.advertise<geometry_msgs::Accel>("tello_acc", 1000);

    State tello_state{pubMap};
    
    LowLevelReceiver low_receiver{recvArr, ip, port};
    std::thread r([&] {low_receiver.receive();});

    ros::Rate sleep_rate(10);

    while(ros::ok())
    {
        while(!low_receiver.getflag());

        tello_state(recvArr, low_receiver.getlength());
        low_receiver.resetflag();
        
        tello_state.pubPos();
        tello_state.pubAcc();
        tello_state.pubBat();
        tello_state.pubOrien();
        tello_state.pubVel();       

        ros::spinOnce();
        sleep_rate.sleep();
    }
    
    r.join();
}