#include "low_level_sender.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>

class LowLevelComm
{
public:

    LowLevelComm() : 
        low_comm_{ip_, port_}  {}

    void sendData(const std_msgs::String::ConstPtr& msg)
    {
       low_comm_(msg->data.c_str());
    }

private:
    
    const std::string ip_ = "192.168.10.1";
    const int port_ = 8889;
    LowLevelSender low_comm_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tello_ros_low_sender");
    ros::NodeHandle n;
    LowLevelComm tello;
    const ros::Subscriber sub = n.subscribe<std_msgs::String>("/tello_command", 1000, &LowLevelComm::sendData, &tello);
    ros::spin();
}