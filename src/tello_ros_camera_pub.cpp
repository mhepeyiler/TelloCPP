#include "low_level_receiver.hpp"
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "signal.h"
#include "sys/types.h"
#include <thread>
#include <algorithm>
#include "ros/master.h"

pid_t recv_pid{};

void sigIntHandler(int sig)
{

    if(recv_pid != 0)
    {
        kill(recv_pid, sig);
    }
    ros::shutdown();
    abort();
}

int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "camera_node", ros::init_options::NoSigintHandler);
    ros::master::V_TopicInfo topic_info;
    ros::NodeHandle n;
    if(ros::master::getTopics(topic_info))
    {
        if(find_if(cbegin(topic_info), cend(topic_info), [](const auto& val) {return val.name == "tello_command";}) == cend(topic_info))
        {
            ROS_FATAL("tello_command topic needs to be initialized.");
            ros::shutdown();
            exit(EXIT_FAILURE);
        }   
    }    
    else
    {
        ROS_FATAL("ROS master needs to be initialized.");
        ros::shutdown();
        abort();
    }

    const std::string ip = "0.0.0.0";
    const int port = 11111;
    std::array<char, 1600> recv_arr;

    
    ros::Publisher camera_pub = n.advertise<sensor_msgs::CompressedImage>("tello_camera", 1000);
    sensor_msgs::CompressedImage cam_msg;
    cam_msg.format = "H.264";

    LowLevelReceiver low_receiver{recv_arr, ip, port};
    
    recv_pid = low_receiver.getPid();
    std::thread r([&] {low_receiver.receive();});

    signal(SIGINT, sigIntHandler);
    size_t count{};
    ros::Rate sleep_rate{10};


    while(ros::ok())
    {
        while(low_receiver.getFlagRead()); 
        low_receiver.setFlagGet();
        copy(cbegin(recv_arr), cbegin(recv_arr) + low_receiver.getLength(), begin(cam_msg.data));        
        low_receiver.resetFlagGet();

        cam_msg.header.frame_id = count++;
        camera_pub.publish(cam_msg);

        sleep_rate.sleep();
    }
    r.join();
}