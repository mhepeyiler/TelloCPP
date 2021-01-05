#ifndef TELLOROS_INCLUDE_TELLO_ROS_STATE_HPP_
#define TELLOROS_INCLUDE_TELLO_ROS_STATE_HPP_

#include <map>
#include "ros/ros.h"
#include "tello_parser.hpp"

enum class Topic : char
{
    Orientation,
    Velocity,
    Position,
    Battery,
    Acceleration
};

class State : protected Param
{
public:

    void pubOrien()const;
    void pubVel()const;
    void pubPos()const;
    void pubBat()const;
    void pubAcc()const;

private:
    std::map<Topic, ros::Publisher>& pubMap_;
};



#endif