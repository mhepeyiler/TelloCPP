#include <cmath>
#include "tello_ros_state.hpp"
#include "low_level_receiver.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Pose.h"
#include "ros/time.h"
#include "sensor_msgs/BatteryState.h"


static double degree2Radian(double deg)
{
    if(deg < 0)
        return -1*deg*M_PI/180;
    return deg*M_PI/180;    
}

void State::pubOrien()const
{
    static tf2::Quaternion myQuaternion;
    static geometry_msgs::Quaternion msg;

    myQuaternion.setRPY(roll_, pitch_, yaw_);
    msg.x = myQuaternion.x();
    msg.y = myQuaternion.y();
    msg.z = myQuaternion.z();
    msg.w = myQuaternion.w();

    pubMap_[Topic::Orientation].publish(msg);
}   

void State::pubVel()const
{
    static geometry_msgs::Twist msg;
    msg.linear.x = vx_;
    msg.linear.y = vy_;
    msg.linear.z = vz_;
    
    pubMap_[Topic::Velocity].publish(msg);
}


void State::pubPos()const
{
    // TODO : update x and y
    static geometry_msgs::Pose msg;

    msg.position.x = 0; 
    msg.position.y = 0;
    msg.position.z = height_;

    pubMap_[Topic::Position].publish(msg);
}

void State::pubBat()const
{
    static sensor_msgs::BatteryState msg;
    msg.percentage = bat_;

    pubMap_[Topic::Battery].publish(msg);
}

void State::pubAcc()const
{
    static geometry_msgs::Accel msg;

    msg.linear.x = ax_;
    msg.linear.y = ay_;
    msg.linear.z = az_;

    pubMap_[Topic::Acceleration].publish(msg);
}