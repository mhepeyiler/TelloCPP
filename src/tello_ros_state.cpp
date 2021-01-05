/* MIT License
 *
 * Copyright (c) 2020 Murat Hepeyiler
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
**/

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