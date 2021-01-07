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
    State(std::map<Topic, ros::Publisher>& pubMap) : pubMap_{pubMap} {}
    void operator()(const std::array<char, 1600>&, size_t n);
    void pubOrien()const;
    void pubVel()const;
    void pubPos()const;
    void pubBat()const;
    void pubAcc()const;

private:
    std::map<Topic, ros::Publisher>& pubMap_;
    TelloParser tp_;
};



#endif