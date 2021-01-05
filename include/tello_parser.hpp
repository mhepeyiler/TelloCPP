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


#ifndef TELLOROS_INCLUDE_TELLOPARSER_HPP_
#define TELLOROS_INCLUDE_TELLOPARSER_HPP_

#include <string>
#include <tuple>
#include <utility>

using roll_t = int;
using pitch_t = int;
using yaw_t = int;

using vx_t = int;
using vy_t = int;
using vz_t = int;

using ax_t = double;
using ay_t = double;
using az_t = double;

using tof_t = int;
using height_t = int;
using templ_t = int; 
using temph_t = int;
using tm_t = int;
using bar_t = double;
using bat_t = double;

class Param
{
protected:
    roll_t roll_{};
    pitch_t pitch_{};
    yaw_t yaw_{};

    vx_t vx_{};
    vy_t vy_{};
    vz_t vz_{};

    ax_t ax_{};
    ay_t ay_{};
    az_t az_{};

    tof_t tof_{};
    height_t height_{};
    templ_t templ_{};
    temph_t temph_{};
    tm_t tm_{};
    bar_t bar_{};
    bat_t bat_{};
};

class TelloParser : protected Param
{
public:
    TelloParser() = default;

    void operator()(const std::string &);

    std::tuple<roll_t, pitch_t, yaw_t> getOrien() const;
    std::tuple<vx_t, vy_t, vz_t> getVel() const;
    std::tuple<ax_t, ay_t, az_t> getAcc() const;

    std::pair<templ_t, temph_t> getMinMaxTemp() const;

    tof_t getTOF() const;
    height_t getHeight() const;

    tm_t getMotorOnTime() const;
    bar_t getBar() const;
    bat_t getBattery() const;

private:
};

#endif