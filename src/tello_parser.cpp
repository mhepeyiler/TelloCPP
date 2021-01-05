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

#include <tello_parser.hpp>
#include <type_traits>

template<typename T>
static T getVal(const std::string& str)
{
    if constexpr(std::is_same_v<T, double>)
    {
        return std::stod(str);
    }
    else if constexpr(std::is_same_v<T, int>)
    {
        return std::stoi(str);
    }
}

static std::string getToken(const std::string& str, size_t& pos)
{
    size_t beg = str.find_first_of(":", pos);
    size_t end = str.find_first_of(";", beg);
    pos = end;
    return std::string{str, beg+1, end-pos-1};
}


void TelloParser::operator()(const std::string& str)
{
    size_t pos = 0;

    pitch_ = getVal<pitch_t>(getToken(str, pos));
    roll_ = getVal<roll_t>(getToken(str, pos));
    yaw_ = getVal<yaw_t>(getToken(str, pos));

    vx_ = getVal<vx_t>(getToken(str, pos));
    vy_ = getVal<vy_t>(getToken(str, pos));
    vz_ = getVal<vz_t>(getToken(str, pos));

    templ_ = getVal<templ_t>(getToken(str, pos));
    temph_ = getVal<temph_t>(getToken(str, pos));

    tof_ = getVal<tof_t>(getToken(str, pos));
    height_ = getVal<height_t>(getToken(str, pos));

    bat_ = getVal<bat_t>(getToken(str, pos));
    bar_ = getVal<bar_t>(getToken(str, pos));
    
    tm_ = getVal<tm_t>(getToken(str, pos));
    
    ax_ = getVal<ax_t>(getToken(str, pos));
    ay_ = getVal<ay_t>(getToken(str, pos));
    az_ = getVal<az_t>(getToken(str, pos));
    
}

std::tuple<roll_t, pitch_t, yaw_t> TelloParser::getOrien() const
{
    return std::tuple<pitch_t, roll_t, yaw_t>{pitch_, roll_, yaw_};
}

std::tuple<vx_t, vy_t, vz_t> TelloParser::getVel() const
{
    return std::tuple<vx_t, vy_t, vz_t>{vx_, vy_, vz_};
}

std::tuple<ax_t, ay_t, az_t> TelloParser::getAcc() const
{
    return std::tuple<ax_t, ay_t, az_t>{ax_, ay_, az_};
}

std::pair<templ_t, temph_t> TelloParser::getMinMaxTemp() const
{
    return std::pair<templ_t, temph_t>{templ_, temph_};
}

tof_t TelloParser::getTOF() const
{
    return tof_;
}
 
height_t TelloParser::getHeight() const
{
    return height_;
}

tm_t TelloParser::getMotorOnTime() const
{
    return tm_;    
}

bar_t TelloParser::getBar() const
{
    return bar_;
}

bat_t TelloParser::getBattery() const
{
    return bat_;
}