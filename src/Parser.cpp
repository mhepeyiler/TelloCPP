#include <Parser.hpp>
#include <type_traits>
#include <iostream>

template<typename T>
static T getVal(const std::string& str)
{
    if constexpr(std::is_same_v<T, double>)
    {
        return std::stod(str);
    }else if constexpr(std::is_same_v<T, int>)
    {
        return std::stoi(str);
    }
}

static std::string getToken(const std::string& str, size_t& pos)
{
    size_t beg = str.find_first_of(":", pos);
    size_t end = str.find_first_of(";", pos);
    pos = end;
    return std::string{str, beg+1, end};
}


void Parser::operator()(const std::string& str)
{
    size_t pos = 0;

    mpitch = getVal<pitch_t>(getToken(str, pos));    
    std::cout << mpitch;
    mroll = getVal<roll_t>(getToken(str, pos));
    myaw = getVal<yaw_t>(getToken(str, pos));

    mvx = getVal<vx_t>(getToken(str, pos));
    mvy = getVal<vy_t>(getToken(str, pos));
    mvz = getVal<vz_t>(getToken(str, pos));

    mtempl = getVal<templ_t>(getToken(str, pos));
    mtemph = getVal<temph_t>(getToken(str, pos));

    mtof = getVal<tof_t>(getToken(str, pos));
    mheight = getVal<height_t>(getToken(str, pos));

    mbat = getVal<bat_t>(getToken(str, pos));
    mbar = getVal<bar_t>(getToken(str, pos));
    
    mtm = getVal<tm_t>(getToken(str, pos));
    
    max = getVal<ax_t>(getToken(str, pos));
    may = getVal<ay_t>(getToken(str, pos));
    maz = getVal<az_t>(getToken(str, pos));
    
}

std::tuple<roll_t, pitch_t, yaw_t> Parser::getOrien() const
{
    return std::tuple<roll_t, pitch_t, yaw_t>{mroll, mpitch, myaw};
}

std::tuple<vx_t, vy_t, vz_t> Parser::getVel() const
{
    return std::tuple<vx_t, vy_t, vz_t>{mvx, mvy, mvz};
}

std::tuple<ax_t, ay_t, az_t> Parser::getAcc() const
{
    return std::tuple<ax_t, ay_t, az_t>{max, may, maz};
}

std::pair<templ_t, temph_t> Parser::getMinMaxTemp() const
{
    return std::pair<templ_t, temph_t>{mtempl, mtemph};
}

tof_t Parser::getTOF() const
{
    return mtof;
}
 
height_t Parser::getHeight() const
{
    return mheight;
}

tm_t Parser::getMotorOnTime() const
{
    return mtm;    
}

bar_t Parser::getBar() const
{
    return mbar;
}

bat_t Parser::getBattery() const
{
    return mbat;
}

