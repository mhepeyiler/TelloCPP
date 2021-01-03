#ifndef TELLOROS_INCLUDE_PARSER_HPP_
#define TELLOROS_INCLUDE_PARSER_HPP_

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

class Parser
{
public:
    Parser() = default;

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
    roll_t mroll{};
    pitch_t mpitch{};
    yaw_t myaw{};

    vx_t mvx{};
    vy_t mvy{};
    vz_t mvz{};

    ax_t max{};
    ay_t may{};
    az_t maz{};

    tof_t mtof{};
    height_t mheight{};
    templ_t mtempl{};
    temph_t mtemph{};
    tm_t mtm{};
    bar_t mbar{};
    bat_t mbat{};
};

#endif