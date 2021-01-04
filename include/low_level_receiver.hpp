#ifndef TELLO_ROS_INCLUDE_LowLevelReceiver_HPP_
#define TELLO_ROS_INCLUDE_LowLevelReceiver_HPP_

#include <string>
#include <memory>
#include <array>

class LowLevelReceiver
{
public:
    LowLevelReceiver(std::array<char, 1600> &, const std::string &, int);
    ~LowLevelReceiver();
    void receive();
    bool getflag() const;
    void resetflag();
    size_t getlength()const;
private:
    struct LowLevelReceiver_private_;
    std::unique_ptr<LowLevelReceiver_private_> pc;
};

#endif