#ifndef TELLOCPP_INC_RECEIVER_HPP_
#define TELLOCPP_INC_RECEIVER_HPP_

#include <string>
#include <memory>
#include <array>

class Receiver
{
public:
    Receiver(std::array<char, 1600> &, const std::string &, int);
    ~Receiver();
    void receive();
    bool getflag() const;
    void resetflag();
    size_t getlength()const;
private:
    struct mReceiver_private;
    std::unique_ptr<mReceiver_private> pc;
};

#endif