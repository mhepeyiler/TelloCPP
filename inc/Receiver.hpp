#ifndef TELLOCPP_INC_RECEIVER_HPP_
#define TELLOCPP_INC_RECEIVER_HPP_

#include <string>
#include <memory>
#include <array>

class Receiver
{
public:
    Receiver(std::array<char,1600>&, const std::string&, int);
    ~Receiver();
    void receive();
private:
    struct Private_cont;
    std::unique_ptr<Private_cont> pc;
};


#endif