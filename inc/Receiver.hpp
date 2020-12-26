#ifndef TELLOCPP_INC_RECEIVER_HPP_
#define TELLOCPP_INC_RECEIVER_HPP_

#include <string>
#include <memory>
#include <array>
class Receiver
{
public:
    Receiver(const std::string&, int);
    void set_buffer(std::array<char,1600>&);
    void receive();
private:
    struct Private_cont;
    std::unique_ptr<Private_cont> pc;
};


#endif