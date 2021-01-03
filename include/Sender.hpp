#ifndef TELLOROS_INCLUDE_SENDER_HPP_
#define TELLOROS_INCLUDE_SENDER_HPP_

#include <string>
#include <memory>

class Sender
{
public:
    Sender(const std::string &, int);
    ~Sender();
    size_t operator()(const std::string &in);

private:
    struct mSender_private;
    std::unique_ptr<mSender_private> pc;
};

#endif