#ifndef TELLOROS_INCLUDE_LOWLEVELSENDER_HPP_
#define TELLOROS_INCLUDE_LOWLEVELSENDER_HPP_

#include <string>
#include <memory>

class LowLevelSender
{
public:
    LowLevelSender(const std::string &, int);
    ~LowLevelSender();
    size_t operator()(const std::string &in);

private:
    struct LowLevelSender_private_;
    std::unique_ptr<LowLevelSender_private_> pc;
};

#endif