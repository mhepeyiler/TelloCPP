#include "Receiver.hpp"
#include <boost/asio.hpp>

using boost::asio::ip::udp;
using boost::asio::ip::address;

struct Receiver::Private_cont
{
    Private_cont(std::array<char,1600>& buff, const std::string& ip, int port)
        :   mip{ip}, mport{port}, msocket{mio_service}, mbuff{buff} 
    {}

    std::string mip;
    int mport;
    boost::asio::io_service mio_service;
    udp::socket msocket;
    std::array<char,1600>& mbuff;
    udp::endpoint mremote_endpoint;
};

