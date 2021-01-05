#include "low_level_sender.hpp"
#include <boost/asio.hpp>
#include <iostream>

using boost::asio::ip::address;
using boost::asio::ip::udp;

struct LowLevelSender::LowLevelSender_private_
{
    LowLevelSender_private_(const std::string &ip, int port);
    size_t send(const std::string &in) noexcept;

    std::string ip_;
    int port_;
    boost::asio::io_service io_service_;
    udp::socket socket_;
    udp::endpoint remote_endpoint_;
};

LowLevelSender::LowLevelSender_private_::LowLevelSender_private_(const std::string &ip, int port)
    : ip_(ip), port_(port), socket_{io_service_}
{
    remote_endpoint_ = udp::endpoint(address::from_string(ip_), port_);
}

size_t LowLevelSender::LowLevelSender_private_::send(const std::string &in) noexcept
{
    try
    {
        socket_.open(udp::v4());
        boost::system::error_code err;
        auto sent = socket_.send_to(boost::asio::buffer(in), remote_endpoint_, 0, err);
        socket_.close();

        return sent;
    }
    catch (const boost::system::system_error &err)
    {
        std::cout << err.what() << "\n";
    }
    catch (...)
    {
        std::cout << "Unknown error!\n";
    }

    return 0;
}

LowLevelSender::LowLevelSender(const std::string &ip, int port)
    : pc(new LowLevelSender_private_{ip, port})
{
}

LowLevelSender::~LowLevelSender()
{
    pc.release();
}

size_t LowLevelSender::operator()(const std::string &in)
{
    return pc->send(in);
}