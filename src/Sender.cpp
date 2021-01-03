#include "Sender.hpp"
#include <boost/asio.hpp>
#include <iostream>

using boost::asio::ip::address;
using boost::asio::ip::udp;

struct Sender::mSender_private
{
    mSender_private(const std::string &ip, int port);
    size_t send(const std::string &in) noexcept;

    std::string mip;
    int mport;
    boost::asio::io_service io_service;
    udp::socket socket;
    udp::endpoint remote_endpoint;
};

Sender::mSender_private::mSender_private(const std::string &ip, int port)
    : mip(ip), mport(port), socket{io_service}
{
    remote_endpoint = udp::endpoint(address::from_string(mip), port);
}

size_t Sender::mSender_private::send(const std::string &in) noexcept
{
    try
    {
        socket.open(udp::v4());
        boost::system::error_code err;
        auto sent = socket.send_to(boost::asio::buffer(in), remote_endpoint, 0, err);
        socket.close();

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

Sender::Sender(const std::string &ip, int port)
    : pc(new mSender_private{ip, port})
{
}

Sender::~Sender()
{
    pc.release();
}

size_t Sender::operator()(const std::string &in)
{
    return pc->send(in);
}