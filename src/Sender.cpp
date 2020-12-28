#include "Sender.hpp"
#include <boost/asio.hpp>
#include <iostream>

using boost::asio::ip::udp;
using boost::asio::ip::address;

struct Sender::Private_cont
{
    Private_cont(const std::string& ip, int port)
        :   mip(ip), mport(port), socket{io_service}
    {
        remote_endpoint = udp::endpoint(address::from_string(mip), port);
        
    }

    size_t send(const std::string& in) noexcept
    {   
        try
        {
            socket.open(udp::v4());
            boost::system::error_code err;
            auto sent = socket.send_to(boost::asio::buffer(in), remote_endpoint, 0, err);
            socket.close();
            std::cout << "\nSent payload..." << sent << "\n";

            return sent;

        }
        catch(const boost::system::system_error& err)
        {
            std::cout << err.what() << "\n";
        }
        catch(...)
        {
            std::cout << "Unknown error!\n";
        }

        return 0;
    }

    
    std::string mip;
    int mport;
    boost::asio::io_service io_service;
    udp::socket socket;
    udp::endpoint remote_endpoint;
};

Sender::Sender(const std::string& ip, int port)
    :   pc(new Private_cont{ip, port})
{
    
}

Sender::~Sender()
{
    pc.release();
}

size_t Sender::operator()(const std::string& in)
{
    return pc->send(in);
}