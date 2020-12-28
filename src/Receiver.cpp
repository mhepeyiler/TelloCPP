#include "Receiver.hpp"
#include <iostream>
#include <algorithm>
#include <iterator>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

using boost::asio::ip::udp;
using boost::asio::ip::address;

struct Receiver::Private_cont
{
    std::array<char,1600>& mbuff;
    std::string mip;
    int mport;
    boost::asio::io_service mio_service;
    udp::socket msocket;
    std::array<char, 1600> mrecv_buffer;
    udp::endpoint mremote_endpoint;


    Private_cont(std::array<char,1600>& buff, const std::string& ip, int port)
        :   mbuff{buff}, mip{ip}, mport{port}, msocket{mio_service}
    {}

    void handle_receive(const boost::system::error_code& error, size_t bytes_transferred)
    {
        if(error)
        {
            std::cout << "\nReceive failed: " << error.message() << '\n';
        }
        std::copy(begin(mrecv_buffer), begin(mrecv_buffer)+bytes_transferred, begin(mbuff));
        copy(begin(mbuff), begin(mbuff) + bytes_transferred, std::ostream_iterator<char>{std::cout, " "});
    }

    void wait()
    {
        msocket.async_receive_from(boost::asio::buffer(mrecv_buffer), mremote_endpoint,
            boost::bind(&Private_cont::handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }

    void receive()
    {
        msocket.open(udp::v4());
        msocket.bind(udp::endpoint(address::from_string(mip), mport));

        wait();

        mio_service.run();
    }
};

Receiver::Receiver(std::array<char,1600>& arr, const std::string& ip, int port)
                    :   pc{new Private_cont{arr, ip, port}}
{

}

void Receiver::receive()
{
    pc->receive();
}

Receiver::~Receiver()
{
    pc.release();
}