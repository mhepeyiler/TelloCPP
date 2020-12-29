#include "Receiver.hpp"
#include <iostream>
#include <algorithm>
#include <iterator>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/asio/signal_set.hpp>

using boost::asio::ip::address;
using boost::asio::ip::udp;

struct Receiver::mReceiver_private
{
    mReceiver_private(std::array<char, 1600> &buff, const std::string &ip, int port);
    ~mReceiver_private();
    void handle_receive(const boost::system::error_code &error, size_t bytes_transferred);
    void receive();
    void wait();
    void interrupt_handler(const boost::system::error_code &error, int signal_number);
    void interrupt();

    std::array<char, 1600> &mbuff;
    std::string mip;
    int mport;
    boost::asio::io_service mio_service;
    udp::socket msocket;
    std::array<char, 1600> mrecv_buffer{};
    udp::endpoint mremote_endpoint;
    bool mend_flag{false};
};

Receiver::mReceiver_private::mReceiver_private(std::array<char, 1600> &buff, const std::string &ip, int port)
    : mbuff{buff}, mip{ip}, mport{port}, msocket{mio_service} {}

Receiver::mReceiver_private::~mReceiver_private()
{
    boost::system::error_code err;
    std::cout << "test";
    msocket.shutdown(udp::socket::shutdown_receive, err);
    msocket.close();
}

void Receiver::mReceiver_private::handle_receive(const boost::system::error_code &error, size_t bytes_transferred)
{
    if (error)
    {
        std::cout << "\nReceive failed: " << error.message() << '\n';
    }
    std::cout << "Received\n";
    std::copy(begin(mrecv_buffer), begin(mrecv_buffer) + bytes_transferred, begin(mbuff));
    mend_flag = true;
    wait();
}

void Receiver::mReceiver_private::receive()
{
    msocket.open(udp::v4());
    msocket.bind(udp::endpoint(address::from_string(mip), mport));

    wait();

    mio_service.run();

    
}


void Receiver::mReceiver_private::wait()
{
    msocket.async_receive_from(boost::asio::buffer(mrecv_buffer), mremote_endpoint,
                               boost::bind(&mReceiver_private::handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}


void Receiver::mReceiver_private::interrupt_handler(const boost::system::error_code &error, int signal_number)
{
    if(error)
    {
        std::cerr << "Error occurred! Interrupt!\n";
    }
    mio_service.stop();
}

void Receiver::mReceiver_private::interrupt()
{
    boost::asio::signal_set signals(mio_service, SIGINT, SIGTERM);
    signals.async_wait(boost::bind(&mReceiver_private::interrupt_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::signal_number));
}

/*******************************************************/

Receiver::Receiver(std::array<char, 1600> &arr, const std::string &ip, int port)
    : pc{new mReceiver_private{arr, ip, port}}   { }


void Receiver::receive()
{
    pc->receive();
    pc->interrupt();
}

Receiver::~Receiver()
{
    pc.release();
}

bool Receiver::getflag() const
{
    return pc->mend_flag;
}

void Receiver::resetflag()
{
    pc->mend_flag = false;
}