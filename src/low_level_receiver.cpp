/* MIT License
 *
 * Copyright (c) 2020 Murat Hepeyiler
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
**/

#include "low_level_receiver.hpp"
#include <iostream>
#include <algorithm>
#include <iterator>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/asio/signal_set.hpp>

using boost::asio::ip::address;
using boost::asio::ip::udp;

struct LowLevelReceiver::LowLevelReceiver_private_
{
    LowLevelReceiver_private_(std::array<char, 1600> &buff, const std::string &ip, int port);
    ~LowLevelReceiver_private_();
    void handle_receive(const boost::system::error_code &error, size_t bytes_transferred);
    void receive();
    void wait();
    void interrupt_handler(const boost::system::error_code &error, int signal_number);
    void interrupt();

    std::array<char, 1600> &buff_;
    std::string ip_;
    int port_;
    boost::asio::io_service io_service_;
    udp::socket socket_;
    std::array<char, 1600> recv_buffer_{};
    udp::endpoint remote_endpoint_;
    bool read_flag_{true}, get_flag_{false};
    size_t count_;
};

LowLevelReceiver::LowLevelReceiver_private_::LowLevelReceiver_private_(std::array<char, 1600> &buff, const std::string &ip, int port)
    : buff_{buff}, ip_{ip}, port_{port}, socket_{io_service_}, count_{0} {}

LowLevelReceiver::LowLevelReceiver_private_::~LowLevelReceiver_private_()
{
    boost::system::error_code err;
    socket_.shutdown(udp::socket::shutdown_receive, err);
    socket_.close();
}

void LowLevelReceiver::LowLevelReceiver_private_::handle_receive(const boost::system::error_code &error, size_t bytes_transferred)
{
    if (error)
    {
        std::cout << "\nReceive failed: " << error.message() << '\n';
    }
    
    // critical section
    while(get_flag_);
    read_flag_ = true;
    std::copy(begin(recv_buffer_), begin(recv_buffer_) + bytes_transferred, begin(buff_));
    count_ = bytes_transferred;
    read_flag_ = false;

    wait();
}

void LowLevelReceiver::LowLevelReceiver_private_::receive()
{
    
    socket_.open(udp::v4());
    socket_.set_option(boost::asio::socket_base::reuse_address(true));
    socket_.bind(udp::endpoint(address::from_string(ip_), port_));

    wait();

    io_service_.run();
}

void LowLevelReceiver::LowLevelReceiver_private_::wait()
{
    socket_.async_receive_from(boost::asio::buffer(recv_buffer_), remote_endpoint_,
                               boost::bind(&LowLevelReceiver_private_::handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void LowLevelReceiver::LowLevelReceiver_private_::interrupt_handler(const boost::system::error_code &error, int signal_number)
{
    if (error)
    {
        std::cerr << "Error occurred! Interrupt!\n";
    }
    io_service_.stop();
}

void LowLevelReceiver::LowLevelReceiver_private_::interrupt()
{
    boost::asio::signal_set signals(io_service_, SIGINT, SIGTERM);
    signals.async_wait(boost::bind(&LowLevelReceiver_private_::interrupt_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::signal_number));
}

/*******************************************************/

LowLevelReceiver::LowLevelReceiver(std::array<char, 1600> &arr, const std::string &ip, int port)
    : pc{new LowLevelReceiver_private_{arr, ip, port}} {}

void LowLevelReceiver::receive()
{
    pc->receive();
    pc->interrupt();
}

LowLevelReceiver::~LowLevelReceiver()
{
    pc.release();
}

bool LowLevelReceiver::getFlagRead() const
{
    return pc->read_flag_;
}

void LowLevelReceiver::resetFlagGet()
{
    pc->get_flag_ = false;
}

void LowLevelReceiver::setFlagGet()
{
    pc->get_flag_ = true;
}

size_t LowLevelReceiver::getLength() const
{
    return pc->count_;
}

pid_t LowLevelReceiver::getPid()const
{
    return getpid();
}