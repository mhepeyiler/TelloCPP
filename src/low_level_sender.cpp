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