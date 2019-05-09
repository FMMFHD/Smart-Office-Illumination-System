#ifndef SERVER_SERVIDOR_H
#define SERVER_SERVIDOR_H

#include <list>
#include <cstddef>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#
#include <thread>
#include <iostream>

using namespace boost::asio;
using boost::system::error_code;
using namespace boost::asio;
using namespace boost::posix_time;

class session : public std::enable_shared_from_this<session> {
  public:
    session(ip::tcp::socket socket);
    void start();
    int stream;
    ip::tcp::socket s;
    enum {
        max_length = 1024
    };
    char data_[max_length];
    int read_buffer4;
    int read_buffer5;
  private:
    void do_read();
    void do_write(size_t length);
    void do_stream(const boost::system::error_code &ec);

};

class server {
  public:
    server(io_service& io_service, short port);

  private:
    ip::tcp::acceptor acceptor_;
    ip::tcp::socket s;
    void do_accept();
};

void run_I2C();

#endif //SERVER_CLASSES_H
