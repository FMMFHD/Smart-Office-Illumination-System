#include "servidor.h"
#include "classes.h"
#include "I2C.h"
#include <string.h>
#include <algorithm>
#include <iostream>
#include <mutex>
#include <stdlib.h>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#define period 9.9

using namespace boost::asio;
using boost::system::error_code;
using namespace boost::asio;
using namespace boost::posix_time;

nodes all_nodes;

extern io_service io;

steady_timer tim(io);
extern std::mutex all_nodes_mutex;

session::session(ip::tcp::socket socket) : s(std::move(socket)) {
  read_buffer4 = 0;
  read_buffer5 = 0;
}

void session::start() {
  tim.expires_from_now(milliseconds(8));
  tim.async_wait(&do_stream);
  do_read();
}

void session::do_read() {
  auto self(shared_from_this());
  data_[0] = '\0'; // TALVEZ
  s.async_read_some(buffer(data_, max_length),
  [this, self](error_code ec, size_t length) {
    if (!ec) {
      do_write(length);
    }
  });
}

void session::do_write(size_t length) {
  auto self(shared_from_this());
  all_nodes_mutex.lock();
  select_function(data_, &length, &all_nodes, this);
  all_nodes_mutex.unlock();
  async_write(s, buffer(data_, length),
  [this, self](error_code ec, size_t) {
    if (!ec) {
      do_read();
    }
  });
}

void session::do_stream(const boost::system::error_code &ec) {
  char data[30];
  char aux[10];
  auto self(shared_from_this());

  if(stream = 1) {
    strcpy(data, "s l 4 ");
    all_nodes_mutex.lock();

    while (read_buffer4 != all_nodes.node_i[0].get_iterations()) {
      sprintf(aux, "%0.2f %0.2f ", all_nodes.node_i[0].get_lux_i(read_buffer4), all_nodes.node_i[0].get_iterations()*period);
      strcat(data, aux);
      read_buffer4 ++;
    }
    strcat(data,"\n");
    all_nodes_mutex.unlock();
    async_write(s, buffer(data, strlen(data)),[this, self](error_code ec, size_t) {});
  }
  else if(stream = 2) {
    strcpy(data, "s d 4 ");
    all_nodes_mutex.lock();

    while (read_buffer4 != all_nodes.node_i[0].get_iterations()) {
      sprintf(aux, "%d %0.2f ", all_nodes.node_i[0].get_pwm_i(read_buffer4), all_nodes.node_i[0].get_iterations()*period);
      strcat(data, aux);
      read_buffer4 ++;
    }
    strcat(data,"\n");
    all_nodes_mutex.unlock();
    async_write(s, buffer(data, strlen(data)),[this, self](error_code ec, size_t) {});
  }
  else if(stream = 3) {
    strcpy(data, "s l 5 ");
    all_nodes_mutex.lock();

    while (read_buffer5 != all_nodes.node_i[1].get_iterations()) {
      sprintf(aux, "%0.2f %0.2f ", all_nodes.node_i[1].get_lux_i(read_buffer5), all_nodes.node_i[1].get_iterations()*period);
      strcat(data, aux);
      read_buffer5 ++;
    }
    strcat(data,"\n");
    all_nodes_mutex.unlock();
    async_write(s, buffer(data, strlen(data)),[this, self](error_code ec, size_t) {});
  }
  else if(stream = 4) {
    strcpy(data, "s d 5 ");
    all_nodes_mutex.lock();

    while (read_buffer5 != all_nodes.node_i[1].get_iterations()) {
      sprintf(aux, "%d %0.2f ", all_nodes.node_i[1].get_pwm_i(read_buffer5), all_nodes.node_i[1].get_iterations()*period);
      strcat(data, aux);
      read_buffer5 ++;
    }
    strcat(data,"\n");
    all_nodes_mutex.unlock();
    async_write(s, buffer(data, strlen(data)),[this, self](error_code ec, size_t) {});
  }
  tim.expires_from_now(milliseconds(8));
  tim.async_wait(&do_stream);
}

server::server(io_service& io_service, short port)
: s(io_service) ,
    acceptor_(io_service, ip::tcp::endpoint(ip::tcp::v4(), port)) {
  do_accept();
}

void server::do_accept() {
  acceptor_.async_accept(s,
  [this](error_code ec) {
    if (!ec) {
      std::make_shared<session>(std::move(s))->start();
    }
    do_accept();
  });
}

void run_I2C() {
  I2C(&all_nodes);
}
