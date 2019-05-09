#include "servidor.h"
#define BOOST_ASIO_ENABLE_HANDLER_TRACKING
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <iostream>
#include <mutex>

using namespace boost::asio;
using boost::system::error_code;
using namespace boost::asio;
using namespace boost::posix_time;

io_service io;

void run_server() {
  io.run();
}

int main(int argc, char* argv[]) {
  try {
    if (argc != 2) {
      std::cerr << "Usage: server <port>\n"; return 1;
    }
    server s(io, std::atoi(argv[1]));

    std::thread thread_server {run_server};
    std::thread thread_I2C {run_I2C};

    thread_server.join();
    thread_I2C.join();
  }
  catch (std::exception &e) {
    std::cerr << "Exception: " << e.what() << "\n";
  }
}