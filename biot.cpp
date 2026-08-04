#include <iostream>
#include <cassert>
#include <span>
#include "biot.hpp"
#include "packet.hpp"
#include "server.hpp"
#include <thread>
namespace asio = boost::asio;
using tcp = asio::ip::tcp;

int main(){
    biot::BinarySerializer serializer;
    biot::History time{std::chrono::milliseconds{500}};
    biot::Slidding_window<biot::packet_t, biot::History> window(16, time);
    biot::packet_t packet;

    
    asio::io_context io;
    biot::Server server(io);

    io.run();
    
    return 0;
}