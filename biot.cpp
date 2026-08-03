#include <iostream>
#include <cassert>
#include <span>
#include "biot.hpp"
#include "packet.hpp"
#include <thread>
//namespace asio = boost::asio;
//using tcp = asio::ip::tcp;

void aggregate(const biot::WindowView<biot::packet_t>& view)
{
    float total = 0.0f;

    for (std::size_t i = 0; i < view.size(); ++i)
    {
        total += view[i].velocity;
    }

    std::cout << "Average velocity: "
              << total / view.size()
              << '\n';
}
int main(){
    biot::BinarySerializer serializer;
    biot::History time{std::chrono::milliseconds{500}};
    biot::Slidding_window<biot::packet_t, biot::History> window(16, time);
    biot::packet_t packet;
    std::vector<uint8_t> bytes = {
        0x01, 0x02, 0x03, 0x04,   // tilt
        0x11, 0x22, 0x33, 0x44,   // accel
        0x55, 0x66, 0x77, 0x88,   // velocity
        0xAA                      // flag
    };
    auto start = std::chrono::steady_clock::now();
    /*std::vector<uint8_t> bytes = {
        0x01, 0x02, 0x03, 0x04,   // tilt
        0x11, 0x22, 0x33, 0x44,   // accel
        0x55, 0x66, 0x77, 0x88,   // velocity
        0xAA                      // flag
    };
    std::vector<uint8_t> bytes = {
        0x01, 0x02, 0x03, 0x04,   // tilt
        0x11, 0x22, 0x33, 0x44,   // accel
        0x55, 0x66, 0x77, 0x88,   // velocity
        0xAA                      // flag
    };
    asio::io_context io;
    biot::Server server(io);

    io.run();*/
    
    packet = serializer.deserialize( bytes.data() , serializer.packet_size);
    for ( auto& s: bytes){
        std::cout << s<< ' ';
    }
    std::cout << '\n';
    std::cout
    << packet.accel_mag << ' '
    << packet.accel_jerk<< ' '
    << packet.velocity<< ' '
    << packet.roll << ' '
    << packet.pitch << ' '
    << static_cast<int>(packet.flag)
    << '\n';
    /*
    std::vector<uint8_t> bytes2 = serializer.serialize(packet);
    assert( bytes2 == bytes);
    for ( auto& s: bytes2){
        std::cout << s<< ' ';
    }*/
    for (;;)
    {
        if (window.ready() || !window.push(packet))
        {
            auto trigger = std::chrono::steady_clock::now();
            auto since_start = trigger - start;
            std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(since_start).count() << '\n';
            auto view = window.view();

            aggregate(view);

            window.clear();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return 0;
}