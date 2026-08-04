#include <iostream>
#include "packet.hpp"
#include "biot.hpp"
#include "statistic.hpp"
#include "analysis.hpp"
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
    biot::packet_t packet{
        9.9, // accel_mag
        10, // roll
        5, // pitch
        5, // jerk
        16, // veloc
        0 // flag
    };

    biot::packet_t packet1{
        18.0,
        -20,
        10,
        90,
        16,
        0
    };
    biot::packet_t packet2{
        40.0,
        -30,
        -20,
        100,
        16,
        0
    };
    biot::packet_t packet3{
        8.0,
        -30,
        -20,
        100,
        16,
        0
    };
    biot::packet_t packet4{
        2.0,
        -30,
        -20,
        20,
        16,
        0
    };
    biot::feature_t impact_f;
    biot::feature_t orient_f;
    biot::Analyzer analyze_;
    biot::ImpactEngine impact;

    analyze_.normalize(packet);
    analyze_.normalize(packet1);
    analyze_.normalize(packet2);

    /*std::cout
        << packet1.accel_mag << ' '
        << packet1.accel_jerk<< ' '
        << packet1.velocity<< ' '
        << packet1.roll << ' '
        << packet1.pitch << ' '
        << '\n';

    std::vector<uint8_t> bytes = {
        0x01, 0x02, 0x03, 0x04,   // tilt
        0x11, 0x22, 0x33, 0x44,   // accel
        0x55, 0x66, 0x77, 0x88,   // velocity
        0xAA                      // flag
    };
    auto start = std::chrono::steady_clock::now();
    std::vector<uint8_t> bytes = {
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
    };*/
    std::cout << '\n';
    std::cout
    << packet.accel_mag << ' '
    << packet.accel_jerk<< ' '
    << packet.velocity<< ' '
    << packet.roll << ' '
    << packet.pitch << ' '
    << static_cast<int>(packet.flag)
    << '\n';
    
    std::vector<uint8_t> bytes2 = serializer.serialize(packet);
    for ( auto& s: bytes2){
        std::cout << s<< ' ';
    }
    for (;;)
    {
        window.push(packet);
        if (window.ready())
        {
            auto view = window.view();

            impact_f = analyze_.extract(view);
            impact.evaluate(impact_f);
            window.clear();
        }
    }
    return 0;
}