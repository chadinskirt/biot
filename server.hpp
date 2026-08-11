#ifndef SERVER
#define SERVER
#include <boost/asio.hpp>
#include "packet.hpp"
#include "biot.hpp"
#include "analysis.hpp"
#include "statistic.hpp"
namespace asio = boost::asio;
using tcp = asio::ip::tcp;

namespace biot{
    class Receiver : public std::enable_shared_from_this<Receiver>{
        private:
            asio::ip::tcp::socket socket_;
            std::array<uint8_t,256> buffer_;
            std::function<void(packet_t)> handler_;
            std::queue<std::vector<uint8_t>> tx_queue_;
            bool writing_ = false;
            BinarySerializer serializer_;
            Analyzer analyze_;
            ImpactEngine impact_;
            OrientationEngine orientation_;
            FusionEngine fusion_;
            biot::History time_;
            biot::Slidding_window<biot::packet_t, biot::History> window;
            uint8_t last_flag_ = NONE;
            event_packet_t event;
            void read(){
                // persistent lifetime via global sharing ptr
                auto self = shared_from_this();
                asio::async_read(socket_, 
                    asio::buffer(self->buffer_.data(), BinarySerializer::packet_size),
                     [self](boost::system::error_code ec, std::size_t bytes){
                        if(ec)
                            return;
                        packet_t packet = self->serializer_.deserialize(self->buffer_.data(), bytes);
                        self->analyze_.normalize(packet);
                        if (!self->window.push(packet) || self->window.ready() )
                        {
                            if(self->window.size() == 0){
                                std::cout<< "no available packet to consume";
                            }
                            else{
                                //iterate through window
                                auto view = self->window.view();
                                //compute variance and value
                                auto sensor_f = self->analyze_.extract(view);
                                //statistic decision engine
                                auto impact_belief = self->impact_.evaluate(sensor_f);
                                auto orientation_belief = self->orientation_.evaluate(sensor_f);
                                //fusion output
                                self->fusion_.combine(impact_belief, orientation_belief, packet);
                                // used one of packet_t flag for bool checking to activate async_write
                                if (packet.flag == CRASH_WARNING && !self->last_flag_)
                                {
                                    self->event.seq++;
                                    self->event.flag = packet.flag;

                                    self->send_event(self->event);
                                    self->last_flag_ = true;
                                }
                                //clear window after finish work
                                self->window.clear();
                            }
                        }
                        if(self->handler_)
                            self->handler_(packet);
                        self->read();
                    }
                );
            }
            void write_next(){
                if (writing_ || tx_queue_.empty())
                    return;
                // flag checker
                writing_ = true;

                auto self = shared_from_this();

                asio::async_write(
                    socket_,
                    asio::buffer(tx_queue_.front()),
                    [self](boost::system::error_code ec, std::size_t bytes)
                    {
                        self->writing_ = false;

                        if (ec)
                            return;

                        self->tx_queue_.pop();

                        self->write_next();
                    });
            }
            void send_event(const event_packet_t& event){
                serializer_.serialize(event, tx_queue_);
                write_next();
            }
        public:
            Receiver(tcp::socket socket) : socket_(std::move(socket)), time_(std::chrono::milliseconds{500}) , window(16, time_) {}
            void on_packet(std::function<void(packet_t)> handler){
                handler_ = std::move(handler);
            }
            void start(){
                read();
            }
    };
    class Server{
        private:
            tcp::acceptor accept_;
            void start_accept(){
                accept_.async_accept(
                [this](auto ec, tcp::socket socket)
                {
                    if (!ec){
                        std::cout<< "client connected\n";
                        // passing socket to shared Receiver obj
                        auto receiver = std::make_shared<Receiver>(std::move(socket));
                        // handler for async_read not used it yet
                        receiver->on_packet([](packet_t p){
                            // process workflow
                        });
                        receiver->start();
                    }
                    start_accept();
                }
            );
        };
        public:
            Server(asio::io_context& io) : accept_(io, tcp::endpoint(tcp::v4(),5050)){
                std::cout<< "server started\n";
                start_accept();
            };
    };
};
#endif