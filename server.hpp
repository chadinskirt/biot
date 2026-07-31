#ifndef SERVER
#define SERVER
#include <iostream>
#include <boost/asio.hpp>
#include "packet.hpp"
#include "biot.hpp"
namespace asio = boost::asio;
using tcp = asio::ip::tcp;

namespace biot{
    class Receiver : public std::enable_shared_from_this<Receiver>{
        private:
            asio::ip::tcp::socket socket_;
            std::array<uint8_t,256> buffer_;
            std::function<void(packet_t)> handler_;
            BinarySerializer serializer_;
            void read(){
                auto self = shared_from_this();
                asio::async_read(socket_, 
                    asio::buffer(self->buffer_.data(), BinarySerializer::packet_size),
                     [self](boost::system::error_code ec, std::size_t bytes){
                        if(ec)
                            std::cout<< "Accept Fail:"
                                     << ec.message();
                            return;
                        std::cout<< "Read" << bytes << "bytes\n";
                        packet_t packet = self->serializer_.deserialize(self->buffer_.data(), bytes);
                        if(self->handler_)
                            self->handler_(packet);
                        self->read();
                    }
                );
            }
        public:
            Receiver(tcp::socket socket) : socket_(std::move(socket)) {}
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
                        auto receiver = std::make_shared<Receiver>(std::move(socket));
                        receiver->on_packet([](packet_t p){
                            // process workflow
                            std::cout << "Packet receive\n";
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