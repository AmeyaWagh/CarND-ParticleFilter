#ifndef SOCKET_SERVER_HPP
#define SOCKET_SERVER_HPP

#include <uWS/uWS.h>
#include <functional>
#include <memory>

class SocketServer
{
private:
    std::unique_ptr<uWS::Hub> m_hub;
    int m_port;

public:
    SocketServer(int port);

    ~SocketServer();

    void registerOnMessageFunction(std::function<void(uWS::WebSocket<uWS::SERVER>, char *, size_t , uWS::OpCode )> on_msg_fn);

    int runServer();

};

#endif