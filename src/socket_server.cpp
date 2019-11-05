#include <socket_server.hpp>

SocketServer::SocketServer(int port):m_port(port)
{
    m_hub = std::make_unique<uWS::Hub>();

    // We don't need this since we're not using HTTP but if it's removed the program
    // doesn't compile :-(
    m_hub->onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) 
    {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1)
        {
            res->end(s.data(), s.length());
        }
        else
        {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    m_hub->onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) 
    {
        std::cout << "Connected!!!" << std::endl;
    });

    m_hub->onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) 
    {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

}

SocketServer::~SocketServer()
{
}

void SocketServer::registerOnMessageFunction(
    std::function<void(uWS::WebSocket<uWS::SERVER>, char *, size_t , uWS::OpCode )> on_msg_fn)
{
    m_hub->onMessage(on_msg_fn);
}

int SocketServer::runServer()
{
    if (m_hub->listen(m_port))
    {
        std::cout << "Listening to port " << m_port << std::endl;
        m_hub->run();
        return 0;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
}