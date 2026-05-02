#pragma once

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <nlohmann/json.hpp>
#include <set>
#include <thread>
#include <mutex>

namespace networking {

typedef websocketpp::server<websocketpp::config::asio> server;

class WebSocketPublisher {
public:
    WebSocketPublisher();
    ~WebSocketPublisher();

    void start(uint16_t port);
    void stop();
    void publish(const nlohmann::json& data);

private:
    void on_open(websocketpp::connection_hdl hdl);
    void on_close(websocketpp::connection_hdl hdl);

    server m_server;
    std::thread m_thread;
    std::mutex m_connection_mutex;
    std::set<websocketpp::connection_hdl, std::owner_less<websocketpp::connection_hdl>> m_connections;
};

}
