#include "websocket_publisher.hpp"

namespace networking {

WebSocketPublisher::WebSocketPublisher() {
    m_server.init_asio();
    m_server.set_open_handler(std::bind(&WebSocketPublisher::on_open, this, std::placeholders::_1));
    m_server.set_close_handler(std::bind(&WebSocketPublisher::on_close, this, std::placeholders::_1));
    m_server.set_access_channels(websocketpp::log::alevel::none);
    m_server.set_error_channels(websocketpp::log::elevel::none);
}

WebSocketPublisher::~WebSocketPublisher() {
    stop();
}

void WebSocketPublisher::start(uint16_t port) {
    m_server.listen(port);
    m_server.start_accept();
    m_thread = std::thread([this]() {
        m_server.run();
    });
}

void WebSocketPublisher::stop() {
    m_server.stop_listening();
    {
        std::lock_guard<std::mutex> lock(m_connection_mutex);
        for (auto hdl : m_connections) {
            m_server.close(hdl, websocketpp::close::status::normal, "Server stopping");
        }
    }
    m_server.stop();
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

void WebSocketPublisher::publish(const nlohmann::json& data) {
    std::string msg = data.dump();
    std::lock_guard<std::mutex> lock(m_connection_mutex);
    for (auto hdl : m_connections) {
        m_server.send(hdl, msg, websocketpp::frame::opcode::text);
    }
}

void WebSocketPublisher::on_open(websocketpp::connection_hdl hdl) {
    std::lock_guard<std::mutex> lock(m_connection_mutex);
    m_connections.insert(hdl);
}

void WebSocketPublisher::on_close(websocketpp::connection_hdl hdl) {
    std::lock_guard<std::mutex> lock(m_connection_mutex);
    m_connections.erase(hdl);
}

}
