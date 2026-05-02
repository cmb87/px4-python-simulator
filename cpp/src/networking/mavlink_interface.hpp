#pragma once

#include <common/mavlink.h>
#include <asio.hpp>
#include <thread>
#include <vector>
#include <mutex>
#include <functional>

namespace networking {

class MavlinkInterface {
public:
    MavlinkInterface();
    ~MavlinkInterface();

    void listen(const std::string& host, uint16_t port);
    void disconnect();

    void send_message(const mavlink_message_t& msg);
    
    void set_source_system(uint8_t sysid) { m_source_system = sysid; }
    uint8_t get_source_system() const { return m_source_system; }

    bool is_running() const { return m_running; }

    // Callback for received HIL_ACTUATOR_CONTROLS
    void set_on_controls(std::function<void(const mavlink_hil_actuator_controls_t&)> cb) {
        m_on_controls = cb;
    }

    uint8_t get_target_system() const { return m_target_system; }
    uint8_t get_target_component() const { return m_target_component; }
    bool has_target() const { return m_has_target; }

    int32_t get_hil_state_interval_us() const { return m_hil_state_interval_us; }

private:
    void receive_loop();

    asio::io_context m_io_context;
    asio::ip::tcp::acceptor m_acceptor;
    asio::ip::tcp::socket m_socket;
    std::thread m_receive_thread;
    bool m_running = false;

    uint8_t m_source_system = 1;
    uint8_t m_target_system = 0;
    uint8_t m_target_component = 0;
    bool m_has_target = false;
    int32_t m_hil_state_interval_us = -1;

    std::function<void(const mavlink_hil_actuator_controls_t&)> m_on_controls;
};

}
