#include "mavlink_interface.hpp"
#include <iostream>

namespace networking {

MavlinkInterface::MavlinkInterface() : m_acceptor(m_io_context), m_socket(m_io_context) {}

MavlinkInterface::~MavlinkInterface() {
    disconnect();
}

void MavlinkInterface::listen(const std::string& host, uint16_t port) {
    asio::ip::tcp::endpoint endpoint(asio::ip::make_address(host), port);
    m_acceptor.open(endpoint.protocol());
    m_acceptor.set_option(asio::ip::tcp::acceptor::reuse_address(true));
    m_acceptor.bind(endpoint);
    m_acceptor.listen();

    std::cout << "Waiting for MAVLink connection on " << host << ":" << port << "..." << std::endl;
    m_acceptor.accept(m_socket);
    std::cout << "MAVLink connected!" << std::endl;

    m_running = true;
    m_receive_thread = std::thread(&MavlinkInterface::receive_loop, this);
}

void MavlinkInterface::disconnect() {
    m_running = false;
    if (m_socket.is_open()) {
        m_socket.close();
    }
    if (m_receive_thread.joinable()) {
        m_receive_thread.join();
    }
}

void MavlinkInterface::send_message(const mavlink_message_t& msg) {
    if (!m_socket.is_open()) return;

    try {
        std::vector<uint8_t> buffer(MAVLINK_MAX_PACKET_LEN);
        uint16_t len = mavlink_msg_to_send_buffer(buffer.data(), &msg);
        asio::write(m_socket, asio::buffer(buffer.data(), len));
    } catch (const std::exception& e) {
        if (m_running) {
            std::cerr << "MAVLink send error: " << e.what() << std::endl;
            // Don't kill m_running here, let the main loop decide or reconnect
        }
    }
}

void MavlinkInterface::receive_loop() {
    mavlink_status_t status;
    mavlink_message_t msg;
    uint8_t byte;

    try {
        while (m_running) {
            size_t n = asio::read(m_socket, asio::buffer(&byte, 1));
            if (n > 0) {
                if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
                    if (!m_has_target && msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        m_target_system = msg.sysid;
                        m_target_component = msg.compid;
                        m_has_target = true;
                        std::cout << "Discovered target: SysID " << (int)m_target_system << ", CompID " << (int)m_target_component << std::endl;
                    }

                    if (msg.msgid == MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS) {
                        mavlink_hil_actuator_controls_t controls;
                        mavlink_msg_hil_actuator_controls_decode(&msg, &controls);
                        if (m_on_controls) {
                            m_on_controls(controls);
                        }
                    }

                    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
                        mavlink_command_long_t cmd;
                        mavlink_msg_command_long_decode(&msg, &cmd);
                        if (cmd.command == MAV_CMD_SET_MESSAGE_INTERVAL) {
                            int msg_id = (int)(cmd.param1 + 0.5);
                            if (msg_id == MAVLINK_MSG_ID_HIL_STATE_QUATERNION) {
                                m_hil_state_interval_us = (int)(cmd.param2 + 0.5);
                            }
                        }
                    }
                }
            }
        }
    } catch (const std::exception& e) {
        if (m_running) {
            std::cerr << "MAVLink receive error: " << e.what() << std::endl;
        }
    }
}

}
