#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

extern "C" {
#include <stddef.h>
#include <stdio.h>
#include <jpeglib.h>
}

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <csetjmp>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using WsServer = websocketpp::server<websocketpp::config::asio>;
using ConnectionHdl = websocketpp::connection_hdl;

struct JpegErrorManager {
  jpeg_error_mgr pub;
  jmp_buf setjmp_buffer;
};

extern "C" void jpeg_error_exit(j_common_ptr cinfo) {
  auto *error = reinterpret_cast<JpegErrorManager *>(cinfo->err);
  longjmp(error->setjmp_buffer, 1);
}

class WebsocketFpvBridgeNode : public rclcpp::Node {
public:
  WebsocketFpvBridgeNode()
  : Node("websocket_fpv_bridge_node") {
    host_ = this->declare_parameter<std::string>("host", "127.0.0.1");
    port_ = static_cast<uint16_t>(this->declare_parameter<int>("port", 9001));
    image_topic_ = this->declare_parameter<std::string>("image_topic", "/sim/image");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "camera");

    //auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    //qos.best_effort();
    //qos.durability_volatile();
    //image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic_, qos);

    auto qos = rclcpp::SensorDataQoS().keep_last(1);

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>( image_topic_, qos);


    stats_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&WebsocketFpvBridgeNode::on_stats_timer, this));

    try {
      init_websocket_server();
      ws_server_available_ = true;
    } catch (const std::exception &exc) {
      ws_server_available_ = false;
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to start FPV websocket server on ws://%s:%u: %s",
        host_.c_str(),
        port_,
        exc.what());
    }

    RCLCPP_INFO(
      this->get_logger(),
      "FPV websocket receiver listening on ws://%s:%u -> %s",
      host_.c_str(),
      port_,
      image_topic_.c_str());
  }

  ~WebsocketFpvBridgeNode() override {
    if (!ws_server_available_) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(ws_mutex_);
      websocketpp::lib::error_code ec;
      ws_server_.stop_listening(ec);
      for (const auto &hdl : clients_) {
        ws_server_.close(hdl, websocketpp::close::status::going_away, "shutdown", ec);
      }
      clients_.clear();
      ws_server_.stop();
    }

    if (ws_thread_.joinable()) {
      ws_thread_.join();
    }
  }

private:
  void init_websocket_server() {
    ws_server_.clear_access_channels(websocketpp::log::alevel::all);
    ws_server_.clear_error_channels(websocketpp::log::elevel::all);
    ws_server_.init_asio();
    ws_server_.set_reuse_addr(true);

    ws_server_.set_open_handler([this](ConnectionHdl hdl) {
      std::lock_guard<std::mutex> lock(ws_mutex_);
      clients_.insert(hdl);
      RCLCPP_INFO(this->get_logger(), "FPV client connected (clients=%zu)", clients_.size());
    });

    ws_server_.set_close_handler([this](ConnectionHdl hdl) {
      std::lock_guard<std::mutex> lock(ws_mutex_);
      clients_.erase(hdl);
      RCLCPP_INFO(this->get_logger(), "FPV client disconnected (clients=%zu)", clients_.size());
    });

    ws_server_.set_fail_handler([this](ConnectionHdl hdl) {
      std::lock_guard<std::mutex> lock(ws_mutex_);
      clients_.erase(hdl);
    });

    ws_server_.set_message_handler([this](ConnectionHdl, WsServer::message_ptr msg) {
      if (msg->get_opcode() != websocketpp::frame::opcode::binary) {
        rx_non_binary_count_.fetch_add(1, std::memory_order_relaxed);
        return;
      }

      const std::string &payload = msg->get_payload();
      if (payload.empty()) {
        rx_decode_fail_count_.fetch_add(1, std::memory_order_relaxed);
        return;
      }

      std::vector<uint8_t> bgr;
      int width = 0;
      int height = 0;
      if (!decode_jpeg(
          reinterpret_cast<const uint8_t *>(payload.data()),
          payload.size(),
          width,
          height,
          bgr)) {
        rx_decode_fail_count_.fetch_add(1, std::memory_order_relaxed);
        return;
      }

      publish_image(width, height, bgr);
      rx_frame_count_.fetch_add(1, std::memory_order_relaxed);
    });

    websocketpp::lib::error_code ec;
    websocketpp::lib::asio::ip::address address;
    const std::string listen_host = (host_ == "localhost") ? "127.0.0.1" : host_;
    try {
      address = websocketpp::lib::asio::ip::address::from_string(listen_host);
    } catch (const std::exception &) {
      throw std::runtime_error("Invalid websocket host address: " + host_);
    }

    websocketpp::lib::asio::ip::tcp::endpoint endpoint(address, port_);
    ws_server_.listen(endpoint, ec);
    if (ec) {
      throw std::runtime_error("Failed to listen websocket endpoint");
    }
    ws_server_.start_accept(ec);
    if (ec) {
      throw std::runtime_error("Failed to accept websocket connections");
    }

    ws_thread_ = std::thread([this]() { ws_server_.run(); });
  }

  static bool decode_jpeg(
    const uint8_t *data,
    size_t data_size,
    int &width,
    int &height,
    std::vector<uint8_t> &bgr) {
    if (data == nullptr || data_size == 0) {
      return false;
    }

    jpeg_decompress_struct cinfo;
    JpegErrorManager jerr;
    cinfo.err = jpeg_std_error(&jerr.pub);
    jerr.pub.error_exit = jpeg_error_exit;

    if (setjmp(jerr.setjmp_buffer)) {
      jpeg_destroy_decompress(&cinfo);
      return false;
    }

    jpeg_create_decompress(&cinfo);
    jpeg_mem_src(&cinfo, const_cast<unsigned char *>(data), data_size);
    jpeg_read_header(&cinfo, TRUE);
    cinfo.out_color_space = JCS_RGB;
    jpeg_start_decompress(&cinfo);

    width = static_cast<int>(cinfo.output_width);
    height = static_cast<int>(cinfo.output_height);
    const int channels = static_cast<int>(cinfo.output_components);
    if (width <= 0 || height <= 0 || channels != 3) {
      jpeg_finish_decompress(&cinfo);
      jpeg_destroy_decompress(&cinfo);
      return false;
    }

    const size_t row_stride = static_cast<size_t>(width) * static_cast<size_t>(channels);
    std::vector<uint8_t> rgb_row(row_stride);
    bgr.resize(static_cast<size_t>(width) * static_cast<size_t>(height) * 3U);

    while (cinfo.output_scanline < cinfo.output_height) {
      JSAMPROW row_pointer[1] = {rgb_row.data()};
      jpeg_read_scanlines(&cinfo, row_pointer, 1);
      const size_t y = static_cast<size_t>(cinfo.output_scanline - 1);
      uint8_t *dst = bgr.data() + (y * static_cast<size_t>(width) * 3U);
      for (int x = 0; x < width; ++x) {
        const size_t src_idx = static_cast<size_t>(x) * 3U;
        const size_t dst_idx = src_idx;
        dst[dst_idx] = rgb_row[src_idx + 2];
        dst[dst_idx + 1] = rgb_row[src_idx + 1];
        dst[dst_idx + 2] = rgb_row[src_idx];
      }
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    return true;
  }

  void publish_image(int width, int height, const std::vector<uint8_t> &frame_bgr) {
    if (width <= 0 || height <= 0 || frame_bgr.empty()) {
      return;
    }

    sensor_msgs::msg::Image msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = frame_id_;
    msg.height = static_cast<uint32_t>(height);
    msg.width = static_cast<uint32_t>(width);
    msg.encoding = "bgr8";
    msg.is_bigendian = 0;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(width * 3);
    msg.data = frame_bgr;

    image_pub_->publish(msg);
  }

  void on_stats_timer() {
    size_t client_count = 0;
    {
      std::lock_guard<std::mutex> lock(ws_mutex_);
      client_count = clients_.size();
    }

    RCLCPP_INFO(
      this->get_logger(),
      "fpv stats: clients=%zu rx_frames=%llu decode_fail=%llu non_binary=%llu",
      client_count,
      static_cast<unsigned long long>(rx_frame_count_.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(rx_decode_fail_count_.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(rx_non_binary_count_.load(std::memory_order_relaxed)));
  }

  std::string host_;
  uint16_t port_{9001};
  std::string image_topic_;
  std::string frame_id_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  std::mutex ws_mutex_;
  WsServer ws_server_;
  std::set<ConnectionHdl, std::owner_less<ConnectionHdl>> clients_;
  std::thread ws_thread_;
  bool ws_server_available_{false};

  std::atomic<uint64_t> rx_frame_count_{0};
  std::atomic<uint64_t> rx_decode_fail_count_{0};
  std::atomic<uint64_t> rx_non_binary_count_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WebsocketFpvBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
