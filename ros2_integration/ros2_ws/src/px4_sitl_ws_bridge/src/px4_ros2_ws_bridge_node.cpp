#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#ifdef PX4_SITL_WS_BRIDGE_COMPONENT_ONLY
#include <rclcpp_components/register_node_macro.hpp>
#endif

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using WsServer = websocketpp::server<websocketpp::config::asio>;
using ConnectionHdl = websocketpp::connection_hdl;

class Px4Ros2WsBridgeNode : public rclcpp::Node {
public:
  explicit Px4Ros2WsBridgeNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
  : Node("px4_ros2_ws_bridge_node", options) {
    system_id_ = this->declare_parameter<int>("system_id", 1);
    ws_host_ = this->declare_parameter<std::string>("ws_host", "0.0.0.0");
    ws_port_ = static_cast<uint16_t>(this->declare_parameter<int>("ws_port", 8765));
    tf_topic_ = this->declare_parameter<std::string>("tf_topic", "/sim/tf");
    gps_fix_topic_ = this->declare_parameter<std::string>("gps_fix_topic", "/sim/gps/fix");
    gps_vel_topic_ = this->declare_parameter<std::string>("gps_vel_topic", "/sim/gps/vel");
    actuators_topic_ = this->declare_parameter<std::string>("actuators_topic", "/sim/actuators");
    aero_topic_ = this->declare_parameter<std::string>("aero_topic", "/sim/aero");
    sysid_topic_ = this->declare_parameter<std::string>("sysid_topic", "/sim/px4_sysid");

    tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      tf_topic_,
      10,
      std::bind(&Px4Ros2WsBridgeNode::on_tf, this, std::placeholders::_1));
    gps_fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_fix_topic_,
      10,
      std::bind(&Px4Ros2WsBridgeNode::on_gps_fix, this, std::placeholders::_1));
    gps_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      gps_vel_topic_,
      10,
      std::bind(&Px4Ros2WsBridgeNode::on_gps_vel, this, std::placeholders::_1));
    actuators_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      actuators_topic_,
      10,
      std::bind(&Px4Ros2WsBridgeNode::on_actuators, this, std::placeholders::_1));
    aero_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      aero_topic_,
      10,
      std::bind(&Px4Ros2WsBridgeNode::on_aero, this, std::placeholders::_1));
    const auto sysid_qos = rclcpp::QoS(1).reliable().durability_volatile();
    sysid_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      sysid_topic_,
      sysid_qos,
      std::bind(&Px4Ros2WsBridgeNode::on_sysid, this, std::placeholders::_1));

    publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&Px4Ros2WsBridgeNode::on_publish_timer, this));

    stats_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&Px4Ros2WsBridgeNode::on_stats_timer, this));

    try {
      this->init_websocket_server();
    } catch (const std::exception &exc) {
      ws_server_available_ = false;
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to start websocket server on ws://%s:%u: %s",
        ws_host_.c_str(),
        ws_port_,
        exc.what());
    }

    RCLCPP_INFO(
      this->get_logger(),
      "WS bridge running: ws://%s:%u (available=%s, topics: %s, %s, %s, %s, %s, %s)",
      ws_host_.c_str(),
      ws_port_,
      ws_server_available_ ? "true" : "false",
      tf_topic_.c_str(),
      gps_fix_topic_.c_str(),
      gps_vel_topic_.c_str(),
      actuators_topic_.c_str(),
      aero_topic_.c_str(),
      sysid_topic_.c_str());
  }

  ~Px4Ros2WsBridgeNode() override {
    {
      std::lock_guard<std::mutex> lock(ws_mutex_);
      websocketpp::lib::error_code ec;
      ws_server_.stop_listening(ec);
      for (const auto &hdl : clients_) {
        ws_server_.close(hdl, websocketpp::close::status::going_away, "shutdown", ec);
      }
      clients_.clear();
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

      if (!latest_payload_.empty()) {
        websocketpp::lib::error_code ec;
        ws_server_.send(hdl, latest_payload_, websocketpp::frame::opcode::text, ec);
        if (ec) {
          clients_.erase(hdl);
        }
      }
    });

    ws_server_.set_close_handler([this](ConnectionHdl hdl) {
      std::lock_guard<std::mutex> lock(ws_mutex_);
      clients_.erase(hdl);
    });

    ws_server_.set_fail_handler([this](ConnectionHdl hdl) {
      std::lock_guard<std::mutex> lock(ws_mutex_);
      clients_.erase(hdl);
    });

    websocketpp::lib::error_code ec;
    websocketpp::lib::asio::ip::address address;
    const std::string listen_host = (ws_host_ == "localhost") ? "127.0.0.1" : ws_host_;
    try {
      address = websocketpp::lib::asio::ip::address::from_string(listen_host);
    } catch (const std::exception &) {
      throw std::runtime_error("Invalid websocket host address: " + ws_host_);
    }
    websocketpp::lib::asio::ip::tcp::endpoint endpoint(address, ws_port_);
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

  static std::string json_escape(const std::string &s) {
    std::ostringstream out;
    for (char c : s) {
      switch (c) {
        case '\\': out << "\\\\"; break;
        case '"': out << "\\\""; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default: out << c; break;
      }
    }
    return out.str();
  }

  template<typename MsgT>
  static int64_t stamp_ns(const MsgT &msg) {
    return (static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL) + msg.header.stamp.nanosec;
  }

  static void append_number_or_null(std::ostringstream &out, double value) {
    if (std::isfinite(value)) {
      out << value;
      return;
    }
    out << "null";
  }

  static std::array<double, 9> quat_wxyz_to_rot(double w, double x, double y, double z) {
    const double ww = w * w;
    const double xx = x * x;
    const double yy = y * y;
    const double zz = z * z;
    const double wx = w * x;
    const double wy = w * y;
    const double wz = w * z;
    const double xy = x * y;
    const double xz = x * z;
    const double yz = y * z;

    return {
      ww + xx - yy - zz, 2.0 * (xy - wz), 2.0 * (xz + wy),
      2.0 * (xy + wz), ww - xx + yy - zz, 2.0 * (yz - wx),
      2.0 * (xz - wy), 2.0 * (yz + wx), ww - xx - yy + zz
    };
  }

  static std::array<double, 4> rot_to_quat_wxyz(const std::array<double, 9> &r) {
    const double trace = r[0] + r[4] + r[8];
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    if (trace > 0.0) {
      const double s = std::sqrt(trace + 1.0) * 2.0;
      w = 0.25 * s;
      x = (r[7] - r[5]) / s;
      y = (r[2] - r[6]) / s;
      z = (r[3] - r[1]) / s;
    } else if (r[0] > r[4] && r[0] > r[8]) {
      const double s = std::sqrt(1.0 + r[0] - r[4] - r[8]) * 2.0;
      w = (r[7] - r[5]) / s;
      x = 0.25 * s;
      y = (r[1] + r[3]) / s;
      z = (r[2] + r[6]) / s;
    } else if (r[4] > r[8]) {
      const double s = std::sqrt(1.0 + r[4] - r[0] - r[8]) * 2.0;
      w = (r[2] - r[6]) / s;
      x = (r[1] + r[3]) / s;
      y = 0.25 * s;
      z = (r[5] + r[7]) / s;
    } else {
      const double s = std::sqrt(1.0 + r[8] - r[0] - r[4]) * 2.0;
      w = (r[3] - r[1]) / s;
      x = (r[2] + r[6]) / s;
      y = (r[5] + r[7]) / s;
      z = 0.25 * s;
    }

    const double qn = std::sqrt((w * w) + (x * x) + (y * y) + (z * z));
    if (qn > 0.0) {
      return {w / qn, x / qn, y / qn, z / qn};
    }
    return {1.0, 0.0, 0.0, 0.0};
  }

  static std::array<double, 4> enu_flu_quat_to_ned_frd(double w, double x, double y, double z) {
    const std::array<double, 9> r_enu_flu = quat_wxyz_to_rot(w, x, y, z);
    constexpr std::array<double, 9> t_enu2ned = {
      0.0, 1.0, 0.0,
      1.0, 0.0, 0.0,
      0.0, 0.0, -1.0
    };
    constexpr std::array<double, 9> t_frd2flu = {
      1.0, 0.0, 0.0,
      0.0, -1.0, 0.0,
      0.0, 0.0, -1.0
    };

    std::array<double, 9> tmp{};
    std::array<double, 9> r_ned_frd{};

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        tmp[(i * 3) + j] =
          (t_frd2flu[(i * 3)] * r_enu_flu[j]) +
          (t_frd2flu[(i * 3) + 1] * r_enu_flu[3 + j]) +
          (t_frd2flu[(i * 3) + 2] * r_enu_flu[6 + j]);
      }
    }

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        r_ned_frd[(i * 3) + j] =
          (tmp[(i * 3)] * t_enu2ned[j]) +
          (tmp[(i * 3) + 1] * t_enu2ned[3 + j]) +
          (tmp[(i * 3) + 2] * t_enu2ned[6 + j]);
      }
    }

    return rot_to_quat_wxyz(r_ned_frd);
  }

  static double heading_deg_from_quat_wxyz(double w, double x, double y, double z) {
    constexpr double kRadToDeg = 57.29577951308232;
    const double yaw_rad = std::atan2(
      2.0 * ((w * z) + (x * y)),
      1.0 - (2.0 * ((y * y) + (z * z))));
    double heading = yaw_rad * kRadToDeg;
    while (heading < 0.0) {
      heading += 360.0;
    }
    while (heading >= 360.0) {
      heading -= 360.0;
    }
    return heading;
  }

  void on_tf(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_tf_ = *msg;
    rx_tf_count_.fetch_add(1, std::memory_order_relaxed);
  }

  void on_gps_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_gps_fix_ = *msg;
    rx_gps_fix_count_.fetch_add(1, std::memory_order_relaxed);
  }

  void on_gps_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_gps_vel_ = *msg;
    rx_gps_vel_count_.fetch_add(1, std::memory_order_relaxed);
  }

  void on_actuators(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    std::array<double, 8> u{};
    const size_t n = std::min<size_t>(msg->data.size(), 8);
    for (size_t i = 0; i < n; ++i) {
      u[i] = static_cast<double>(msg->data[i]);
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_u_ = u;
    rx_actuators_count_.fetch_add(1, std::memory_order_relaxed);
  }

  void on_aero(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_alpha_deg_ = static_cast<double>(msg->vector.x);
    latest_beta_deg_ = static_cast<double>(msg->vector.y);
    rx_aero_count_.fetch_add(1, std::memory_order_relaxed);
  }

  void on_sysid(const std_msgs::msg::UInt8::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_system_id_ = static_cast<int>(msg->data);
    rx_sysid_count_.fetch_add(1, std::memory_order_relaxed);
  }

  void on_publish_timer() {
    std::optional<tf2_msgs::msg::TFMessage> tf_opt;
    std::optional<sensor_msgs::msg::NavSatFix> gps_fix_opt;
    std::optional<std::array<double, 8>> u_opt;
    std::optional<double> alpha_opt;
    std::optional<double> beta_opt;
    std::optional<int> sysid_opt;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      tf_opt = latest_tf_;
      gps_fix_opt = latest_gps_fix_;
      u_opt = latest_u_;
      alpha_opt = latest_alpha_deg_;
      beta_opt = latest_beta_deg_;
      sysid_opt = latest_system_id_;
    }
    const int active_system_id = sysid_opt.value_or(system_id_);

    if (!tf_opt.has_value() || tf_opt->transforms.empty()) {
      std::ostringstream heartbeat;
      heartbeat << "{";
      heartbeat << "\"system_id\":" << active_system_id;
      heartbeat << ",\"time_usec\":" << (this->now().nanoseconds() / 1000LL);
      heartbeat << ",\"position_ned_m\":[null,null,null]";
      heartbeat << ",\"quaternion_wxyz\":[1.0,0.0,0.0,0.0]";
      heartbeat << ",\"lla\":{\"lat_deg\":null,\"lon_deg\":null,\"alt_m\":null}";
      heartbeat << ",\"aero\":{";
      heartbeat << "\"alpha_deg\":";
      append_number_or_null(heartbeat, alpha_opt.value_or(std::numeric_limits<double>::quiet_NaN()));
      heartbeat << ",\"beta_deg\":";
      append_number_or_null(heartbeat, beta_opt.value_or(std::numeric_limits<double>::quiet_NaN()));
      heartbeat << "}";
      heartbeat << ",\"heading_deg\":null";
      heartbeat << ",\"u\":[";
      const std::array<double, 8> heartbeat_u = u_opt.value_or(std::array<double, 8>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
      for (size_t i = 0; i < heartbeat_u.size(); ++i) {
        if (i > 0) {
          heartbeat << ",";
        }
        append_number_or_null(heartbeat, heartbeat_u[i]);
      }
      heartbeat << "]";
      heartbeat << "}";
      latest_payload_ = heartbeat.str();
      broadcast(latest_payload_);
      return;
    }

    const auto &tf = tf_opt->transforms.front();

    std::ostringstream msg;
    msg.precision(12);
    const int64_t tf_time_ns = stamp_ns(tf);
    const int64_t tf_time_usec = tf_time_ns / 1000LL;
    const double n = tf.transform.translation.y;
    const double e = tf.transform.translation.x;
    const double d = -tf.transform.translation.z;
    const auto q_ned_frd = enu_flu_quat_to_ned_frd(
      tf.transform.rotation.w,
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z);
    const double heading_deg = heading_deg_from_quat_wxyz(
      q_ned_frd[0], q_ned_frd[1], q_ned_frd[2], q_ned_frd[3]);

    msg << "{";
    msg << "\"system_id\":" << active_system_id;
    msg << ",\"time_usec\":" << tf_time_usec;
    msg << ",\"position_ned_m\":[";
    append_number_or_null(msg, n);
    msg << ",";
    append_number_or_null(msg, e);
    msg << ",";
    append_number_or_null(msg, d);
    msg << "]";
    msg << ",\"quaternion_wxyz\":[";
    append_number_or_null(msg, q_ned_frd[0]);
    msg << ",";
    append_number_or_null(msg, q_ned_frd[1]);
    msg << ",";
    append_number_or_null(msg, q_ned_frd[2]);
    msg << ",";
    append_number_or_null(msg, q_ned_frd[3]);
    msg << "]";
    msg << ",\"heading_deg\":";
    append_number_or_null(msg, heading_deg);
    msg << ",\"u\":[";
    const std::array<double, 8> frame_u = u_opt.value_or(std::array<double, 8>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    for (size_t i = 0; i < frame_u.size(); ++i) {
      if (i > 0) {
        msg << ",";
      }
      append_number_or_null(msg, frame_u[i]);
    }
    msg << "]";
    msg << ",\"aero\":{";
    msg << "\"alpha_deg\":";
    append_number_or_null(msg, alpha_opt.value_or(std::numeric_limits<double>::quiet_NaN()));
    msg << ",\"beta_deg\":";
    append_number_or_null(msg, beta_opt.value_or(std::numeric_limits<double>::quiet_NaN()));
    msg << "}";

    if (gps_fix_opt.has_value()) {
      const auto &gps = gps_fix_opt.value();
      msg << ",\"lla\":{";
      msg << "\"lat_deg\":";
      append_number_or_null(msg, gps.latitude);
      msg << ",\"lon_deg\":";
      append_number_or_null(msg, gps.longitude);
      msg << ",\"alt_m\":";
      append_number_or_null(msg, gps.altitude);
      msg << "}";
    }

    if (!gps_fix_opt.has_value()) {
      msg << ",\"lla\":{";
      msg << "\"lat_deg\":null,\"lon_deg\":null,\"alt_m\":null";
      msg << "}";
    }

    msg << "}";
    latest_payload_ = msg.str();
    broadcast(latest_payload_);
  }

  void on_stats_timer() {
    size_t client_count = 0;
    {
      std::lock_guard<std::mutex> lock(ws_mutex_);
      client_count = clients_.size();
    }

    RCLCPP_INFO(
      this->get_logger(),
      "bridge stats: clients=%zu rx_tf=%llu rx_gps_fix=%llu rx_gps_vel=%llu rx_act=%llu rx_aero=%llu rx_sysid=%llu tx_ok=%llu tx_fail=%llu",
      client_count,
      static_cast<unsigned long long>(rx_tf_count_.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(rx_gps_fix_count_.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(rx_gps_vel_count_.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(rx_actuators_count_.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(rx_aero_count_.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(rx_sysid_count_.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(tx_ok_count_.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(tx_fail_count_.load(std::memory_order_relaxed)));
  }

  void broadcast(const std::string &payload) {
    if (!ws_server_available_) {
      return;
    }
    std::vector<ConnectionHdl> clients_snapshot;
    {
      std::lock_guard<std::mutex> lock(ws_mutex_);
      if (clients_.empty()) {
        return;
      }
      clients_snapshot.assign(clients_.begin(), clients_.end());
    }

    for (const auto &hdl : clients_snapshot) {
      ws_server_.get_io_service().post([this, hdl, payload]() {
        websocketpp::lib::error_code ec;
        ws_server_.send(hdl, payload, websocketpp::frame::opcode::text, ec);
        if (ec) {
          tx_fail_count_.fetch_add(1, std::memory_order_relaxed);
          std::lock_guard<std::mutex> lock(ws_mutex_);
          clients_.erase(hdl);
          return;
        }
        tx_ok_count_.fetch_add(1, std::memory_order_relaxed);
      });
    }
  }

  std::string ws_host_;
  int system_id_;
  uint16_t ws_port_;
  std::string tf_topic_;
  std::string gps_fix_topic_;
  std::string gps_vel_topic_;
  std::string actuators_topic_;
  std::string aero_topic_;
  std::string sysid_topic_;

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_fix_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr gps_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr actuators_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr aero_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sysid_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  std::mutex data_mutex_;
  std::optional<tf2_msgs::msg::TFMessage> latest_tf_;
  std::optional<sensor_msgs::msg::NavSatFix> latest_gps_fix_;
  std::optional<geometry_msgs::msg::TwistStamped> latest_gps_vel_;
  std::optional<std::array<double, 8>> latest_u_;
  std::optional<double> latest_alpha_deg_;
  std::optional<double> latest_beta_deg_;
  std::optional<int> latest_system_id_;

  std::mutex ws_mutex_;
  WsServer ws_server_;
  std::set<ConnectionHdl, std::owner_less<ConnectionHdl>> clients_;
  std::thread ws_thread_;
  std::string latest_payload_;
  bool ws_server_available_{true};

  std::atomic<uint64_t> rx_tf_count_{0};
  std::atomic<uint64_t> rx_gps_fix_count_{0};
  std::atomic<uint64_t> rx_gps_vel_count_{0};
  std::atomic<uint64_t> rx_actuators_count_{0};
  std::atomic<uint64_t> rx_aero_count_{0};
  std::atomic<uint64_t> rx_sysid_count_{0};
  std::atomic<uint64_t> tx_ok_count_{0};
  std::atomic<uint64_t> tx_fail_count_{0};
};

#ifdef PX4_SITL_WS_BRIDGE_COMPONENT_ONLY
RCLCPP_COMPONENTS_REGISTER_NODE(Px4Ros2WsBridgeNode)
#else
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Px4Ros2WsBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
