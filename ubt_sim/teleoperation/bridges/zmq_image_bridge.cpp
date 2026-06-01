#include <iostream>
#include <string>
#include <cstring>
#include <chrono>

#include <zmq.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

// Minimal JSON parser — extracts integer values for known keys.
// Avoids nlohmann-json system dependency for a trivial {"width":N,"height":N} payload.
namespace tinyjson {
inline int get_int(const std::string& json, const std::string& key) {
    auto pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos) return -1;
    auto colon = json.find(':', pos + key.size() + 2);
    if (colon == std::string::npos) return -1;
    size_t i = colon + 1;
    while (i < json.size() && (json[i] == ' ' || json[i] == '\t' || json[i] == '\n' || json[i] == '\r')) i++;
    int sign = 1;
    if (i < json.size() && json[i] == '-') { sign = -1; i++; }
    int val = 0;
    bool found = false;
    while (i < json.size() && json[i] >= '0' && json[i] <= '9') {
        val = val * 10 + (json[i] - '0');
        i++; found = true;
    }
    return found ? val * sign : -1;
}
} // namespace tinyjson

struct BridgeConfig {
    int zmq_port = 5557;
    std::string rgb_topic = "/sim/camera/color/image_raw";
    std::string depth_topic = "/sim/camera/depth/image_raw";
};

static BridgeConfig parse_args(int argc, char* argv[]) {
    BridgeConfig cfg;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "--zmq-port") && i + 1 < argc) {
            cfg.zmq_port = std::stoi(argv[++i]);
        } else if ((arg == "--rgb-topic") && i + 1 < argc) {
            cfg.rgb_topic = argv[++i];
        } else if ((arg == "--depth-topic") && i + 1 < argc) {
            cfg.depth_topic = argv[++i];
        } else if (arg == "--help") {
            std::cout << "Usage: zmq_image_bridge [OPTIONS]\n"
                      << "Options:\n"
                      << "  --zmq-port PORT       ZMQ image port (default: 5557)\n"
                      << "  --rgb-topic TOPIC     RGB image ROS2 topic (default: /sim/camera/color/image_raw)\n"
                      << "  --depth-topic TOPIC   Depth image ROS2 topic (default: /sim/camera/depth/image_raw)\n";
            exit(0);
        }
    }
    return cfg;
}

class ZmqImageBridge : public rclcpp::Node
{
public:
    ZmqImageBridge(const BridgeConfig& cfg) : Node("zmq_image_bridge")
    {
        // ROS Publishers
        pub_rgb_ = this->create_publisher<sensor_msgs::msg::Image>(cfg.rgb_topic, 10);
        pub_depth_ = this->create_publisher<sensor_msgs::msg::Image>(cfg.depth_topic, 10);

        // ZMQ Configuration
        context_ = zmq::context_t(1);
        subscriber_ = zmq::socket_t(context_, ZMQ_SUB);

        std::string zmq_addr = "tcp://127.0.0.1:" + std::to_string(cfg.zmq_port);
        RCLCPP_INFO(this->get_logger(), "Connecting to ZMQ Image Server at %s", zmq_addr.c_str());
        subscriber_.connect(zmq_addr);

        // Subscribe to all topics (empty filter)
        subscriber_.set(zmq::sockopt::subscribe, "");
        // Only keep latest message in ZMQ buffer to avoid lag
        subscriber_.set(zmq::sockopt::rcvhwm, 2);

        RCLCPP_INFO(this->get_logger(), "C++ ZMQ Image Bridge Started (rgb: %s, depth: %s)",
                    cfg.rgb_topic.c_str(), cfg.depth_topic.c_str());

        // Receive Loop
        receive_thread_ = std::thread(&ZmqImageBridge::receive_loop, this);
    }

    ~ZmqImageBridge()
    {
        running_ = false;
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
    }

private:
    void receive_loop()
    {
        while (rclcpp::ok() && running_) {
            zmq::message_t meta_msg, rgb_msg, depth_msg;

            try {
                // 1. Metadata
                auto res = subscriber_.recv(meta_msg, zmq::recv_flags::none);
                if (!res) continue;

                // 2. RGB Data (Wait for more parts)
                if (!subscriber_.get(zmq::sockopt::rcvmore)) continue;
                (void)subscriber_.recv(rgb_msg, zmq::recv_flags::none);

                // 3. Depth Data (Optional/Wait)
                bool has_depth = false;
                if (subscriber_.get(zmq::sockopt::rcvmore)) {
                    (void)subscriber_.recv(depth_msg, zmq::recv_flags::none);
                    has_depth = true;
                }

                publish_images(meta_msg, rgb_msg, depth_msg, has_depth);

            } catch (const zmq::error_t& e) {
                RCLCPP_ERROR(this->get_logger(), "ZMQ Error: %s", e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }

    void publish_images(zmq::message_t& meta_msg, zmq::message_t& rgb_msg, zmq::message_t& depth_msg, bool has_depth)
    {
        // Parse Metadata
        std::string meta_str(static_cast<char*>(meta_msg.data()), meta_msg.size());
        int w = tinyjson::get_int(meta_str, "width");
        int h = tinyjson::get_int(meta_str, "height");
        if (w <= 0 || h <= 0) {
            RCLCPP_WARN(this->get_logger(), "Invalid metadata, skipping frame");
            return;
        }

        auto current_time = this->now();

        // --- Publish RGB ---
        auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
        img_msg->header.stamp = current_time;
        img_msg->header.frame_id = "camera_link";
        img_msg->height = h;
        img_msg->width = w;
        img_msg->encoding = "rgb8";
        img_msg->step = w * 3;

        img_msg->data.resize(rgb_msg.size());
        std::memcpy(img_msg->data.data(), rgb_msg.data(), rgb_msg.size());

        pub_rgb_->publish(std::move(img_msg));

        // --- Publish Depth ---
        if (has_depth && depth_msg.size() > 0) {
            auto depth_ros_msg = std::make_unique<sensor_msgs::msg::Image>();
            depth_ros_msg->header.stamp = current_time;
            depth_ros_msg->header.frame_id = "camera_link";
            depth_ros_msg->height = h;
            depth_ros_msg->width = w;

            // encoding: 16UC1 (uint16 mm), W * H * 2 bytes
            depth_ros_msg->encoding = "16UC1";
            depth_ros_msg->step = w * 2;

            if (depth_msg.size() == static_cast<size_t>(h * w * 2)) {
                depth_ros_msg->data.resize(depth_msg.size());
                std::memcpy(depth_ros_msg->data.data(), depth_msg.data(), depth_msg.size());
                pub_depth_->publish(std::move(depth_ros_msg));
            }
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rgb_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;

    zmq::context_t context_;
    zmq::socket_t subscriber_;
    std::thread receive_thread_;
    std::atomic<bool> running_{true};
};

int main(int argc, char * argv[])
{
    BridgeConfig cfg = parse_args(argc, argv);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZmqImageBridge>(cfg);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
