#include <chrono>
using namespace std::chrono_literals;
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class Lchika : public rclcpp::Node {
    public: 
        explicit Lchika(const std::string & led_cmd) : Node("Lchika"), state_(0) {
            auto timerCallback = [this]()->void {
                
                auto msg = std_msgs::msg::Int32();
                msg.data = state_;
                state_ = 1 - state_;
                RCLCPP_INFO(this->get_logger(), "%d", msg.data);
                pub_->publish(msg);
            };     
            
            rclcpp::QoS qos(rclcpp::KeepLast(10));
            pub_ = create_publisher<std_msgs::msg::Int32>(led_cmd, qos);
            timer_ = create_wall_timer(1000ms, timerCallback);
        }
        // 外部から制御する用のメソッド
        void start_timer() {
            timer_->reset();   // タイマー開始
        }

        void stop_timer() {
            timer_->cancel();  // タイマー停止
        }


    private:
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        int state_;
};

class Togle : public rclcpp::Node {
    public:
        explicit Togle (const std::string & togle_cmd, std::shared_ptr<Lchika> lchika_node)
        : Node("Togle"), lchika_(lchika_node) {
            auto callback = 
                [this](const std_msgs::msg::Int32::UniquePtr msg) -> void {
                    RCLCPP_INFO(this->get_logger(), "%d", msg->data);
                    if (msg->data == 0b0111) {
                        lchika_->stop_timer();
                    } else if (msg->data == 0b1000) {
                        lchika_->start_timer();
                    }
                };
            sub_ = create_subscription<std_msgs::msg::Int32>(togle_cmd, 10, callback);
        }
    
        private:
            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
            std::shared_ptr<Lchika> lchika_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto lchika = std::make_shared<Lchika>("micro_ros_arduino_subscriber");
    auto togle  = std::make_shared<Togle>("togle_topic", lchika);

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(lchika);
    exec.add_node(togle);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
