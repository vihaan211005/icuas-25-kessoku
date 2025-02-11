#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <thread>
#include <future>
#include <vector>
#include <mutex>
#include <queue>
#include <memory>
#include <functional>
#include "utils.hpp"

using namespace std::chrono_literals;

class ComputationNode : public rclcpp::Node {
public:
    ComputationNode() 
        : Node("computation_node")
    {
        timer_ = this->create_wall_timer(500ms, std::bind(&ComputationNode::timer_callback, this), timer_cb_group_);      
        producer_ = this->create_wall_timer(500ms, std::bind(&ComputationNode::producer_callback, this), producer_cb_group_);      
        consumer_ = this->create_wall_timer(500ms, std::bind(&ComputationNode::consumer_callback, this), consumer_cb_group_);      
    }

private:
    void producer_callback(){
        std::vector<double> data = shared_producer_queue_.pop();

        std::cout << "Producer got data: " << getCurrentThreadId().c_str() << " ";
        for(uint i = 0; i < data.size(); i++){
            std::cout << data[i] << " ";
        }
        std::cout << std::endl;

        double res = 0;
        for(uint i = 0; i < data.size(); i++){
            res += data[i];
        }
        shared_consumer_queue_.push(res);
        RCLCPP_INFO(this->get_logger(), "Producer thread [%s] pushed data: %f", getCurrentThreadId().c_str(), res);
        rclcpp::sleep_for(1000ms);
    }

    void consumer_callback(){
        double data = shared_consumer_queue_.pop();
        RCLCPP_INFO(this->get_logger(), "Consumer thread [%s] got data: %f", getCurrentThreadId().c_str(), data);
    }

    void timer_callback(){
        for(int j = 0; j < 10; j++){
            std::vector<double> data;
            for(int i = 0; i < j+1; i++){
                data.push_back(i);
            }
            shared_producer_queue_.push(data);
            RCLCPP_INFO(this->get_logger(), "Timer thread [%s] pushed data of size: %zu", getCurrentThreadId().c_str(), data.size());
            rclcpp::sleep_for(1000ms);
        }
    }

    std::string getCurrentThreadId() {
        std::stringstream ss;
        ss << std::this_thread::get_id();
        return ss.str();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr producer_;
    rclcpp::TimerBase::SharedPtr consumer_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr producer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr consumer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    utils::sharedQueue<std::vector<double>> shared_producer_queue_;
    utils::sharedQueue<double> shared_consumer_queue_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ComputationNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
