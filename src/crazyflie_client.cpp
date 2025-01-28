#include "rclcpp/rclcpp.hpp"

#include "crazyflie_interfaces/srv/go_to.hpp"
#include "crazyflie_interfaces/srv/land.hpp"
#include "crazyflie_interfaces/srv/takeoff.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "builtin_interfaces/msg/duration.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "icuas25_msgs/msg/target_info.hpp"

#include <memory>
#include <thread>
#include <cmath>
#include <string>
#include <stdexcept>
#include <vector>

using namespace std::chrono_literals;
                                              
double EPS = 1E-1;

class CrazyflieCommandClient : public rclcpp::Node
{
public:
    CrazyflieCommandClient(int num = 5) : Node("crazyflie_command_client"), 
        num_cf(num), 
        odom_linear(std::vector<geometry_msgs::msg::Point>(num_cf)),
        odom_quat(std::vector<geometry_msgs::msg::Quaternion>(num_cf)),
        pose_subscriptions_(num_cf),
        aruco_subscriptions_(num_cf)
    {
        aruco_cb_options.callback_group = aruco_cb_group_;
        for(int i = 0; i < num_cf; i++){
            pose_subscriptions_[i] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                                            "/cf_" + std::to_string(i+1) + "/pose", 
                                            10, 
                                            [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                                                this->pose_callback(*msg, i + 1);  // to match ROS2 expected fn signature
                                            });

            aruco_subscriptions_[i] = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
                                            "/cf_" + std::to_string(i+1) + "/aruco_markers", 
                                            10,
                                            [this, i](const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
                                                this->aruco_callback(*msg, i + 1);
                                            }, aruco_cb_options);
        }

        res_publisher_ = this->create_publisher<icuas25_msgs::msg::TargetInfo>("target_found", 10);
        aruco_timer_ = this->create_wall_timer(500ms, std::bind(&CrazyflieCommandClient::timer_callback, this), aruco_cb_group_);
        solution_timer_ = this->create_wall_timer(500ms, std::bind(&CrazyflieCommandClient::intermediate_submission, this), solution_cb_group_);
    }
    
    int takeoff(const int &drone_namespace_)
    {
        RCLCPP_INFO(this->get_logger(), "Initialized CrazyflieCommandClient::takeoff for namespace: %d", drone_namespace_);

        auto client = this->create_client<crazyflie_interfaces::srv::Takeoff>("/cf_" + std::to_string(drone_namespace_) + "/takeoff", rmw_qos_profile_services_default, service_cb_group_);
        auto request = std::make_shared<crazyflie_interfaces::srv::Takeoff::Request>();

        request->group_mask = 0;
        request->height = 1.0; 
        request->duration.sec = 2; 
        request->duration.nanosec = 0;

        wait_for_service(client, "/cf_" + std::to_string(drone_namespace_) + "/takeoff");

        auto result = client->async_send_request(request).get();

        return 0;
    }

    int land(const int &drone_namespace_)
    {
        RCLCPP_INFO(this->get_logger(), "Called CrazyflieCommandClient::land for namespace: %d", drone_namespace_);

        auto client = this->create_client<crazyflie_interfaces::srv::Land>("/cf_" + std::to_string(drone_namespace_) + "/land", rmw_qos_profile_services_default, service_cb_group_);
        auto request = std::make_shared<crazyflie_interfaces::srv::Land::Request>();

        request->group_mask = 0;
        request->height = 0.1; 
        request->duration.sec = 2; 
        request->duration.nanosec = 0;

        wait_for_service(client, "/cf_" + std::to_string(drone_namespace_) + "/land");

        auto result = client->async_send_request(request).get();

        return 0;
    }

    int go_to(const int &drone_namespace_, double x, double y, double z, double yaw)
    {
        RCLCPP_INFO(this->get_logger(), "Initialized CrazyflieCommandClient::go_to for namespace: %d", drone_namespace_);

        auto client = this->create_client<crazyflie_interfaces::srv::GoTo>("/cf_" + std::to_string(drone_namespace_) + "/go_to", rmw_qos_profile_services_default, service_cb_group_);
        auto request = std::make_shared<crazyflie_interfaces::srv::GoTo::Request>();

        request->group_mask = 0;
        request->relative = false;
        request->goal.x = x;
        request->goal.y = y;
        request->goal.z = z;
        request->yaw = yaw; 
        request->duration.sec = std::min(20.0, 5 * dist(std::vector<double>{x, y, z}, std::vector<double>{odom_linear[drone_namespace_ - 1].x, odom_linear[drone_namespace_ - 1].y, odom_linear[drone_namespace_ - 1].z})); 
        request->duration.nanosec = 0;

        wait_for_service(client, "/cf_" + std::to_string(drone_namespace_) + "/go_to");

        RCLCPP_INFO(this->get_logger(), "GoTo request sent to %d with goal: [%.2f, %.2f, %.2f]",
                    drone_namespace_, x, y, z);

        auto result = client->async_send_request(request).get();


        int i = drone_namespace_ - 1;
        while(dist(std::vector<double>({x, y, z}), std::vector<double>({odom_linear[i].x, odom_linear[i].y, odom_linear[i].z})) < EPS){
            RCLCPP_DEBUG(this->get_logger(), "%d going to goal: [%.2f, %.2f, %.2f], odom: [%.2f, %.2f, %.2f]",
            drone_namespace_, x, y, z, odom_linear[i].x, odom_linear[i].y, odom_linear[i].z);
        }
        return 0;
    }

    void intermediate_submission(){
        if(!completed){
            /* 
                <pose>1 1 1 1.57 1.57 0</pose>
                <pose>-15 -1 1 1.57 0 0</pose>
            */

            this->takeoff(1);
            rclcpp::sleep_for(std::chrono::seconds(10)); 
            RCLCPP_INFO(this->get_logger(), "Takeoff Completed!");

            this->go_to(1, 1.0, 0.0, 1.0, 1.57);
            rclcpp::sleep_for(std::chrono::seconds(21));
            this->check_for_aruco = true;
            rclcpp::sleep_for(std::chrono::seconds(5));

            this->go_to(1, -15.0, -2.0, 1.0, 1.57);
            rclcpp::sleep_for(std::chrono::seconds(21)); 
            this->check_for_aruco = true;
            rclcpp::sleep_for(std::chrono::seconds(5));

            this->go_to(1, -0.5, -4.0, 1.0, 0.0);
            rclcpp::sleep_for(std::chrono::seconds(20)); 

            this->land(1);
            
            RCLCPP_INFO(this->get_logger(), "Mission Completed!");

            this->completed = true;
        }
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped & msg, const int drone_namespace_){
        int i = drone_namespace_ - 1; //cf_1 -> 0
        odom_linear[i] = geometry_msgs::msg::Point(msg.pose.position);
        odom_quat[i] = geometry_msgs::msg::Quaternion(msg.pose.orientation);
    }

    void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers & msg, const int drone_namespace_){
        int k = drone_namespace_ - 1;

        if (!msg.marker_ids.empty() && check_for_aruco) { 
            for(uint i = 0; i < msg.marker_ids.size(); i++){
                if(dist(prev_aruco_position, {msg.poses[k].position.x, msg.poses[k].position.y, msg.poses[k].position.z}) > 5*EPS){
                    RCLCPP_INFO(this->get_logger(), "AruCo spotted at {%.2f, %.2f, %.2f}", msg.poses[k].position.x, msg.poses[k].position.y, msg.poses[k].position.z);
                    
                    curr_aruco_position = {msg.poses[k].position.x, msg.poses[k].position.y, msg.poses[k].position.z};
                    curr_aruco_id = msg.marker_ids[k];

                    prev_aruco_position = {msg.poses[k].position.x, msg.poses[k].position.y, msg.poses[k].position.z};

                    this->check_for_aruco = false;
                }
            }
        }
    }

    void timer_callback(){
        if(curr_aruco_position[0] != -1e8){
            // RCLCPP_INFO(this->get_logger(), "Publishing to /target_found!");

            auto res = icuas25_msgs::msg::TargetInfo();
            res.id = curr_aruco_id;
            res.location.x = curr_aruco_position[0];
            res.location.y = curr_aruco_position[1];
            res.location.z = curr_aruco_position[2];

            res_publisher_->publish(res);

            curr_aruco_position = {-1e8, -1e8, -1e8};
        }
    }

    template <typename T>
    void wait_for_service(typename std::shared_ptr<rclcpp::Client<T>> client, const std::string &service_name)
    {
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for service %s...", service_name.c_str());
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service. Exiting.");
                throw std::runtime_error("Service interrupted");
            }
        }
    }

    template <typename T>
    T dist(typename std::vector<T> target, typename std::vector<T> odom){
        double res = 0;
        for(uint i = 0; i < target.size(); i++){
            res += pow(target[i] - odom[i], 2);
        }
        return std::sqrt(res);
    }

    int num_cf;
    bool completed = false;
    bool check_for_aruco = false;

    std::vector<geometry_msgs::msg::Point> odom_linear;
    std::vector<geometry_msgs::msg::Quaternion> odom_quat;

    rclcpp::CallbackGroup::SharedPtr solution_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr aruco_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions aruco_cb_options;

    rclcpp::TimerBase::SharedPtr solution_timer_;
    rclcpp::TimerBase::SharedPtr aruco_timer_;

    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subscriptions_;
    std::vector<rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr> aruco_subscriptions_;
    rclcpp::Publisher<icuas25_msgs::msg::TargetInfo>::SharedPtr res_publisher_;

    double curr_aruco_id;
    std::vector<double> curr_aruco_position = {-1e8, -1e8, -1e8};
    std::vector<double> prev_aruco_position = {-1e8, -1e8, -1e8};
};

int main(int argc, char **argv)
{   
    rclcpp::init(argc, argv);

    auto client_node = std::make_shared<CrazyflieCommandClient>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(client_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}