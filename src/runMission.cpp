#include "rclcpp/rclcpp.hpp"
#include "crazyflie_interfaces/srv/go_to.hpp"
#include "crazyflie_interfaces/srv/land.hpp"
#include "crazyflie_interfaces/srv/takeoff.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "builtin_interfaces/msg/duration.hpp"

#include <memory>
#include <string>
#include <stdexcept>

class CrazyflieCommandClient : public rclcpp::Node
{
public:
    CrazyflieCommandClient(const std::string &drone_namespace)
        : Node("crazyflie_command_client"), drone_namespace_(drone_namespace)
    {
        RCLCPP_INFO(this->get_logger(), "Initialized CrazyflieCommandClient for namespace: %s", drone_namespace_.c_str());
        subscription_ = this->create_subscription<std_msgs::msg::String>(drone_namespace_ + "/pose", 10, std::bind(&CrazyflieCommandClient::pose_callback, this, _1));

    }

    void takeoff()
    {
        auto client = this->create_client<crazyflie_interfaces::srv::Takeoff>(drone_namespace_ + "/takeoff");
        auto request = std::make_shared<crazyflie_interfaces::srv::Takeoff::Request>();

        request->group_mask = 0;
        request->height = 1.0; // Hardcoded height
        request->duration.sec = 2; // Hardcoded duration
        request->duration.nanosec = 0;

        wait_for_service(client, drone_namespace_ + "/takeoff");

        auto result = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Service Call Failed!");
            client->remove_pending_request(result);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Takeoff request sent to %s", drone_namespace_.c_str());
    }

    void land()
    {
        auto client = this->create_client<crazyflie_interfaces::srv::Land>(drone_namespace_ + "/land");
        auto request = std::make_shared<crazyflie_interfaces::srv::Land::Request>();

        request->group_mask = 0;
        request->height = 0.1; // Hardcoded landing height
        request->duration.sec = 2; // Hardcoded duration
        request->duration.nanosec = 0;

        wait_for_service(client, drone_namespace_ + "/land");

        auto result = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Service Call Failed!");
            client->remove_pending_request(result);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Land request sent to %s", drone_namespace_.c_str());
    }

    void go_to(float x, float y, float z)
    {
        auto client = this->create_client<crazyflie_interfaces::srv::GoTo>(drone_namespace_ + "/go_to");
        auto request = std::make_shared<crazyflie_interfaces::srv::GoTo::Request>();

        request->group_mask = 0;
        request->relative = false;
        request->goal.x = x;
        request->goal.y = y;
        request->goal.z = z;
        request->yaw = 0.0; // Hardcoded yaw
        request->duration.sec = 4; // Hardcoded duration
        request->duration.nanosec = 0;

        wait_for_service(client, drone_namespace_ + "/go_to");

        auto result = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Service Call Failed!");
            client->remove_pending_request(result);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "GoTo request sent to %s with goal: [%.2f, %.2f, %.2f]",
                    drone_namespace_.c_str(), x, y, z);
    }

private:

    void topic_callback(const geometry_msgs::msg::PoseStamped & msg) const
    {
        this->drone_x = pose.position.x;
        this->drone_y = pose.position.y;
        this->drone_z = pose.position.z;
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

    std::string drone_namespace_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    float64 drone_x;
    float64 drone_y;
    float64 drone_z;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<CrazyflieCommandClient>("/cf_1");
    try
    {
        // Hardcoded commands
        client->takeoff();
        rclcpp::sleep_for(std::chrono::seconds(15)); // Wait for takeoff to complete

        client->go_to(2.0, 3.0, 1.5); // Example coordinates
        rclcpp::sleep_for(std::chrono::seconds(15)); // Wait for GoTo to complete

        client->land();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(client->get_logger(), "Error: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
