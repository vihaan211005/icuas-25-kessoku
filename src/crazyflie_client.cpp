#include "rclcpp/rclcpp.hpp"

#include "crazyflie_interfaces/srv/go_to.hpp"
#include "crazyflie_interfaces/srv/land.hpp"
#include "crazyflie_interfaces/srv/takeoff.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "builtin_interfaces/msg/duration.hpp"

#include <octomap/octomap.h>
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/srv/get_octomap.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "icuas25_msgs/msg/target_info.hpp"

#include <boost/functional/hash.hpp>
#include <boost/thread.hpp>
#include <Eigen/Dense>

#include <memory>
#include <thread>
#include <cmath>
#include <string>
#include <stdexcept>
#include <vector>

#include "traversal.hpp"


//TODO: currently initializing the timer only when done with mission, can we do it from the start with empty stuff being published?
//      stacking error, next_h > curr_h; next_h < curr_h
        
using namespace std::chrono_literals;
                                              
double EPS = 1E-2;

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
        aruco_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options;
        options.callback_group = aruco_cb_group_;
        this->aruco_timer_ = this->create_wall_timer(500ms, std::bind(&CrazyflieCommandClient::timer_callback, this), aruco_cb_group_);
        
        res_publisher_ = this->create_publisher<icuas25_msgs::msg::TargetInfo>("target_found", 10);

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
                                            }, 
                                            options);
        }

        get_octomap();
        RCLCPP_INFO(this->get_logger(), "Initializing Solver object");
        solver = new Solver(Vector3d(0, 0, 0), *tree, 43, 5);

        mutex_ptr = &(solver->param_mutex);
        solution = &(solver->solution);

        RCLCPP_INFO(this->get_logger(), "Starting search");
        solver->initialSetup();
        
        runner_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        compute_timer_ = this->create_wall_timer(500ms, std::bind(&CrazyflieCommandClient::compute_callback, this), runner_cb_group_);
        run_mission_timer_ = this->create_wall_timer(500ms, std::bind(&CrazyflieCommandClient::run_mission, this), runner_cb_group_);
    }
    
    int get_octomap(const std::string &octomap_topic_ = "/octomap_binary"){
        auto octomap_client = this->create_client<octomap_msgs::srv::GetOctomap>(octomap_topic_);
        auto octomap_request = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();

        wait_for_service(octomap_client, octomap_topic_);
        auto result = octomap_client->async_send_request(octomap_request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Service Call Failed!");
            octomap_client->remove_pending_request(result);
            return 1;
        }
        RCLCPP_INFO(this->get_logger(), "Requested Octomap as binary");

        octomap::AbstractOcTree* abstree = octomap_msgs::msgToMap(result.get()->map);
        if (abstree) {
            tree = dynamic_cast<octomap::OcTree*>(abstree);
        } else {
            RCLCPP_ERROR(this->get_logger(), "octomap server did not return a proper tree!");
            return 1;
        }
        return 0;
    }

    int takeoff(const int &drone_namespace_)
    {
        RCLCPP_INFO(this->get_logger(), "Initialized CrazyflieCommandClient::takeoff for namespace: %d", drone_namespace_);

        auto client = this->create_client<crazyflie_interfaces::srv::Takeoff>("/cf_" + std::to_string(drone_namespace_) + "/takeoff");
        auto request = std::make_shared<crazyflie_interfaces::srv::Takeoff::Request>();

        request->group_mask = 0;
        request->height = 1.0; 
        request->duration.sec = 2; 
        request->duration.nanosec = 0;

        wait_for_service(client, "/cf_" + std::to_string(drone_namespace_) + "/takeoff");

        using ServiceResponseFuture = rclcpp::Client<crazyflie_interfaces::srv::Takeoff>::SharedFuture;
        auto takeoffCallback = [&](ServiceResponseFuture future) {
            auto result = future.get();
            RCLCPP_INFO(this->get_logger(), "Takeoff request sent to %d", drone_namespace_);
        };
        auto result = client->async_send_request(request, takeoffCallback);

        return 0;
    }

    int land(const int &drone_namespace_)
    {
        RCLCPP_INFO(this->get_logger(), "Called CrazyflieCommandClient::land for namespace: %d", drone_namespace_);

        auto client = this->create_client<crazyflie_interfaces::srv::Land>("/cf_" + std::to_string(drone_namespace_) + "/land");
        auto request = std::make_shared<crazyflie_interfaces::srv::Land::Request>();

        request->group_mask = 0;
        request->height = 0.1; 
        request->duration.sec = 2; 
        request->duration.nanosec = 0;

        wait_for_service(client, "/cf_" + std::to_string(drone_namespace_) + "/land");

        using ServiceResponseFuture = rclcpp::Client<crazyflie_interfaces::srv::Land>::SharedFuture;
        auto landCallback = [&](ServiceResponseFuture future) {
            auto result = future.get();
            RCLCPP_INFO(this->get_logger(), "Land request sent to %d", drone_namespace_);
        };
        auto result = client->async_send_request(request, landCallback);

        return 0;
    }

    int go_to(const int &drone_namespace_, double x, double y, double z, double yaw)
    {
        RCLCPP_INFO(this->get_logger(), "Initialized CrazyflieCommandClient::go_to for namespace: %d", drone_namespace_);

        auto client = this->create_client<crazyflie_interfaces::srv::GoTo>("/cf_" + std::to_string(drone_namespace_) + "/go_to");
        auto request = std::make_shared<crazyflie_interfaces::srv::GoTo::Request>();

        request->group_mask = 0;
        request->relative = false;
        request->goal.x = x;
        request->goal.y = y;
        request->goal.z = z;
        request->yaw = yaw; 
        request->duration.sec = 2 * dist(vector<double>{x, y, z}, vector<double>{odom_linear[drone_namespace_ - 1].x, odom_linear[drone_namespace_ - 1].y, odom_linear[drone_namespace_ - 1].z}); 
        request->duration.nanosec = 0;

        wait_for_service(client, "/cf_" + std::to_string(drone_namespace_) + "/go_to");

        RCLCPP_INFO(this->get_logger(), "GoTo request sent to %d with goal: [%.2f, %.2f, %.2f]",
                    drone_namespace_, x, y, z);

        using ServiceResponseFuture = rclcpp::Client<crazyflie_interfaces::srv::GoTo>::SharedFuture;
        auto goToCallback = [&](ServiceResponseFuture future) {
            auto result = future.get();
        };
        auto result = client->async_send_request(request, goToCallback);


        int i = drone_namespace_ - 1;
        while(dist(std::vector<double>({x, y, z}), std::vector<double>({odom_linear[i].x, odom_linear[i].y, odom_linear[i].z})) < EPS){
            RCLCPP_DEBUG(this->get_logger(), "%d going to goal: [%.2f, %.2f, %.2f], odom: [%.2f, %.2f, %.2f]",
            drone_namespace_, x, y, z, odom_linear[i].x, odom_linear[i].y, odom_linear[i].z);
        }
        return 0;
    }

    int intermediate_submission(){
        /* 
            <pose>1 1 1 1.57 1.57 0</pose>
            <pose>-15 -1 1 1.57 0 0</pose>
        */

        if(this->takeoff(1)) return 1;
        rclcpp::sleep_for(std::chrono::seconds(10)); 

        if(this->go_to(1, 1.0, 0.0, 1.0, 1.57)) return 1; 
        rclcpp::sleep_for(std::chrono::seconds(20)); 

        if(this->go_to(1, -15.0, -2.0, 1.0, 1.57)) return 1; 
        rclcpp::sleep_for(std::chrono::seconds(20)); 

        if(this->land(1)) return 1;
        
        RCLCPP_INFO(this->get_logger(), "Mission Completed!");

        return 0;
    }

    int run_mission(){
        {
            boost::lock_guard<boost::mutex> lock(*(this->mutex_ptr));
            if(solution->flag == false && solution->eval > 0){
                solution->flag = true;

                RCLCPP_INFO(this->get_logger(), "Got a solution!");

                // vector<Vector3d> startPts; // dont use the startPt[0]; set this as the spawn position
                // vector<vector<pair<Vector3d, int>>> toVisit;
                // vector<vector<int>> toBreak; // visit idx and go back

                //takeoff all 
                // for(int i = 1; i <= 5; i++){
                //     this->takeoff(i);
                //     rclcpp::sleep_for(std::chrono::seconds(5));
                // }
                // rclcpp::sleep_for(std::chrono::seconds(10));

                // go to start point
                double diff_z = 0.2;
                double curr_x = solution->startPts[0].x();
                double curr_y = solution->startPts[0].y();
                double curr_z = 4;
                for(int i = 1; i <= 5; i++){
                    this->go_to(i, curr_x, curr_y, curr_z, 0);
                    rclcpp::sleep_for(std::chrono::seconds(5));
                    curr_z += diff_z;
                }

                rclcpp::sleep_for(std::chrono::seconds(1000));
                // go onward to their respective vantage points
                // for(int i = 1; i < startPts.size(); i++){
                //     diff_h = 0.2;
                //     if(startPts[i].z() <= prev_z){
                //         for(int drone_ = i + 1; drone_ <= 5; drone_++){
                //             this->go_to(drone_, solution->startPts[i].x(), solution->startPts[i].y(), solution->startPts[i].z())
                //             startPts[i].z() += diff_h;
                //         }
                //     }
                //     else{
                //         for(int drone_ = 5; drone_ >= i + 1; drone_--){
                //             this->go_to(drone_, solution->startPts[i].x(), solution->startPts[i].y(), solution->startPts[i].z())
                //             startPts[i].z() -= diff_h;
                //         }
                //     }
                // }

            }
        }
        return 0;
    }

private:
    void compute_callback(){
       RCLCPP_INFO(this->get_logger(), "Computing solution...");
       solver->mainLogic();
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped & msg, const int drone_namespace_){
        int i = drone_namespace_ - 1; //cf_1 -> 0
        odom_linear[i] = geometry_msgs::msg::Point(msg.pose.position);
        odom_quat[i] = geometry_msgs::msg::Quaternion(msg.pose.orientation);
    }

    void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers & msg, const int drone_namespace_){
        int k = drone_namespace_ - 1;

        if (!msg.marker_ids.empty()) { 
            for(uint i = 0; i < msg.marker_ids.size(); i++){
                if(dist(prev_aruco_position, {msg.poses[k].position.x, msg.poses[k].position.y, msg.poses[k].position.z}) > EPS){
                    RCLCPP_INFO(this->get_logger(), "AruCo spotted at {%.2f, %.2f, %.2f}", msg.poses[k].position.x, msg.poses[k].position.y, msg.poses[k].position.z);
                    
                    curr_aruco_position = {msg.poses[k].position.x, msg.poses[k].position.y, msg.poses[k].position.z};
                    curr_aruco_id = msg.marker_ids[k];

                    prev_aruco_position = {msg.poses[k].position.x, msg.poses[k].position.y, msg.poses[k].position.z};
                }
            }
        }
    }

    void timer_callback(){
        if(curr_aruco_position[0] != -1e8){
            RCLCPP_INFO(this->get_logger(), "Publishing to /target_found!");

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

    octomap::OcTree * tree;
    Solver* solver;
    Solution* solution;
    int num_cf;
    boost::mutex *mutex_ptr;

    std::vector<geometry_msgs::msg::Point> odom_linear;
    std::vector<geometry_msgs::msg::Quaternion> odom_quat;

    std::vector<std::thread> thread_block;
    rclcpp::CallbackGroup::SharedPtr aruco_cb_group_;
    rclcpp::CallbackGroup::SharedPtr runner_cb_group_;
    rclcpp::TimerBase::SharedPtr aruco_timer_;
    rclcpp::TimerBase::SharedPtr compute_timer_;
    rclcpp::TimerBase::SharedPtr run_mission_timer_;

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
    rclcpp::spin(std::make_shared<CrazyflieCommandClient>());
    rclcpp::shutdown();
    return 0;
}