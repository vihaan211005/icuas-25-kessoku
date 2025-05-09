#include "rclcpp/rclcpp.hpp"

#include "crazyflie_interfaces/srv/go_to.hpp"
#include "crazyflie_interfaces/srv/land.hpp"
#include "crazyflie_interfaces/srv/takeoff.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "builtin_interfaces/msg/duration.hpp"

#include "crazyflie_interfaces/srv/upload_trajectory.hpp"
#include "crazyflie_interfaces/srv/start_trajectory.hpp"
#include "crazyflie_interfaces/msg/trajectory_polynomial_piece.hpp"

#include "fcl/fcl.h"

#include <octomap/octomap.h>
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/srv/get_octomap.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "icuas25_msgs/msg/target_info.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <boost/functional/hash.hpp>
#include <boost/thread.hpp>
#include <Eigen/Dense>

#include <memory>
#include <thread>
#include <cmath>
#include <string>
#include <stdexcept>
#include <vector>
#include <stack>
#include <deque>
#include <cstdlib>  
#include <fstream>
#include "math.h"

#include "utils.hpp"
#include "planner.hpp"
#include "octomap_utils.hpp"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

#ifndef M_PI_F
#define M_PI_F   (3.14159265358979323846f)
#define M_1_PI_F (0.31830988618379067154f)
#define M_PI_2_F (1.57079632679f)
#endif

using namespace std::chrono_literals;
                                              
double EPS = 0.1;
double ARUCO_EPS = 1.0;
double land_h = 1;
double land_h_0 = 0.03;

class CrazyflieCommandClient : public rclcpp::Node
{
public:
    CrazyflieCommandClient() : 
        Node("crazyflie_command_client"), 
        num_cf(std::getenv("NUM_ROBOTS") ? std::stoi(std::getenv("NUM_ROBOTS")) : 0), 
        range(std::getenv("COMM_RANGE") ? std::stod(std::getenv("COMM_RANGE")) : 0.0),
        drone_h(0.6),
        h_diff(0.4),
        edge_length(0.3),
        start_positions(num_cf),    
        odom_linear(std::vector<geometry_msgs::msg::Point>(num_cf)),
        odom_quat(std::vector<geometry_msgs::msg::Quaternion>(num_cf)),
        odom_linear_vel(std::vector<geometry_msgs::msg::Vector3>(num_cf)),
        odom_angular_vel(std::vector<geometry_msgs::msg::Vector3>(num_cf)),
        pose_subscriptions_(num_cf),
        odom_subscriptions_(num_cf),
        aruco_subscriptions_(num_cf),
        battery_subscriptions_(num_cf),
        battery_status_(num_cf, 100),
        drone_status(num_cf, std::make_pair(true, Eigen::Vector3d())),
        quad_obj(num_cf),
        requestType(1, false, 1, false)
    {
        if(range == 0){
            std::cout << "COMM_RANGE not set!" << std::endl;
            throw std::runtime_error("COMM_RANGE not set!");
        }
        if(num_cf == 0){
            std::cout << "NUM_ROBOTS not set!" << std::endl;
            throw std::runtime_error("NUM_ROBOTS not set!");
        }

        aruco_cb_options.callback_group = aruco_cb_group_;
        battery_cb_options.callback_group = battery_cb_group_;

        for(int i = 0; i < num_cf; i++){
            pose_subscriptions_[i] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                                            "/cf_" + std::to_string(i+1) + "/pose", 
                                            10, 
                                            [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                                                this->pose_callback(*msg, i + 1);  // to match ROS2 expected fn signature
                                            });

            odom_subscriptions_[i] = this->create_subscription<nav_msgs::msg::Odometry>(
                                                "/cf_" + std::to_string(i+1) + "/odom", 
                                                10, 
                                                [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) {
                                                    this->odom_callback(*msg, i + 1);  // to match ROS2 expected fn signature
                                                }); 

            aruco_subscriptions_[i] = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
                                            "/cf_" + std::to_string(i+1) + "/aruco_markers", 
                                            10,
                                            [this, i](const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
                                                this->aruco_callback(*msg, i + 1);
                                            }, aruco_cb_options);

            battery_subscriptions_[i] = this->create_subscription<sensor_msgs::msg::BatteryState>(
                                            "/cf_" + std::to_string(i+1) + "/battery_status", 
                                            10,
                                            [this, i](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
                                                this->battery_callback(*msg, i + 1);
                                            }, battery_cb_options);
        }

        RCLCPP_INFO(this->get_logger(), "Getting Octomap...");
        get_octomap();
        generateOctomapJSON(*tree);
        std::ifstream file("/output.json");
        if (!file) {
            std::cerr << "Could not open octomap JSON file.\n";
            exit(1);
        }

        json config;
        file >> config;
        min_bound = config["min_bound"].get<std::vector<double>>();
        max_bound = config["max_bound"].get<std::vector<double>>(); 
        octomap_resolution = config["resolution"].get<double>();
        octomap_dimensions = config["dimensions"].get<std::vector<double>>();
        size_t n_circles = config["circles"].size();

        std::cout << utils::Color::FG_BLUE << "Number of circles/pillars detected: " << n_circles << utils::Color::FG_DEFAULT << std::endl;

        // init fcl collision geometries
        fcl::OcTree<double>* fcl_tree = new fcl::OcTree<double>(std::shared_ptr<const octomap::OcTree>(tree));
		auto tree_obj = std::shared_ptr<fcl::CollisionGeometry<double>>(fcl_tree);
        tree_collision_object = std::make_shared<fcl::CollisionObject<double>>(tree_obj);

        for(int i = 0; i < num_cf; i++){
            quad_obj[i] = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Box(0.10, 0.10, 0.03));
        }
        for(int i = 0; i < num_cf; i++){
            collision_objects.push_back(fcl::CollisionObject<double> (quad_obj[i]));
        }
        
        //init planner
        RCLCPP_INFO(this->get_logger(), "Initializing Path Planner object...");
        Bounds mapBounds(Eigen::Vector3d(min_bound[0], min_bound[1], min_bound[2] + 0.1), Eigen::Vector3d(max_bound[0], max_bound[1], max_bound[2]));
        planner = std::make_shared<Planner>(Planner(tree, mapBounds, this->get_logger()));

        RCLCPP_INFO(this->get_logger(), "Starting search...");
        res_publisher_ = this->create_publisher<icuas25_msgs::msg::TargetInfo>("target_found", 10);
        run_mission_timer_ = this->create_wall_timer(500ms, std::bind(&CrazyflieCommandClient::run_mission, this), run_mission_cb_group_);
        check_collision_timer_ = this->create_wall_timer(500ms, std::bind(&CrazyflieCommandClient::check_collision, this), check_collision_cb_group_);
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
            tree = dynamic_cast<octomap::OcTree*>(abstree); //TODO:assigning ptr to shared_ptr 
        } else {
            RCLCPP_ERROR(this->get_logger(), "octomap server did not return a proper tree!");
            return 1;
        }
        return 0;
    }

    int takeoff(int drone_namespace_)
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

    int land(int drone)
    {
        RCLCPP_INFO(this->get_logger(), "Called CrazyflieCommandClient::land for namespace: %d", drone);

        auto client = this->create_client<crazyflie_interfaces::srv::Land>("/cf_" + std::to_string(drone) + "/land", rmw_qos_profile_services_default, service_cb_group_);
        auto request = std::make_shared<crazyflie_interfaces::srv::Land::Request>();

        request->group_mask = 0;
        request->height = land_h_0; 
        request->duration.sec = 2; 
        request->duration.nanosec = 0;

        wait_for_service(client, "/cf_" + std::to_string(drone) + "/land");

        auto result = client->async_send_request(request).get();

        return 0;
    }

    double go_to(int drone, double x, double y, double z, double yaw)
    {
        RCLCPP_INFO(this->get_logger(), "Initialized CrazyflieCommandClient::go_to for namespace: %d", drone);
        auto client = this->create_client<crazyflie_interfaces::srv::GoTo>("/cf_" + std::to_string(drone) + "/go_to", rmw_qos_profile_services_default, service_cb_group_);
        auto request = std::make_shared<crazyflie_interfaces::srv::GoTo::Request>();

        double v_max = 1.0;
        double a_max = 1.0;
        request->group_mask = 0;
        request->relative = false;
        request->goal.x = x;
        request->goal.y = y;
        request->goal.z = z;
        request->yaw = yaw; 

        double distance = dist(std::vector<double>{x, y, z}, std::vector<double>{odom_linear[drone - 1].x, odom_linear[drone - 1].y, odom_linear[drone - 1].z}); 
        double tau = 30 * v_max/(16.0 * a_max);
        double c = 30 * v_max /pow(tau, 5);
        double distance_tau = c * pow(tau,6)/60;
        double T;

        if(distance < 2*distance_tau){
            T = pow(84/5.0,0.5)/pow(5.0,0.25);
        }
        else{
            T = 2 * tau + (distance - 2 * distance_tau) / v_max;
        }
        request->duration.sec = static_cast<int32_t>(T);
        request->duration.nanosec = static_cast<uint32_t>((T - request->duration.sec) * 1e9);

        wait_for_service(client, "/cf_" + std::to_string(drone) + "/go_to");

        int i = drone - 1;
        RCLCPP_INFO(this->get_logger(), "GoTo request sent to %d with goal: [%.2f, %.2f, %.2f] yaw: [%.2f], distance: %.2f",
                    drone, x, y, z, yaw, dist(std::vector<double>({x, y, z}), std::vector<double>({odom_linear[i].x, odom_linear[i].y, odom_linear[i].z})));

        auto result = client->async_send_request(request).get();

        /*
        while(dist(std::vector<double>({x, y, z}), std::vector<double>({odom_linear[i].x, odom_linear[i].y, odom_linear[i].z})) < EPS){
            RCLCPP_DEBUG(this->get_logger(), "%d going to goal: [%.2f, %.2f, %.2f], odom: [%.2f, %.2f, %.2f]",
            drone_namespace_, x, y, z, odom_linear[i].x, odom_linear[i].y, odom_linear[i].z);
        }
        */
        drone_status[drone - 1] = std::make_pair(false, Eigen::Vector3d(x, y, z));
        // return 2 * dist(std::vector<double>{x, y, z}, std::vector<double>{odom_linear[drone_namespace_ - 1].x, odom_linear[drone_namespace_ - 1].y, odom_linear[drone_namespace_ - 1].z});
        return T;
    }

    int go_back_using_planner(bool bring_back = false){
        std::cout << utils::Color::FG_RED << "All drones going back to base!" << utils::Color::FG_DEFAULT << std::endl;
        std::vector<std::vector<double>> prev_positions(num_cf);
        for(int i = 0; i < num_cf; i++){
            prev_positions[i] = std::vector<double>({odom_linear[i].x, odom_linear[i].y, odom_linear[i].z});
        }

        for(int i = 0; i < num_cf; i++){
            std::vector<Eigen::Vector4d> pathArray;
            octomap::point3d center(0, 0, 0);
            Eigen::Vector3d start(prev_positions[i][0], prev_positions[i][1], prev_positions[i][2]);
            Eigen::Vector3d goal(start_positions[i][0], start_positions[i][1], start_positions[i][2]);
            run_planner(center, goal, start, pathArray);

            for(uint j = 0; j < pathArray.size(); j++){
                go_to(i, pathArray[j][0], pathArray[j][1], pathArray[j][2], pathArray[j][3]);
                wait_to_reach();
            }
            if(bring_back) rclcpp::sleep_for(std::chrono::milliseconds(2000));
        }

        if(bring_back){
            std::cout << utils::Color::FG_GREEN << "Bringing back all drones back to vertices!" << utils::Color::FG_DEFAULT << std::endl;
            for(int i = 0; i < num_cf; i++){
                std::vector<Eigen::Vector4d> pathArray;
                octomap::point3d center(0, 0, 0);
                Eigen::Vector3d start(start_positions[i][0], start_positions[i][1], start_positions[i][2]);
                Eigen::Vector3d goal(prev_positions[i][0], prev_positions[i][1], prev_positions[i][2]);
                run_planner(center, goal, start, pathArray);

                for(uint j = 0; j < pathArray.size(); j++){
                    go_to(i, pathArray[j][0], pathArray[j][1], pathArray[j][2], pathArray[j][3]);
                    wait_to_reach();
                }
            }
        }
        return 0;
    }

    int run_mission(){
        if(!flag){
            for(int i = 0; i < num_cf; i++){
                start_positions[i] = std::vector<double>({odom_linear[i].x, odom_linear[i].y, odom_linear[i].z});
            }

            // generate path
            std::ostringstream oss;
            oss << "python $PATH_GEN " << edge_length << " /output.json";
            auto res = std::system(oss.str().c_str()); // "Usage: python main.py <EDGE> <OCTOMAP.JSON>"
            if(res){
                std::cerr << "Failed to run pysim program" << std::endl;
                exit(1);
            }

            std::ifstream file("/solution.json");
            if (!file) {
                std::cerr << "Could not open solution JSON file.\n";
                return 1;
            }
                
            // start the mission
            mission_started = true;
            json j;
            file >> j;

            auto poses = j["poses"].get<std::vector<std::vector<std::array<int, 2>>>>();
            auto yaws = j["yaws"].get<std::vector<std::vector<std::vector<double>>>>();
            auto matrix = j["matrix"].get<std::vector<std::vector<std::vector<std::vector<bool>>>>>();
            
            for (uint goal_idx = 0; goal_idx < poses.size(); goal_idx++){

                if(recharge_flag){
                    go_back_using_planner(true);
                }
                
                std::vector<int> scan_drones;
                uint max_n_buildings = 0;

                /*go to next state*/
                for(int drone_idx = 0; drone_idx < num_cf; drone_idx++){
                    auto pose = poses[goal_idx][drone_idx];
                    auto yaw = yaws[goal_idx][drone_idx]; 
                    if(yaw.empty()){ // no building found
                        if(!yaw.empty()) {
                            max_n_buildings = yaw.size() > max_n_buildings? yaw.size() : max_n_buildings;
                            scan_drones.push_back(drone_idx);
                        }
                        go_to(drone_idx + 1, coord_x(pose[0]), coord_y(pose[1]), drone_h, 0);
                    } 

                }
                wait_to_reach();

                /*scan buildings*/
                for(uint k = 0; k < max_n_buildings; k++){
                    for(int drone : scan_drones){
                        if(k < yaws[goal_idx][drone].size())
                            go_to(drone + 1, coord_x(poses[goal_idx][drone][0]), coord_y(poses[goal_idx][drone][1]), drone_h - h_diff, yaws[goal_idx][drone][k]);
                    }
                    wait_to_reach();
    
                    for(int drone : scan_drones){
                        if(k < yaws[goal_idx][drone].size())
                            go_to(drone + 1, coord_x(poses[goal_idx][drone][0]), coord_y(poses[goal_idx][drone][1]), drone_h + h_diff, yaws[goal_idx][drone][k]);
                    }
                    wait_to_reach();
    
                    for(int drone : scan_drones){
                        if(k < yaws[goal_idx][drone].size())
                            go_to(drone + 1, coord_x(poses[goal_idx][drone][0]), coord_y(poses[goal_idx][drone][1]), drone_h, yaws[goal_idx][drone][k]);
                    }
                    wait_to_reach();
                }
            }

            std::cout << utils::Color::FG_GREEN << "Mission ended! Recalling all drones going back to base!" << utils::Color::FG_DEFAULT << std::endl;
            go_back_using_planner(false);
            
            flag = true;
        }
        return 0;
    }

private:
    void check_collision() {
        for(int i = 0; i < num_cf; i++){
            fcl::Vector3d translation(odom_linear[i].x, odom_linear[i].y, odom_linear[i].z);
            fcl::Quaterniond rotation(odom_quat[i].w, odom_quat[i].x, odom_quat[i].y, odom_quat[i].z);
            collision_objects[i].setTransform(rotation, translation);
        }

        for(int i = 0; i < num_cf; i++){
            for(int j = i + 1; j < num_cf; j++){
                fcl::CollisionResult<double> collisionResult;
                fcl::collide(&collision_objects[i], &collision_objects[j], requestType, collisionResult);
                if (collisionResult.isCollision()){
                    std::cout << utils::Color::FG_RED << "Collision happened between " << i + 1 << " and " << j + 1 << utils::Color::FG_DEFAULT << std::endl;
                }
            }
        }

        for(int i = 0; i < num_cf; i++){
            if(odom_linear[i].z < 0.5) continue;
            fcl::CollisionResult<double> collisionResult;
            fcl::collide(&collision_objects[i], &(*tree_collision_object), requestType, collisionResult);
            if (collisionResult.isCollision()){
                std::cout << utils::Color::FG_RED << "Collision happened between " << i + 1 << " and building" << utils::Color::FG_DEFAULT << std::endl;
            }
        }
    }

    double coord_x(double x){
        return min_bound[0] + edge_length*x;
    }

    double coord_y(double y){
        return min_bound[1] + edge_length*y;
    }

    double get_yaw(int drone){
        return atan2(2*(odom_quat[drone].w*odom_quat[drone].z + odom_quat[drone].x*odom_quat[drone].y), 1 - 2*(odom_quat[drone].y*odom_quat[drone].y + odom_quat[drone].z*odom_quat[drone].z));
    }

    float fmodf_floored(float x, float n){
        return x - floorf(x / n) * n;
    }

    float shortest_signed_angle_radians(float start, float goal) {
        float diff = goal - start;
        float signed_diff = fmodf_floored(diff + M_PI_F, 2 * M_PI_F) - M_PI_F;
        return signed_diff;
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped & msg, const int drone_namespace_){
        int i = drone_namespace_ - 1; //cf_1 -> 0
        odom_linear[i] = geometry_msgs::msg::Point(msg.pose.position);
        odom_quat[i] = geometry_msgs::msg::Quaternion(msg.pose.orientation);
    }

    void odom_callback(const nav_msgs::msg::Odometry & msg, const int drone_namespace_){
        int i = drone_namespace_ - 1; //cf_1 -> 0
        odom_linear_vel[i] = geometry_msgs::msg::Vector3(msg.twist.twist.linear);
        odom_angular_vel[i] = geometry_msgs::msg::Vector3(msg.twist.twist.angular);
    }

    void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers & msg, const int drone_namespace_){
        int k = drone_namespace_ - 1;

        if (!msg.marker_ids.empty()) { 
            for(uint i = 0; i < msg.marker_ids.size(); i++){
                double x = msg.poses[i].position.x;
                double y = msg.poses[i].position.y;
                double z = msg.poses[i].position.z;

                if(check_distance(prev_aruco_positions, {x, y, z}, ARUCO_EPS)){
                    RCLCPP_INFO(this->get_logger(), "AruCo spotted at {%.2f, %.2f, %.2f}", x, y, z);
                    
                    curr_aruco_position = {x, y, z};
                    curr_aruco_id = msg.marker_ids[i];
                    
                    RCLCPP_INFO(this->get_logger(), "Publishing to /target_found!");
                    auto res = icuas25_msgs::msg::TargetInfo();

                    res.id = curr_aruco_id;
                    
                    Eigen::Quaterniond quaternion(odom_quat[k].w, odom_quat[k].x, odom_quat[k].y, odom_quat[k].z); 
                    quaternion.normalize();
                    
                    Eigen::Vector3d point(curr_aruco_position[2], -curr_aruco_position[0], -curr_aruco_position[1]);
                    
                    Eigen::Vector3d transformed_point = quaternion * point;
                    res.location.x = odom_linear[k].x + transformed_point.x();
                    res.location.y = odom_linear[k].y + transformed_point.y();
                    res.location.z = odom_linear[k].z + transformed_point.z();
                    
                    res_publisher_->publish(res);

                    prev_aruco_positions.push_back(curr_aruco_position);
                }
            }
        }
    }

    void battery_callback(const sensor_msgs::msg::BatteryState & msg, const int drone_namespace_){
        if(mission_started){
            int k = drone_namespace_ - 1;
    
            battery_status_[k] = msg.percentage;
            // std::cout << "charge of " << drone_namespace_ << " " << msg.percentage << std::endl;
            double curr_min = 100.0;
            int curr_min_k = -1;
    
            for(int k = 0; k < battery_status_.size(); k++){
                if(dist(std::vector<double>({odom_linear[k].x, odom_linear[k].y, odom_linear[k].z}), start_positions[k]) < 10 * EPS){
                    continue;
                }
                curr_min = std::min(curr_min, battery_status_[k]);
                curr_min_k = k + 1;
            }
            min_charge = curr_min;
            min_drone = curr_min_k;
    
            if(min_charge < 30){
                // std::cout << "Setting recharge flag to TRUE, min_charge: " << min_charge << std::endl;
                recharge_flag = true;
            }
            else{
                // std::cout << "Recharge flag not set" << std::endl;
            }
        }
    }

    bool check(){
        for(int i = 0; i < num_cf; ++i){
            double x = drone_status[i].second.x();
            double y = drone_status[i].second.y();
            double z = drone_status[i].second.z();

            if(drone_status[i].first == false && dist(std::vector<double>({x, y, z}), std::vector<double>({odom_linear[i].x, odom_linear[i].y, odom_linear[i].z})) < EPS){
                drone_status[i].first = true; // reached at destination
                if(drone_status[i].second.x() == start_positions[i][0] && drone_status[i].second.y() == start_positions[i][1]){
                    land(i+1);
                }
            }
            if(drone_status[i].first == false){
                std::cout << "Going to goal: [" << i + 1 << "] " << x << "," << y << "," << z << " odom: " << odom_linear[i].x << "," << odom_linear[i].y << "," << odom_linear[i].z << std::endl;
                return true;
            }
        }
        return false;
    }

    void wait_to_reach(){
        auto start_time = std::chrono::steady_clock::now();
        while(check()){
            if (std::chrono::steady_clock::now() - start_time > std::chrono::minutes(15)) {
                std::cout << utils::Color::FG_RED << "Collision happened possibly, exiting..." << utils::Color::FG_DEFAULT << std::endl;
                exit(1);
            }
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
    }

    bool check_distance(const std::vector<std::vector<double>>& prev_aruco_positions, const std::vector<double>& new_point, double ARUCO_EPS) {
        for (const auto& prev_point : prev_aruco_positions) {
            if (dist(prev_point, new_point) <= ARUCO_EPS) {
                return false; // Found a point where the distance is too small, return false
            }
        }
        return true; // All distances are greater than ARUCO_EPS
    }
    
    void run_planner(octomap::point3d center, Eigen::Vector3d goal, Eigen::Vector3d start, std::vector<Eigen::Vector4d>& pathArray)
    {
        planner->setCenter(center);

        std::vector<Eigen::Vector3d> start_points;
        for(int i = 0; i < num_cf; i++){
            start_points.push_back(Eigen::Vector3d(odom_linear[i].x, odom_linear[i].y, odom_linear[i].z));
        }
        planner->setPosition(start_points);

        Eigen::Vector4d start_(start[0], start[1], start[2], 0.0);
        Eigen::Vector4d goal_(goal[0], goal[1], goal[2], 0.0);

        planner->runPlanner(start_, goal_, pathArray);
    }

    bool inLoS(Eigen::Vector3d A, Eigen::Vector3d B) const {
        octomap::point3d prev(A.x(), A.y(), A.z());
        octomap::point3d curr(B.x(), B.y(), B.z());
        octomap::point3d hit;

        bool collision = octree_->castRay(prev, curr - prev, hit, true, (curr - prev).norm() - 0.01);
        if(collision){
            return false;
        }
        return true;
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

    octomap::OcTree* tree;

    int num_cf;
    bool flag = false;
    bool mission_started = false;
    double range;
    double drone_h;
    double h_diff;
    double edge_length;
    double octomap_resolution;
    std::vector<double> min_bound;
    std::vector<double> max_bound;
    std::vector<double> octomap_dimensions;
    std::shared_ptr<Planner> planner;

    Eigen::Vector4d start;
    Eigen::Vector4d goal;
    std::vector<std::vector<double>> start_positions;

    std::vector<geometry_msgs::msg::Point> odom_linear;
    std::vector<geometry_msgs::msg::Quaternion> odom_quat;
    std::vector<geometry_msgs::msg::Vector3> odom_linear_vel;
    std::vector<geometry_msgs::msg::Vector3> odom_angular_vel;

    rclcpp::CallbackGroup::SharedPtr service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr aruco_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr battery_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr run_mission_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr check_collision_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    rclcpp::SubscriptionOptions aruco_cb_options;
    rclcpp::SubscriptionOptions battery_cb_options;

    rclcpp::TimerBase::SharedPtr aruco_timer_;
    rclcpp::TimerBase::SharedPtr run_mission_timer_;
    rclcpp::TimerBase::SharedPtr check_collision_timer_;

    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subscriptions_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subscriptions_;
    std::vector<rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr> aruco_subscriptions_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr> battery_subscriptions_;
    rclcpp::Publisher<icuas25_msgs::msg::TargetInfo>::SharedPtr res_publisher_;

    double curr_aruco_id;
    std::vector<double> curr_aruco_position = {-1e8, -1e8, -1e8};
    std::vector<std::vector<double>> prev_aruco_positions = {{-1e8, -1e8, -1e8}};

    std::vector<double> battery_status_;
    bool recharge_flag;
    std::map<int,std::deque<int>> mp;
    double min_charge = 100;
    int min_drone = -1;
    std::vector<std::pair<bool, Eigen::Vector3d>> drone_status;

    std::vector<std::shared_ptr<fcl::CollisionGeometry<double>>> quad_obj;
    std::shared_ptr<fcl::CollisionObject<double>> tree_collision_object;
    std::vector<fcl::CollisionObject<double>> collision_objects;
    fcl::CollisionRequest<double> requestType;
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
