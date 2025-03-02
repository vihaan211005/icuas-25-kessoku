#include "rclcpp/rclcpp.hpp"

#include "crazyflie_interfaces/srv/go_to.hpp"
#include "crazyflie_interfaces/srv/land.hpp"
#include "crazyflie_interfaces/srv/takeoff.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "builtin_interfaces/msg/duration.hpp"

#include "crazyflie_interfaces/srv/upload_trajectory.hpp"
#include "crazyflie_interfaces/srv/start_trajectory.hpp"
#include "crazyflie_interfaces/msg/trajectory_polynomial_piece.hpp"

#include <octomap/octomap.h>
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/srv/get_octomap.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "icuas25_msgs/msg/target_info.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"

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

#include "traversal.hpp"
#include "viz_tree.hpp"
#include "utils.hpp"

using namespace std::chrono_literals;
                                              
double EPS = 0.1;
double land_h = 1;
double land_h_0 = 0.03;

class CrazyflieCommandClient : public rclcpp::Node
{
public:
    CrazyflieCommandClient() : 
        Node("crazyflie_command_client"), 
        num_cf(std::getenv("NUM_ROBOTS") ? std::stoi(std::getenv("NUM_ROBOTS")) : 0), 
        range(std::getenv("COMM_RANGE") ? std::stod(std::getenv("COMM_RANGE")) : 0.0),
        start_positions(num_cf),
        odom_linear(std::vector<geometry_msgs::msg::Point>(num_cf)),
        odom_quat(std::vector<geometry_msgs::msg::Quaternion>(num_cf)),
        pose_subscriptions_(num_cf),
        aruco_subscriptions_(num_cf),
        battery_subscriptions_(num_cf),
        start_charging_pub_(num_cf),
        stop_charging_pub_(num_cf),
        battery_status_(num_cf, 100),
        recharge_flag(false),
        drone_status(num_cf, std::make_pair(true, Eigen::Vector3d()))
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

        RCLCPP_INFO(this->get_logger(), "Initializing Solver object...");
        solver = std::make_shared<Solver>(Eigen::Vector3d(0, 0, 0), *tree, range, 5);
        solver->initialSetup();


        RCLCPP_INFO(this->get_logger(), "Starting search...");

        res_publisher_ = this->create_publisher<icuas25_msgs::msg::TargetInfo>("target_found", 10);
        aruco_timer_ = this->create_wall_timer(500ms, std::bind(&CrazyflieCommandClient::timer_callback, this), aruco_cb_group_);
        run_mission_timer_ = this->create_wall_timer(500ms, std::bind(&CrazyflieCommandClient::run_mission, this), run_mission_cb_group_);
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
        request->group_mask = 0;
        request->relative = false;
        request->goal.x = x;
        request->goal.y = y;
        request->goal.z = z;
        request->yaw = yaw; 

        double distance = dist(std::vector<double>{x, y, z}, std::vector<double>{odom_linear[drone - 1].x, odom_linear[drone - 1].y, odom_linear[drone - 1].z}); 
        double tau = 30/16.0;
        double c = 30/pow(tau, 5);
        double distance_tau = c*pow(tau,6)/60;
        double T;
        if(distance < 2*distance_tau){
            T = pow(84/5.0,0.5)/pow(5.0,0.25);
        }
        else{
            T = 2 * tau + (distance - 2 * distance_tau) * v_max;
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

    int go_to_traj(int drone, double start_x, double start_y, double start_z, double start_yaw, double x, double y, double z, double yaw){
        std::ofstream file("/wp.csv");
        file << start_x << "," << start_y << "," << start_z << "," << start_yaw << std::endl;
        file << x << "," << y << "," << z << "," << yaw << std::endl;
        file.close();
        
        std::system("$TRAJ_GEN -i /wp.csv --v_max 1.0 --a_max 1.0 -o /out.csv");
        auto duration = upload_trajectory(drone, 0, x, y, z, yaw, "/out.csv");
        start_trajectory(drone, duration, 0);
        
        drone_status[drone - 1] = std::make_pair(false, Eigen::Vector3d(x, y, z));

        return 0;
    }

    int go_to_traj(int drone, std::vector<double> x, std::vector<double> y, std::vector<double> z, double yaw){
        std::ofstream file("/wp.csv");
        for(int i = 0; i < x.size(); i++){
            file << x[i] << "," << y[i] << "," << z[i] << "," << yaw << std::endl;
        }
        file.close();
        
        std::system("$TRAJ_GEN -i /wp.csv --v_max 1.0 --a_max 1.0 -o /out.csv");
        auto duration = upload_trajectory(drone, 0, x.back(), y.back(), z.back(), yaw, "/out.csv");
        start_trajectory(drone, duration, 0);
        
        drone_status[drone - 1] = std::make_pair(false, Eigen::Vector3d(x.back(), y.back(), z.back()));
        rclcpp::sleep_for(std::chrono::milliseconds(200));
        return 0;
    }

    double upload_trajectory(const int& drone, uint trajectory_id, double x, double y, double z, double yaw, std::string filename){
        RCLCPP_INFO(this->get_logger(), "Initialized CrazyflieCommandClient::upload_trajectory for namespace: %d", drone);

        auto client = this->create_client<crazyflie_interfaces::srv::UploadTrajectory>("/cf_" + std::to_string(drone) + "/upload_trajectory", rmw_qos_profile_services_default, service_cb_group_);
        auto request = std::make_shared<crazyflie_interfaces::srv::UploadTrajectory::Request>();
        
        std::vector<crazyflie_interfaces::msg::TrajectoryPolynomialPiece> pieces;
        
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Cannot open " << filename << std::endl;
            return 1;
        }

        std::string line;
        std::getline(file, line);
        bool startLine = true;
        double total_d = 0;

        while (std::getline(file, line)) {
            if (line.empty() || startLine){
                startLine = false;
                continue;
            }

            std::istringstream ss(line);
            std::string token;
            std::vector<float> values;
            while (std::getline(ss, token, ',')) {
                values.push_back(std::stof(token));
            }

            if (values.size() < 33) {
                std::cerr << "Unexpected token count: " << values.size() << std::endl;
                continue;
            }

            crazyflie_interfaces::msg::TrajectoryPolynomialPiece piece;
            piece.poly_x.resize(8);
            piece.poly_y.resize(8);
            piece.poly_z.resize(8);
            piece.poly_yaw.resize(8);

            // Row pattern: token[0]=duration, tokens[1-8]=poly_x, [9-16]=poly_y, [17-24]=poly_z, [25-32]=poly_yaw
            float duration_value = values[0];
            for (size_t i = 0; i < 8; ++i) {
                piece.poly_x[i]   = values[1 + i];
                piece.poly_y[i]   = values[1 + 8 + i];
                piece.poly_z[i]   = values[1 + 16 + i];
                piece.poly_yaw[i] = values[1 + 24 + i];
            }
            // piece.poly_yaw[0] = yaw;

            builtin_interfaces::msg::Duration d;
            d.sec = static_cast<int32_t>(duration_value);
            d.nanosec = static_cast<uint32_t>((duration_value - d.sec) * 1e9);
            piece.duration = d;
            total_d = total_d + duration_value;
            pieces.push_back(piece);
        }

        // crazyflie_interfaces::msg::TrajectoryPolynomialPiece piece;
        // piece.poly_x.resize(8);
        // piece.poly_y.resize(8);
        // piece.poly_z.resize(8);
        // piece.poly_yaw.resize(8);
        // float duration_value = 20.0;
        // for (size_t i = 0; i < 8; ++i) {
        //     piece.poly_x[i]   = 0.0;
        //     piece.poly_y[i]   = 0.0;
        //     piece.poly_z[i]   = 0.0;
        //     piece.poly_yaw[i] = 0.0;
        // }
        // piece.poly_yaw[0] = yaw;
        // piece.poly_x[0]   = x;
        // piece.poly_y[0]   = y;
        // piece.poly_z[0]   = z;

        // builtin_interfaces::msg::Duration d;
        // d.sec = static_cast<int32_t>(duration_value);
        // d.nanosec = static_cast<uint32_t>((duration_value - d.sec) * 1e9);
        // piece.duration = d;
        // total_d = total_d + duration_value;
        // pieces.push_back(piece);

        file.close();
        request->trajectory_id = trajectory_id;
        request->piece_offset = 0;
        request->pieces = pieces;

        wait_for_service(client, "/cf_" + std::to_string(drone) + "/upload_trajectory");

        for(int i = 0; i < pieces.size(); i++){
            for(int j = 0; j < pieces[i].poly_x.size(); j++){
                std::cout << pieces[i].poly_x[j] << " ";
            }
            for(int j = 0; j < pieces[i].poly_y.size(); j++){
                std::cout << pieces[i].poly_y[j] << " ";
            }
            for(int j = 0; j < pieces[i].poly_z.size(); j++){
                std::cout << pieces[i].poly_z[j] << " ";
            }
            for(int j = 0; j < pieces[i].poly_yaw.size(); j++){
                std::cout << pieces[i].poly_yaw[j] << " ";
            }
            std::cout << std::endl;
        }

        auto result = client->async_send_request(request).get();
        RCLCPP_INFO(this->get_logger(), "UploadTrajectory request sent to %d",
                    drone);

        return total_d;
    }

    int start_trajectory(const int& drone, float timescale, uint trajectory_id){
        RCLCPP_INFO(this->get_logger(), "Initialized CrazyflieCommandClient::start_trajectory for namespace: %d", drone);

        auto client = this->create_client<crazyflie_interfaces::srv::StartTrajectory>("/cf_" + std::to_string(drone) + "/start_trajectory", rmw_qos_profile_services_default, service_cb_group_);
        auto request = std::make_shared<crazyflie_interfaces::srv::StartTrajectory::Request>();
         
        request->group_mask = 0;
        request->trajectory_id = trajectory_id;
        request->timescale = 1;
        request->reversed = false;
        request->relative = false;

        wait_for_service(client, "/cf_" + std::to_string(drone) + "/start_trajectory");


        auto result = client->async_send_request(request).get();
        RCLCPP_INFO(this->get_logger(), "StartTrajectory request sent to %d",
                    drone);

        return 0;
    }

    int go_for_recharge(int curr_vertex){
        std::cout << "inside go_for_recharge, min_charge: " << min_charge << " recharge_flag: " << recharge_flag << std::endl; 
        int lca = 0;
        
        int v = curr_vertex;
        // back to base from curr
        std::cout << utils::Color::FG_RED << "going to base! curr:" << v << " -> base:" << lca << utils::Color::FG_DEFAULT << std::endl;
        int n_drones_curr = mp[v].size(); 
        std::cout << "Number of drones at v (discharge): " << n_drones_curr << std::endl;

        int duration = 0;
        int max_duration = 0;
        while(v != lca){
            max_duration = 0;
            while(!mp[v].empty()){
                int drone = mp[v].back(); mp[v].pop_back();
                duration = go_to_vertex(drone, solution_ptr->parent[v], solution_ptr->nodes_graph);
                max_duration = std::max(duration, max_duration);
                
                mp[solution_ptr->parent[v]].push_back(drone);
            }
            wait_to_reach();
            // rclcpp::sleep_for(std::chrono::seconds(max_duration)); 
            v = solution_ptr->parent[v];
        }
        wait_to_reach();
        // rclcpp::sleep_for(std::chrono::seconds(max_duration)); 

        while(min_charge < 88){
            rclcpp::sleep_for(std::chrono::seconds(1));
            std::cout << "Charging..., min_charge: " << min_charge << " target_charge: " << 89 << std::endl;
        }

        // back to base from curr (v)

        v = curr_vertex;
        std::vector<int> lcaToCurrPath;
        int tmp = v;
        while(tmp != lca){
            lcaToCurrPath.push_back(tmp);
            tmp = solution_ptr->parent[tmp];
        }

        std::cout << "lcaToCurrPath: ";
        for(int i = 0; i < lcaToCurrPath.size(); i++){
            std::cout << lcaToCurrPath[i] << " ";
        }
        std::cout << std::endl;

        int n_drones_req = n_drones_curr;
        std::cout << utils::Color::FG_RED << "going to curr:" << v << " <- base:" << lca << utils::Color::FG_DEFAULT << std::endl;;    
        int k = lca;
        for(int j = int(lcaToCurrPath.size()) - 1; j >= 0; j--){
            max_duration = 0;
            for(int itr = 0; itr < n_drones_req; itr++){
                
                int drone = mp[k].back(); mp[k].pop_back();
                
                duration = go_to_vertex(drone, lcaToCurrPath[j], solution_ptr->nodes_graph);
                if(j < 0){
                    break;
                }
                
                mp[lcaToCurrPath[j]].push_back(drone);
                max_duration = std::max(duration, max_duration);
            }
            wait_to_reach();
            // rclcpp::sleep_for(std::chrono::seconds(max_duration)); 

            k = lcaToCurrPath[j];
            n_drones_req--;
        }
        
        return 0;
    }

    double go_to_vertex(int drone, int v, std::vector<Eigen::Vector3d>& nodes_graph){
        if(v == 0){
            std::cout << utils::Color::FG_BLUE << "GoTo: [" << drone << "]" << ":" << "(" <<   0 << ")" << " min_charge: " << min_charge << utils::Color::FG_DEFAULT << std::endl;
            int duration = go_to(drone, start_positions[drone-1][0], start_positions[drone-1][1], start_positions[drone-1][2] + land_h + drone_h[drone-1], 0.0);
            // go_to_traj(drone, odom_linear[drone-1].x, odom_linear[drone-1].y, odom_linear[drone-1].z, get_yaw(drone-1),start_positions[drone-1][0], start_positions[drone-1][1], start_positions[drone-1][2] + land_h + drone_h[drone-1], 0.0);

            rclcpp::sleep_for(std::chrono::milliseconds(1000));
            return 1;
        }
        auto curr = nodes_graph[v];
        curr[2] += drone_h[drone];
        int duration = 0;
        std::cout << utils::Color::FG_BLUE << "GoTo: [" << drone << "]" << ":" << "(" <<   v << ")" << " min_charge: " << min_charge << utils::Color::FG_DEFAULT << std::endl;

        duration = go_to(drone, curr[0], curr[1], curr[2], 0);
        // go_to_traj(drone, odom_linear[drone-1].x, odom_linear[drone-1].y, odom_linear[drone-1].z, get_yaw(drone-1), curr[0], curr[1], curr[2], 0);

        // rclcpp::sleep_for(std::chrono::milliseconds(500));
        return duration;
    }

    int getLca(int u, int v, std::vector<int>& parent){
        while(u != v){
            u = parent[u];
            v = parent[v];

            if(u == 0 || v == 0) return 0;
        }
        return u;
    }

    int visualizeTree(){
        std::cout << "Visualizing the tree..." << std::endl;
        std::map<int,tree_viz::Node*> tree_node;
        
        for(uint i = 0; i < solution_ptr->bfs_order.size(); i++){
            int curr = solution_ptr->bfs_order[i].first;
            
            // std::cout << "added: " << curr << std::endl;
            tree_node[curr] = new tree_viz::Node(curr);
        }
        
        for(uint i = 1; i < solution_ptr->bfs_order.size(); i++){
            int curr = solution_ptr->bfs_order[i].first;
            int parent_curr = solution_ptr->parent[curr];

            // std::cout << "curr: " << curr << " parent[curr]: " << parent_curr << std::endl;

            if(tree_node.find(parent_curr) != tree_node.end()){
                tree_node[parent_curr]->children.push_back(tree_node[curr]);
            }
            else{
                std::cerr << parent_curr << " not in the map" << std::endl;
                throw std::runtime_error("parent_curr not initialized in the map!");
            }
        }
        auto root = tree_node[0];

        std::ofstream dotFile("tree.dot", std::ios::out);
        dotFile << "digraph Tree {" << std::endl;
        dotFile << "    node [shape=circle];" << std::endl;

        generateDot(dotFile, root);

        dotFile << "}" << std::endl;
        dotFile.close();
        std::cout << "DOT file 'tree.dot' generated successfully!" << std::endl;

        std::system("dot -Tpng tree.dot -o tree.png");
        return 0;
    }

    int run_mission(){
        if(!flag){
            // go_to_traj(1, 0, 0, 0, 21, 18, 0.7, 1.57);
            // store the start positions
            for(int i = 0; i < num_cf; i++){
                start_positions[i] = std::vector<double>({odom_linear[i].x, odom_linear[i].y, odom_linear[i].z});
            }

            solution_ptr = &(solver->solution);
            visualizeTree();
            // exit(1);
    
            double curr_h = 0;
            for(int i = 1; i <= num_cf; i++){
                drone_h[i] = curr_h;
                curr_h -= 0.15;
            }

            mp[0] = {1,2,3,4,5};
            
            int prev = 0;
            int curr = 0;   
            int duration = 0;
            int max_duration = 0;
            std::cout << solution_ptr->bfs_order.size() << std::endl;

            auto start_time = std::chrono::steady_clock::now();
            std::cout << "Mission started!" << std::endl;
            
            for(uint i = 0; i < solution_ptr->bfs_order.size(); i++){

                auto current_time = std::chrono::steady_clock::now();
                auto elapsed_time = std::chrono::duration_cast<std::chrono::hours>(current_time - start_time).count();


                auto& first_face = (solution_ptr->bfs_order[i].second).first; 
                auto& second_face = (solution_ptr->bfs_order[i].second).second; 

                if(first_face.empty() && second_face.empty()) continue; //dont visit nodes with nothing to visit
                if(first_face.size() + second_face.size() <= 6) continue; // dont visit trivial nodes (waste of time) 
                if(curr == solution_ptr->bfs_order[i].first) continue; // if already there, dont need to go there again (initialized at base itself)

                curr = solution_ptr->bfs_order[i].first;
                int lca = getLca(prev, curr, solution_ptr->parent);
                
                if (elapsed_time >= 2) {
                    std::cout << "Mission time greater than 2 hr, going back to base" << std::endl;
                    lca = 0;
                }

                // back to lca from prev
                std::cout << utils::Color::FG_BLUE << "going from prev:" << prev << " -> lca:" << lca << utils::Color::FG_DEFAULT << std::endl;;
                    
                while(prev != lca){
                    max_duration = 0;
                    while(!mp[prev].empty()){
                        int drone = mp[prev].back(); mp[prev].pop_back();

                        if(recharge_flag){
                            go_for_recharge(prev);
                        }

                        duration = go_to_vertex(drone, solution_ptr->parent[prev], solution_ptr->nodes_graph);
                        max_duration = std::max(duration, max_duration);
                        
                        mp[solution_ptr->parent[prev]].push_back(drone);
                    }
                    wait_to_reach();
                    // rclcpp::sleep_for(std::chrono::seconds(max_duration)); 
                    prev = solution_ptr->parent[prev];
                }

                if(elapsed_time >= 2){
                    std::cout << "Mission time exceeded 2hr, exiting...." << std::endl;
                    exit(1);
                }

                // if lca is base (reorder for the most charged ones to go)
                if(lca == 0){
                    std::vector<int> drones_lca;
                    while(!mp[lca].empty()){
                        drones_lca.push_back(mp[lca].back());
                        mp[lca].pop_back();
                    }
                    std::sort(drones_lca.begin(), drones_lca.end(), [&](int a, int b){ return battery_status_[a-1] < battery_status_[b-1]; });

                    for(uint i = 0; i < drones_lca.size(); i++){
                        mp[lca].push_back(drones_lca[i]);
                    } // TODO: cross-check this logic
                    
                    if(recharge_flag){
                        while(min_charge < 88){
                            rclcpp::sleep_for(std::chrono::seconds(1));
                            std::cout << "Charging..., min_charge: " << min_charge << " target charge: " << 89 << std::endl;
                        }
                    }
                }

                std::vector<int> lcaToCurrPath;
                int tmp = curr;
                while(tmp != lca){
                    lcaToCurrPath.push_back(tmp);
                    tmp = solution_ptr->parent[tmp];
                }
    
                // from lca to curr
                int n_drones_req = solution_ptr->distance[curr] - solution_ptr->distance[lca] + 1;
                std::cout << utils::Color::FG_BLUE << "going from lca:" << lca << " -> curr:" << curr << utils::Color::FG_DEFAULT << std::endl;;    
                int k = lca;
                for(int j = int(lcaToCurrPath.size()) - 1; j >= 0; j--){
                    max_duration = 0;
                    for(int itr = 0; itr < n_drones_req; itr++){
                        
                        int drone = mp[k].back(); mp[k].pop_back();
                        
                        // std::cout << "3minimum charge = " << min_charge << "recharge_flag: " << recharge_flag << std::endl;
                        if(recharge_flag){
                            go_for_recharge(k);
                        }


                        duration = go_to_vertex(drone, lcaToCurrPath[j], solution_ptr->nodes_graph);
                        if(j < 0){
                            break;
                        }
                        
                        mp[lcaToCurrPath[j]].push_back(drone);
                        max_duration = std::max(duration, max_duration);
                    }
                    
                    wait_to_reach();
                    // rclcpp::sleep_for(std::chrono::seconds(max_duration)); 
                    k = lcaToCurrPath[j];
                    n_drones_req--;

                }
               
                if(mp[curr].size() < 2){
                    rclcpp::sleep_for(std::chrono::seconds(2)); 
                    throw std::runtime_error("size of mp[curr] is less than 2, at last node less than two drones?");
                }
    
                std::vector<int> curr_drones;
                std::cout << "size: " << mp[curr].size() << std::endl;
                int temp_size = mp[curr].size();
                for(int i = 0; i < temp_size; i++){
                    curr_drones.push_back(mp[curr].back());
                    mp[curr].pop_back();
                }
                std::sort(curr_drones.begin(), curr_drones.end());
                for(int i = 0; i < curr_drones.size(); i++){
                    std::cout << curr_drones[i] << " ";
                    mp[curr].push_back(curr_drones[i]);
                }
                int scan_drone = mp[curr].back();
                std::cout << std::endl;
                
                Eigen::Vector4d start;
                Eigen::Vector4d end;


                while(!first_face.empty()){
                    if(recharge_flag){
                        duration = go_to_vertex(scan_drone, curr, solution_ptr->nodes_graph);
                        wait_to_reach();
                        // rclcpp::sleep_for(std::chrono::seconds(duration)); 

                        std::cout << "number of drones to recall at " << curr << ": " << mp[curr].size() << std::endl;
                        go_for_recharge(curr);
                    }

                    end = first_face.back(); first_face.pop_back();
                    start = first_face.back(); first_face.pop_back();

                    duration = go_to(scan_drone, start[0], start[1], start[2], start[3]);
                    std::cout << utils::FG_GREEN << "GoTo: [" << scan_drone << "]" << ":" << "(" << start[0] << "," << start[1] << "," << start[2] <<  "," << start[3] << ")" << " min_charge: " << min_charge << utils::FG_DEFAULT << std::endl;
                    wait_to_reach();
                    // rclcpp::sleep_for(std::chrono::seconds(duration)); 
                    
                    duration = go_to(scan_drone, end[0], end[1], end[2], end[3]);
                    std::cout << utils::FG_GREEN <<"GoTo: [" << scan_drone << "]" << ":" << "(" << end[0] << "," << end[1] << "," << end[2] <<  "," << end[3] << ")" << " min_charge: " << min_charge << utils::FG_DEFAULT << std::endl;
                    wait_to_reach();
                    // rclcpp::sleep_for(std::chrono::seconds(duration)); 
                }
                std::cout << "Returning to vertex from face!" << std::endl;
                duration = go_to_vertex(scan_drone, curr, solution_ptr->nodes_graph);
                wait_to_reach();
                // rclcpp::sleep_for(std::chrono::seconds(duration)); 

    
                while(!second_face.empty()){
                    if(recharge_flag){
                        duration = go_to_vertex(scan_drone, curr, solution_ptr->nodes_graph);
                        wait_to_reach();
                        // rclcpp::sleep_for(std::chrono::seconds(duration)); 

                        std::cout << "number of drones to recall at " << curr << ": " << mp[curr].size() << std::endl;
                        go_for_recharge(curr);
                    }

                    end = second_face.back(); second_face.pop_back();
                    start = second_face.back(); second_face.pop_back();

                    duration = go_to(scan_drone, start[0], start[1], start[2], start[3]);
                    std::cout << utils::FG_GREEN << "GoTo: [" << scan_drone << "]" << ":" << "(" << start[0] << "," << start[1] << "," << start[2] <<  "," << start[3] << ")" << " min_charge: " << min_charge << utils::FG_DEFAULT << std::endl;
                    wait_to_reach();
                    // rclcpp::sleep_for(std::chrono::seconds(duration)); 
                    
                    duration = go_to(scan_drone, end[0], end[1], end[2], end[3]);
                    std::cout << utils::FG_GREEN <<"GoTo: [" << scan_drone << "]" << ":" << "(" << end[0] << "," << end[1] << "," << end[2] <<  "," << end[3] << ")" << " min_charge: " << min_charge << utils::FG_DEFAULT << std::endl;
                    wait_to_reach();
                    // rclcpp::sleep_for(std::chrono::seconds(duration)); 
                }
                std::cout << "Returning to vertex from face!" << std::endl;
                duration = go_to_vertex(scan_drone, curr, solution_ptr->nodes_graph);
                wait_to_reach();
                // rclcpp::sleep_for(std::chrono::seconds(duration)); 
    
                prev = curr;

                std::cout << "Going to next vertex!" << std::endl;
            }
            
            flag = true;
        }
        return 0;
    }

private:
    double get_yaw(int drone){
        return atan2(2*(odom_quat[drone].w*odom_quat[drone].z + odom_quat[drone].x*odom_quat[drone].y), 1 - 2*(odom_quat[drone].y*odom_quat[drone].y + odom_quat[drone].z*odom_quat[drone].z));
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

    void battery_callback(const sensor_msgs::msg::BatteryState & msg, const int drone_namespace_){
        int k = drone_namespace_ - 1;

        battery_status_[k] = msg.percentage;
        // std::cout << "charge of " << drone_namespace_ << " " << msg.percentage << std::endl;
        double curr_min = 100.0;
        for(double curr : battery_status_){
            curr_min = std::min(curr_min, curr);
        }
        min_charge = curr_min;

        if(min_charge < 35){
            // std::cout << "Setting recharge flag to TRUE, min_charge: " << min_charge << std::endl;
            recharge_flag = true;
        }
        else{
            // std::cout << "Recharge flag not set" << std::endl;
        }
    }

    void timer_callback(){
        if(curr_aruco_position[0] != -1e8){
            RCLCPP_INFO(this->get_logger(), "Publishing to /target_found!");

            auto res = icuas25_msgs::msg::TargetInfo();
            res.id = curr_aruco_id;
            Eigen::Quaterniond quaternion(odom_quat[0].w, odom_quat[0].x, odom_quat[0].y, odom_quat[0].z); 
            quaternion.normalize();

            Eigen::Vector3d point(curr_aruco_position[2], -curr_aruco_position[0], -curr_aruco_position[1]);

            Eigen::Vector3d transformed_point = quaternion * point;
            res.location.x = odom_linear[0].x + transformed_point.x();
            res.location.y = odom_linear[0].y + transformed_point.y();
            res.location.z = odom_linear[0].z + transformed_point.z();

            res_publisher_->publish(res);

            curr_aruco_position = {-1e8, -1e8, -1e8};
        }
    }

    bool check(){
        for(int i = 0; i < num_cf; ++i){
            double x = drone_status[i].second.x();
            double y = drone_status[i].second.y();
            double z = drone_status[i].second.z();

            if(drone_status[i].first == false && dist(std::vector<double>({x, y, z}), std::vector<double>({odom_linear[i].x, odom_linear[i].y, odom_linear[i].z})) < EPS){
                drone_status[i].first = true;
                
                if(drone_status[i].second.x() == start_positions[i][0] && drone_status[i].second.y() == start_positions[i][1] && drone_status[i].second.z() == start_positions[i][2] + land_h + drone_h[i]){
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
            rclcpp::sleep_for(std::chrono::milliseconds(500));
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

    octomap::OcTree* tree;
    std::shared_ptr<Solver> solver;

    int num_cf;
    double range;
    bool flag = false;
    Solution* solution_ptr;

    Eigen::Vector4d start;
    Eigen::Vector4d goal;
    std::map<int,double> drone_h;
    std::vector<std::vector<double>> start_positions;

    std::vector<geometry_msgs::msg::Point> odom_linear;
    std::vector<geometry_msgs::msg::Quaternion> odom_quat;

    rclcpp::CallbackGroup::SharedPtr service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr aruco_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr battery_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr run_mission_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    rclcpp::SubscriptionOptions aruco_cb_options;
    rclcpp::SubscriptionOptions battery_cb_options;

    rclcpp::TimerBase::SharedPtr aruco_timer_;
    rclcpp::TimerBase::SharedPtr run_mission_timer_;

    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subscriptions_;
    std::vector<rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr> aruco_subscriptions_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr> battery_subscriptions_;
    rclcpp::Publisher<icuas25_msgs::msg::TargetInfo>::SharedPtr res_publisher_;

    std::vector<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> start_charging_pub_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> stop_charging_pub_;

    double curr_aruco_id;
    std::vector<double> curr_aruco_position = {-1e8, -1e8, -1e8};
    std::vector<double> prev_aruco_position = {-1e8, -1e8, -1e8};

    std::vector<float> battery_status_;
    bool recharge_flag;
    std::map<int,std::deque<int>> mp;
    double min_charge = 100;
    std::vector<std::pair<bool, Eigen::Vector3d>> drone_status;

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
