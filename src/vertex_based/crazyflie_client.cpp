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
#include <cstdlib>  

#include "vertex_based/traversal.hpp"

using namespace std::chrono_literals;
                                              
double EPS = 1E-1;

class CrazyflieCommandClient : public rclcpp::Node
{
public:
    CrazyflieCommandClient(int num = 5) : 
        Node("crazyflie_command_client"), 
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

// /cf_1/battery_charge/start
// /cf_1/battery_charge/stop
// /cf_1/battery_status

        RCLCPP_INFO(this->get_logger(), "Getting Octomap...");
        get_octomap();

        RCLCPP_INFO(this->get_logger(), "Initializing Solver object...");
        solver = std::make_shared<Solver>(Eigen::Vector3d(0, 0, 0), *tree, 43, 5);
        solver->initialSetup();

        RCLCPP_INFO(this->get_logger(), "Initializing Path Planner object...");
        planner = std::make_shared<Planner>(Planner(tree, solver->mapBounds, this->get_logger()));

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

    double go_to(const int &drone_namespace_, double x, double y, double z, double yaw)
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
        request->duration.sec = 2 * dist(std::vector<double>{x, y, z}, std::vector<double>{odom_linear[drone_namespace_ - 1].x, odom_linear[drone_namespace_ - 1].y, odom_linear[drone_namespace_ - 1].z}); 
        request->duration.nanosec = 0;

        wait_for_service(client, "/cf_" + std::to_string(drone_namespace_) + "/go_to");

        int i = drone_namespace_ - 1;
        RCLCPP_INFO(this->get_logger(), "GoTo request sent to %d with goal: [%.2f, %.2f, %.2f], distance: %.2f",
                    drone_namespace_, x, y, z, dist(std::vector<double>({x, y, z}), std::vector<double>({odom_linear[i].x, odom_linear[i].y, odom_linear[i].z})));

        auto result = client->async_send_request(request).get();

        /*
        while(dist(std::vector<double>({x, y, z}), std::vector<double>({odom_linear[i].x, odom_linear[i].y, odom_linear[i].z})) < EPS){
            RCLCPP_DEBUG(this->get_logger(), "%d going to goal: [%.2f, %.2f, %.2f], odom: [%.2f, %.2f, %.2f]",
            drone_namespace_, x, y, z, odom_linear[i].x, odom_linear[i].y, odom_linear[i].z);
        }
        */
        return 2 * dist(std::vector<double>{x, y, z}, std::vector<double>{odom_linear[drone_namespace_ - 1].x, odom_linear[drone_namespace_ - 1].y, odom_linear[drone_namespace_ - 1].z});
    }

    double go_to_vertex(int& drone_namepsace_, int& v, std::vector<Eigen::Vector3d>& nodes_graph, std::map<int,double> drone_h){
        auto p = nodes_graph[v];
        p[2] += drone_h[drone_namespace_];
        auto duration = this->go_to(drone_namespace_, p[0], p[1], p[2], 0);
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

    int run_mission(){
        std::map<int,double> drone_h;
        double curr_h = 0;
        for(int i = 1; i <= num_cf; i++){
            drone_h[i] = curr_h;
            curr_h -= 0.15;
        }

        std::map<int,std::deque<int>> mp;
        mp[0] = {1,2,3,4,5};
        
        int prev = 0;
        int curr = 0;   
        double duration = 0.0;
        double max_duration = 0.0;
        for(int i = 0; i < solution->bfs_order.size(); i++){
            curr = solution->bfs_order[i].first;
            int lca = getLca(prev, curr);

            // back to lca from prev
            while(prev > lca){
                max_duration = 0.0;
                while(!mp[prev].empty()){
                    int drone = mp.back(); mp.pop_back();
                    duration = go_to_vertex(drone, solution->parent[prev], solution->node_graph, drone_h);
                    max_duration = std::max(duration, max_duration);

                    mp[solution->parent[prev]].push_back(drone);
                }
                rclcpp::sleep_for(std::chrono::seconds(max_duration)); 
                prev = solution->parent[prev];
                mp[prev].clear();
            }
            

            std::vector<int> lcaToCurrPath;
            int tmp = curr;
            while(tmp != lca){
                lcaToCurrPath.push_back(tmp);
                tmp = solution->parent[tmp];
            }

            // from lca to curr
            int k = lca;
            for(int j = lcaToCurrPath.size() - 1; j >= 0; j--){
                while(mp[k].size() > 1){
                    int drone = mp.back(); mp.pop_back();

                    duration = go_to_vertex(drone, localToCurrPath[j], solution->node_graph, drone_h);
                    max_duration = std::max(duration, max_duration);
                } 
                rclcpp::sleep_for(std::chrono::seconds(max_duration)); 
                k = lcaToCurrPath[j];
            }
            
            std::assert(mp[curr].size() > 2);
            int scan_drone = mp[curr].back();
            auto& first_face = (bfs_order[i].second).first; 
            auto& second_face = (bfs_order[i].second).second; 

            if(!(first_face).empty()){
                Eigen::Vector4d start;
                Eigen::Vector4d end;
                for(int j = 0; j < (first_face).size(); j+=2){
                    start = first_face[j];
                    end = first_face[j+1];

                    duration = this->go_to(drone_, curr_x, curr_y, curr_z, 0);
                    rclcpp::sleep_for(std::chrono::seconds(duration)); 

                    duration = go_to_vertex(scan_drone, end, solution->node_graph, drone_h, false);
                    rclcpp::sleep_for(std::chrono::seconds(duration)); 

                    duration = go_to_vertex(scan_drone, start, solution->node_graph, drone_h, false);
                    rclcpp::sleep_for(std::chrono::seconds(duration)); 
                }
                duration = go_to_vertex(scan_drone, curr, solution->node_graph, drone_h);
            }

            if(!(second_face).empty()){
                Eigen::Vector3d start;
                Eigen::Vector3d end;
                for(int j = 0; j < (second_face).size(); j+=2){
                    start = second_face[j];
                    end = second_face[j+1];

                    duration = go_to_vertex(scan_drone, start, solution->node_graph, drone_h, false);
                    rclcpp::sleep_for(std::chrono::seconds(duration)); 

                    duration = go_to_vertex(scan_drone, end, solution->node_graph, drone_h, false);
                    rclcpp::sleep_for(std::chrono::seconds(duration)); 

                    duration = go_to_vertex(scan_drone, start, solution->node_graph, drone_h, false);
                    rclcpp::sleep_for(std::chrono::seconds(duration)); 
                }
                duration = go_to_vertex(scan_drone, curr, solution->node_graph, drone_h);
            }
        }
        return 0;
    }

    ~CrazyflieCommandClient(){
        delete tree;
    }

private:
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

    std::vector<std::vector<double>> runTSP(std::vector<std::vector<double>> waypoints){
        std::ofstream wfile;
        wfile.open("/waypoints.csv");    
        for(int i = 0; i < waypoints.size(); i++){
            if(wfile.is_open()){
                wfile << waypoints[i][0] << " " << waypoints[i][1] << " " << waypoints[i][2] << "\n";
            }
        }
        wfile.close();

        const char* tsp_solver = std::getenv("TSP_SOLVER");
        std::string command = tsp_solver; 
        int status = system(command.c_str());

        if (status == 0) {
            std::cout << "TSP solver ran successfully" << std::endl;
        } else {
            std::cerr << "Error in running TSP solver" << std::endl;
        }

        std::vector<std::vector<double>> reordered_waypoints;
        double x, y, z;
        std::string line;
        std::ifstream rfile;
        rfile.open("/waypoints_tsp.csv");
        if (rfile.is_open()) {
            while (std::getline(rfile, line)) {
                rfile >> x >> y >> z;
                reordered_waypoints.push_back({x, y, z});
            }
            rfile.close();
        }
        return reordered_waypoints;
    }

    bool checkLoS(Eigen::Vector3d start, Eigen::Vector3d goal) const {
        octomap::OcTreeNode *node = tree->search(goal.x(), goal.y(), goal.z());
        if (node != nullptr && tree->isNodeOccupied(node)) {
            return false;
        }
        octomap::point3d start(start.x(), start.y(), start.z());
        octomap::point3d goal(goal.x(), goal.y(), goal.z());
        octomap::point3d hit;

        bool collision = octree_->castRay(start, goal - start, hit, true, (goal - start).norm() - 0.01);
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
    std::shared_ptr<Solver> solver;
    std::shared_ptr<Solution> solution;
    std::shared_ptr<Planner> planner;

    int num_cf;

    Eigen::Vector4d start;
    Eigen::Vector4d goal;

    std::vector<geometry_msgs::msg::Point> odom_linear;
    std::vector<geometry_msgs::msg::Quaternion> odom_quat;

    rclcpp::CallbackGroup::SharedPtr service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr aruco_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr run_mission_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions aruco_cb_options;

    rclcpp::TimerBase::SharedPtr aruco_timer_;
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

    auto client_node = std::make_shared<CrazyflieCommandClient>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(client_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}