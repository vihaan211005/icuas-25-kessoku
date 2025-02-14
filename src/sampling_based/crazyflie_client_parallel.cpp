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

#include "planner_parallel.hpp"

//TODO: currently initializing the timer only when done with mission, can we do it from the start with empty stuff being published?
//      stacking error, next_h > curr_h; next_h < curr_h
        
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

        RCLCPP_INFO(this->get_logger(), "Getting Octomap...");
        get_octomap();

        RCLCPP_INFO(this->get_logger(), "Initializing Solver object...");
        solver = std::make_shared<Solver>(Eigen::Vector3d(0, 0, 0), *tree, 43, 5);
        solver->initialSetup();
        mutex_ptr = &(solver->param_mutex);

        RCLCPP_INFO(this->get_logger(), "Initializing Path Planner object...");
        planner = std::make_shared<Planner>(Planner(tree, solver->mapBounds, this->get_logger(), &plannerConsumerQueue));

        RCLCPP_INFO(this->get_logger(), "Starting search...");

        res_publisher_ = this->create_publisher<icuas25_msgs::msg::TargetInfo>("target_found", 10);
        aruco_timer_ = this->create_wall_timer(500ms, std::bind(&CrazyflieCommandClient::timer_callback, this), aruco_cb_group_);
        compute_timer_ = this->create_wall_timer(500ms, std::bind(&CrazyflieCommandClient::compute_callback, this), solution_cb_group_);
        run_mission_timer_ = this->create_wall_timer(500ms, std::bind(&CrazyflieCommandClient::run_mission, this), run_mission_cb_group_);
        planner_consumer_timer_ = this->create_wall_timer(500ms, std::bind(&CrazyflieCommandClient::planner_consumer_callback, this), planner_consumer_cb_group_);
        planner_producer_timer_ = this->create_wall_timer(500ms, std::bind(&CrazyflieCommandClient::planner_producer_callback, this), planner_producer_cb_group_);
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

    int upload_trajectory(const int& drone_namespace_, uint trajectory_id = 0, std::string filename="/figure8.csv"){
        RCLCPP_INFO(this->get_logger(), "Initialized CrazyflieCommandClient::upload_trajectory for namespace: %d", drone_namespace_);

        auto client = this->create_client<crazyflie_interfaces::srv::UploadTrajectory>("/cf_" + std::to_string(drone_namespace_) + "/upload_trajectory", rmw_qos_profile_services_default, service_cb_group_);
        auto request = std::make_shared<crazyflie_interfaces::srv::UploadTrajectory::Request>();
        
        std::vector<crazyflie_interfaces::msg::TrajectoryPolynomialPiece> pieces;
        
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Cannot open /figure8.csv" << std::endl;
            return 1;
        }

        std::string line;
        std::getline(file, line);
        bool startLine = true;

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

            if (values.size() != 33) {
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

            builtin_interfaces::msg::Duration d;
            d.sec = static_cast<int32_t>(duration_value);
            d.nanosec = static_cast<uint32_t>((duration_value - d.sec) * 1e9);
            piece.duration = d;

            pieces.push_back(piece);
        }
        file.close();

        request->trajectory_id = trajectory_id;
        request->piece_offset = 0;
        request->pieces = pieces;

        wait_for_service(client, "/cf_" + std::to_string(drone_namespace_) + "/upload_trajectory");

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
                    drone_namespace_);

        return 0;
    }

    int start_trajectory(const int& drone_namespace_, float timescale = 0, uint trajectory_id = 0){
        RCLCPP_INFO(this->get_logger(), "Initialized CrazyflieCommandClient::start_trajectory for namespace: %d", drone_namespace_);

        auto client = this->create_client<crazyflie_interfaces::srv::StartTrajectory>("/cf_" + std::to_string(drone_namespace_) + "/start_trajectory", rmw_qos_profile_services_default, service_cb_group_);
        auto request = std::make_shared<crazyflie_interfaces::srv::StartTrajectory::Request>();
         
        request->group_mask = 0;
        request->trajectory_id = trajectory_id;
        request->timescale = 5;
        request->reversed = false;
        request->relative = false;

        wait_for_service(client, "/cf_" + std::to_string(drone_namespace_) + "/start_trajectory");

        int i = drone_namespace_ - 1;

        auto result = client->async_send_request(request).get();
        RCLCPP_INFO(this->get_logger(), "StartTrajectory request sent to %d",
                    drone_namespace_);

        return 0;
    }

    int run_mission(){
        bool run = false;
        {
            boost::lock_guard<boost::mutex> lock(*(this->mutex_ptr));
            if(solver->solution.flag == false && solver->solution.eval > 0){
                solution = std::make_shared<Solution>(solver->solution);
                solver->solution.flag = true;
                run = true;
            }
        }

        if(!run) return 0;

        RCLCPP_INFO(this->get_logger(), "Got a solution!");

        double diff_z = 0.2;

        // go to start point
        double curr_x = solution->startPts[0].x();
        double curr_y = solution->startPts[0].y();
        double curr_z = 4;
        double prev_z = 4;

        // rclcpp::sleep_for(std::chrono::seconds(2));

        std::stack<std::pair<int,std::vector<double>> > st;
        long long time_to_wait = 0;
        long long curr_time_to_wait = 0;

        std::vector<Eigen::Vector3d> pos_(5);

        // double start_yaw;
        // double goal_yaw;

        for(int i = 1; i <= 5; i++){
            st.push({0, {i, curr_x, curr_y, curr_z}});
            //std::cout << "[" << i << "]" << ":" << "(" << curr_x << "," << curr_y << "," << curr_z << ")" << std::endl;
            // curr_time_to_wait = this->go_to(i, curr_x, curr_y, curr_z, 0);
            time_to_wait = std::max(curr_time_to_wait, time_to_wait);

            curr_z += diff_z;
        }
        // rclcpp::sleep_for(std::chrono::seconds(time_to_wait));
        

        // go onward to their respective vantage points
        for(uint i = 1; i < solution->startPts.size(); i++){
            time_to_wait = 0;

            double curr_x = solution->startPts[i].x();
            double curr_y = solution->startPts[i].y();
            double curr_z = solution->startPts[i].z();
            if(solution->startPts[i].z() <= prev_z){
                for(uint drone_ = i; drone_ <= 5; drone_++){
                    // rclcpp::sleep_for(std::chrono::seconds(5));
                    st.push({i, {drone_, curr_x, curr_y, curr_z}});
                    // std::cout << "[" << drone_ << "]" << ":" << "(" << curr_x << "," << curr_y << "," << curr_z << ")" << std::endl;
                    curr_time_to_wait = this->go_to(drone_, curr_x, curr_y, curr_z, 0);
                    time_to_wait = std::max(curr_time_to_wait, time_to_wait);

                    curr_z += diff_z;

                    if(i == solution->startPts.size() - 1 && drone_ == 4){
                        center = octomap::point3d(curr_x, curr_y, curr_z);
                    }
                    if(i == solution->startPts.size() - 1 && drone_ == 5){
                        start = Eigen::Vector4d(curr_x, curr_y, curr_z, 0);
                    }
                    pos_[drone_-1] = Eigen::Vector3d(curr_x, curr_y, curr_z); 
                }
            }
            else{
                curr_z += diff_z*(5 - i);
                for(uint drone_ = 5; drone_ >= i; drone_--){
                    // rclcpp::sleep_for(std::chrono::seconds(5));
                    st.push({i, {drone_, curr_x, curr_y, curr_z}});
                    // std::cout << "[" << drone_ << "]" << ":" << "(" << curr_x << "," << curr_y << "," << curr_z << ")" << std::endl;
                    curr_time_to_wait = this->go_to(drone_, curr_x, curr_y, curr_z, 0);
                    time_to_wait = std::max(curr_time_to_wait, time_to_wait);

                    curr_z -= diff_z;

                    if(i == solution->startPts.size() - 1 && drone_ == 4){
                        center = octomap::point3d(curr_x, curr_y, curr_z);
                    }
                    if(i == solution->startPts.size() - 1 && drone_ == 5){
                        start = Eigen::Vector4d(curr_x, curr_y, curr_z, 0);
                    }
                    pos_[drone_-1] = Eigen::Vector3d(curr_x, curr_y, curr_z); 
                }
            }
            prev_z = curr_z;
            rclcpp::sleep_for(std::chrono::seconds(time_to_wait));
        }
        
        planner->setCenter(center);
        planner->setPosition(pos_);

        /* DEBUG STATEMENTS
        std::cout << "pos_ size: " << pos_.size() << std::endl;
        for(int k = 0; k < pos_.size(); k++){
            std::cout << pos_[k].x() << " " << pos_[k].y() << " " << pos_[k].z() << std::endl;
        }

        std::vector<Eigen::Vector3d> poses = {Eigen::Vector3d(16.644729, 14.208915, 20.504419),Eigen::Vector3d(16.644729, 41.814807, 24.564109),Eigen::Vector3d(28.011861, 66.984885, 25.376047),Eigen::Vector3d(13.396977, 56.429691, 2.229845)};
        octomap::point3d cemter_(poses[3].x(), poses[3].y(), poses[3].z());
        Eigen::Vector4d start_(13.396977, 56.429691, 2.429845, 0);
        Eigen::Vector4d goal_(19.892481, 74.292327, 14.208915, -1);
        planner->runPlanner(start_, goal_);
        */

        for(uint i = 0; i < solution->toVisit[4].size(); i++){
            auto goalxyz = solution->toVisit[4][i].first;
            double yaw = getYaw(solution->toVisit[4][i].second);
            
            goal << goalxyz.x(), goalxyz.y(), goalxyz.z(), static_cast<double>(yaw);
            std::cout << "Pushing to producer queue" << std::endl;
            plannerProducerQueue.push({start, goal});
            start = goal;
        }

        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this] { return finishedCount == solution->toVisit[4].size(); });
        RCLCPP_INFO(this->get_logger(), "Traversed all points!");

        int idx = 0;
        while(!st.empty()){
            time_to_wait = 0;
            if(!st.empty()) idx = st.top().first;
            while(!st.empty() && st.top().first == idx){
                auto curr = st.top(); st.pop();
                // std::cout << "[" << curr.first << "]" << ":" << "(" << curr.second[0] << "," << curr.second[1] << "," << curr.second[2] << ")" << std::endl;
                curr_time_to_wait = this->go_to(curr.second[0], curr.second[1], curr.second[2], curr.second[3], 0);
                time_to_wait = std::max(curr_time_to_wait, time_to_wait);
            }
            rclcpp::sleep_for(std::chrono::seconds(time_to_wait));
        }
        return 0;
    }

    ~CrazyflieCommandClient(){
        delete tree;
    }

private:
    void compute_callback(){
    //    RCLCPP_INFO(this->get_logger(), "Computing solution...");
       solver->mainLogic();
    }

    void planner_producer_callback(){
        auto curr = plannerProducerQueue.pop();
        // std::cout << "Inside planner_producer: " << std::endl;
        planner->runPlanner(curr.first, curr.second);
    }

    void planner_consumer_callback(){
        auto curr = plannerConsumerQueue.pop();
        long long curr_time_to_wait = 0;
        // std::cout << "Inside planner_consumer: [" << 5 << "]" << ":" << "(" << curr[0] << "," << curr[1] << "," << curr[2] <<  "," << curr[3] << ")" << std::endl;
        curr_time_to_wait = this->go_to(5, curr[0], curr[1], curr[2], curr[3]);
        rclcpp::sleep_for(std::chrono::seconds(curr_time_to_wait));
        {
            std::lock_guard<std::mutex> lock(mtx);
            ++finishedCount;
        }
        cv.notify_one();
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
    double getYaw(double yaw) {
        static const double yawValues[] = {
            M_PI / 4,   // 0
            M_PI / 2,   // 1
            3*M_PI / 4, // 2
            0,          // 3
            -M_PI / 2,  // 4
            M_PI,       // 5
            -M_PI / 4,  // 6
            -M_PI / 2,  // 7
            -3*M_PI / 4 // 8
        };

        int index = static_cast<int>(yaw);
        if (index < 0 || index > 8) {
            throw std::runtime_error("yaw is not from [0-8]");
        }

        return yawValues[index];
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
    boost::mutex *mutex_ptr;

    std::mutex mtx;
    std::condition_variable cv;
    utils::sharedQueue<Eigen::Vector4d> plannerConsumerQueue;
    utils::sharedQueue<std::pair<Eigen::Vector4d, Eigen::Vector4d>> plannerProducerQueue;
    uint finishedCount = 0;
    octomap::point3d center;
    Eigen::Vector4d start;
    Eigen::Vector4d goal;

    std::vector<geometry_msgs::msg::Point> odom_linear;
    std::vector<geometry_msgs::msg::Quaternion> odom_quat;

    rclcpp::CallbackGroup::SharedPtr service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr solution_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr aruco_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr run_mission_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr planner_producer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr planner_consumer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions aruco_cb_options;

    rclcpp::TimerBase::SharedPtr aruco_timer_;
    rclcpp::TimerBase::SharedPtr compute_timer_;
    rclcpp::TimerBase::SharedPtr run_mission_timer_;
    rclcpp::TimerBase::SharedPtr planner_producer_timer_;
    rclcpp::TimerBase::SharedPtr planner_consumer_timer_;

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