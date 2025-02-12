#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <octomap/octomap.h>
#include <Eigen/Dense>
#include <omp.h>  // OpenMP header
#include <unistd.h>
#include <chrono>
struct Point {
    int x, y, z;
};

class Bounds {
public:
    Eigen::Vector3d min, max;
    Bounds(Eigen::Vector3d min = Eigen::Vector3d(0, 0, 0), Eigen::Vector3d max = Eigen::Vector3d(0, 0, 0))
        : min(min), max(max) {}

    friend std::ostream &operator<<(std::ostream &os, const Bounds &b) {
        os << "Bounds(" << b.min << ", " << b.max << ")";
        return os;
    }
};

octomap::OcTree octree("city_1.binvox.bt");
Bounds mapBounds;

Eigen::Vector3d indexToPoint(Eigen::Vector3i index) {
    double resolution = octree.getResolution();
    octree.getMetricMin(mapBounds.min.x(), mapBounds.min.y(), mapBounds.min.z());
    octree.getMetricMax(mapBounds.max.x(), mapBounds.max.y(), mapBounds.max.z());
    return index.cast<double>() * resolution + mapBounds.min;
}

bool check2points_octree(Eigen::Vector3i p1, Eigen::Vector3i p2) {
    if (p1 == p2)
        return true;
    Eigen::Vector3d startidx = indexToPoint(p1);
    Eigen::Vector3d endidx = indexToPoint(p2);
    octomap::point3d start(startidx.x(), startidx.y(), startidx.z());
    octomap::point3d end(endidx.x(), endidx.y(), endidx.z());
    octomap::point3d hit;
    bool hits = octree.castRay(start, end - start, hit, true, (end - start).norm() - 0.01);
    return !hits;
}
long long total_time = 0;

// Parallel function to process edges for a single vertex
void processEdgesForVertex(size_t i, const std::vector<Eigen::Vector3i>& vertices, std::vector<std::vector<Eigen::Vector3i>>& adjacencyList) {
    std::vector<Eigen::Vector3i> local_neighbors;

    auto start = std::chrono::high_resolution_clock::now();

    for (size_t j = i + 1; j < vertices.size(); ++j) {
        if (check2points_octree(vertices[i], vertices[j])) {
            local_neighbors.push_back(vertices[j]);  // Store actual Vector3i instead of index
            // std::cout << "Joining " << vertices[i].transpose() << " and " << vertices[j].transpose() << std::endl;
        }
    }
    
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    total_time += duration.count();
    

    // Merge local adjacency list (critical section for thread safety)
    #pragma omp critical
    adjacencyList[i].insert(adjacencyList[i].end(), local_neighbors.begin(), local_neighbors.end());

}

int main() {
    std::ifstream file("updated_array.csv");
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file!\n";
        return 1;
    }

    std::string line;
    std::getline(file, line); // Skip the header line
    std::vector<Point> points;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        int x, y, z, value;

        if (std::getline(ss, token, ',')) x = std::stoi(token);
        if (std::getline(ss, token, ',')) y = std::stoi(token);
        if (std::getline(ss, token, ',')) z = std::stoi(token);
        if (std::getline(ss, token, ',')) value = std::stoi(token);

        if (value == 6) {
            points.push_back({x, y, z});
        }
    }

    file.close();

    std::vector<Eigen::Vector3i> vertices;
    for (const auto &p : points) {
        vertices.emplace_back(p.x, p.y, p.z);
    }

    std::cout << "Vertices size: " << vertices.size() << std::endl;

    std::vector<std::vector<Eigen::Vector3i>> adjacencyList(vertices.size());  // Adjacency list representation

    // Parallel loop using OpenMP
    #pragma omp parallel for num_threads(4)
    for (size_t i = 0; i < vertices.size(); ++i) {
        processEdgesForVertex(i, vertices, adjacencyList);
    }
    

    std::cout<<"Total_time: "<<total_time<<std::endl;
    
    
    return 0;



    std::cout << "Graph constructed successfully!" << std::endl;

    // Print adjacency list
    for (size_t i = 0; i < adjacencyList.size(); ++i) {
        std::cout << "Vertex " << vertices[i].transpose() << " -> ";
        for (const auto& neighbor : adjacencyList[i]) {
            std::cout << neighbor.transpose() << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
