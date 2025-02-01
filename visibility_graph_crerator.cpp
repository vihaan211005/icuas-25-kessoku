#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <octomap/OcTree.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/base/StateValidityChecker.h>

using namespace Eigen;
namespace ob = ompl::base;

struct MapBounds {
    Vector3d min;
};

double resolution = 1.0; // Define resolution value
MapBounds mapBounds = {Vector3d(0.0, 0.0, 0.0)}; // Define map bounds

class OctoMapValidityChecker : public ob::StateValidityChecker {
public:
    OctoMapValidityChecker(const ob::SpaceInformationPtr &si, const octomap::OcTree &octree)
        : ob::StateValidityChecker(si), octree_(octree) {}

    bool isValid(const ob::State *state) const override {
        const auto *realState = state->as<ob::RealVectorStateSpace::StateType>();
        double x = realState->values[0];
        double y = realState->values[1];
        double z = realState->values[2];

        octomap::OcTreeNode *node = octree_.search(x, y, z);
        return (node == nullptr || octree_.isNodeOccupied(node) == false);
    }

    bool isValid(const ob::State *state, double x_target, double y_target, double z_target, bool check_line_of_sight) const {
        if (!isValid(state)) {
            return false;
        }
        return true;
    }

private:
    const octomap::OcTree &octree_;
};

Vector3d indexToPoint(Vector3i index) {
    return index.cast<double>() * resolution + mapBounds.min;
}

struct Point {
    double x, y, z;
};

void depthLimitedDFS(const std::vector<std::vector<int>> &graph, int node, int depth, std::vector<int> &path,
                      std::vector<std::vector<std::vector<double>>> &dfs_paths, const std::vector<Point> &points) {
    if (depth == 5) {
        std::vector<std::vector<double>> row;
        for (int idx : path) {
            row.push_back({points[idx].x, points[idx].y, points[idx].z});
        }
        dfs_paths.push_back(row);
        return;
    }

    for (int neighbor : graph[node]) {
        path.push_back(neighbor);
        depthLimitedDFS(graph, neighbor, depth + 1, path, dfs_paths, points);
        path.pop_back();
    }
}


int main() {
    std::ifstream file("updated_array.csv");
    if (!file.is_open()) {
        std::cerr << "Error opening file!\n";
        return 1;
    }

    // Initialize octomap inside main()
    std::string octomapFile = "city_1.binvox.bt";
    octomap::OcTree octree(octomapFile);

    std::vector<Point> points;
    std::string line;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        int ix, iy, iz, value;
        char comma;
        
        if (ss >> ix >> comma >> iy >> comma >> iz >> comma >> value) {
            if (value == 6) {
                Vector3d transformed = indexToPoint(Vector3i(ix, iy, iz));
                points.push_back({transformed.x(), transformed.y(), transformed.z()});
            }
        }
    }
    file.close();

    std::vector<std::vector<int>> adjacencyList(points.size());
    
    ob::SpaceInformationPtr si;
    OctoMapValidityChecker checker(si, octree);  // Pass octree instance to checker

    for (size_t i = 0; i < points.size(); ++i) {
        for (size_t j = 0; j < points.size(); ++j) {
            if (i != j) {
                ob::RealVectorStateSpace::StateType state;
                state.values[0] = points[i].x;
                state.values[1] = points[i].y;
                state.values[2] = points[i].z;
                
                if (checker.isValid(&state, points[j].x, points[j].y, points[j].z, true)) {
                    adjacencyList[i].push_back(j);
                }
            }
        }
    }

    std::cout << "Graph adjacency list with line-of-sight constraints:\n";
    for (size_t i = 0; i < adjacencyList.size(); ++i) {
        std::cout << "Point " << i << " (" << points[i].x << ", " << points[i].y << ", " << points[i].z << ") -> ";
        for (int neighbor : adjacencyList[i]) {
            std::cout << neighbor << " ";
        }
        std::cout << "\n";
    }

    std::vector<std::vector<std::vector<double>>> dfs_paths;
    for (size_t i = 0; i < points.size(); ++i) {
        std::vector<int> path = {static_cast<int>(i)};
        depthLimitedDFS(adjacencyList, i, 1, path, dfs_paths, points);
    }
    
    std::cout << "DFS Paths (limited to depth 5):\n";
    for (const auto &path : dfs_paths) {
        for (const auto &point : path) {
            std::cout << "(" << point[0] << ", " << point[1] << ", " << point[2] << ") ";
        }
        std::cout << "\n";
    }
    
    return 0;
}

