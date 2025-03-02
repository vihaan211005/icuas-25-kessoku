#include <octomap/octomap.h>
#include <vector>
#include <iostream>
#include <queue>
#include <cmath>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <string>

#define Array3D std::vector<std::vector<std::vector<int>>>
#define BoolArray3D std::vector<std::vector<std::vector<bool>>>

class Bounds
{
public:
    Eigen::Vector3d min, max;
    Bounds(Eigen::Vector3d min = Eigen::Vector3d(0, 0, 0), Eigen::Vector3d max = Eigen::Vector3d(0, 0, 0)) : min(min), max(max) {}

    friend std::ostream &operator<<(std::ostream &os, const Bounds &b);
};

class DronePos
{
public:
    Eigen::Vector3i pos;
    int yaw;
    DronePos(Eigen::Vector3i pos = Eigen::Vector3i(0, 0, 0), int yaw = 0) : pos(pos), yaw(yaw) {}
};

class Solution
{
public:
    int n;
    std::vector<std::vector<bool>> adjacency_matrix;
    std::vector<int> parent;
    std::vector<int> distance;
    std::vector<Eigen::Vector3d> nodes_graph;

    std::vector<std::pair<int, std::pair<std::vector<Eigen::Vector4d>, std::vector<Eigen::Vector4d>>>> bfs_order;
};

class Solver
{
public:
    Solution solution;
    Bounds mapBounds;

    Solver();
    Solver(Eigen::Vector3d base, octomap::OcTree octree, int radius, int num_drones);

    void initialSetup();

private:
    Eigen::Vector3d baseStation;
    Eigen::Vector3i baseIndex;
    octomap::OcTree octree;
    int radius;
    int num_drones;

    Array3D binaryArray;
    BoolArray3D visited;
    double resolution;
    Eigen::Vector3i dimArray;

    std::vector<std::tuple<int, int, Eigen::Vector3i>> node_dirs;

    std::vector<Eigen::Vector3i> nodes_graph;
    std::vector<int> distance;
    std::vector<int> parent;
    std::vector<std::vector<bool>> adjacency_matrix;

    std::pair<std::vector<std::vector<DronePos>>, std::vector<std::vector<DronePos>>> getAdjacentFace(int node);
    void runBFS();
    void makeAdjacency();
    void saveEdgesToCSV(std::string file_name, std::vector<std::vector<bool>> adjacency_matrix);
    void markCorner();
    Eigen::Vector3d indexToPoint(Eigen::Vector3i index);
    void saveToCSV(std::string file_name);
    void octreeToBinaryArray();
    void markInterior();
    void reduceResolution(int factor);
    void addBoundary(int x_front = 1, int y_front = 1, int x_back = 1, int y_back = 1, int z_back = 1);
    void markFaces();
    bool check2points_octree(Eigen::Vector3i p1, Eigen::Vector3i p2, double radius = 10000);
    void check();
};
