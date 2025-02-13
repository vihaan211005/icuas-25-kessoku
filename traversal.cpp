#include <octomap/octomap.h>
#include <vector>
#include <iostream>
#include <queue>
#include <cmath>
#include <Eigen/Dense>
#include <chrono>
#include <boost/thread.hpp>

#define Array3D std::vector<std::vector<std::vector<int>>>
#define BoolArray3D std::vector<std::vector<std::vector<bool>>>

class Bounds
{
public:
    Eigen::Vector3d min, max;
    Bounds(Eigen::Vector3d min = Eigen::Vector3d(0, 0, 0), Eigen::Vector3d max = Eigen::Vector3d(0, 0, 0)) : min(min), max(max) {}

    friend std::ostream &operator<<(std::ostream &os, const Bounds &b)
    {
        os << "Bounds(" << b.min << ", " << b.max << ")";
        return os;
    }
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
    std::vector<Eigen::Vector3d> startPts; // dont use the startPt[0]; set this as the spawn position
    std::vector<std::vector<std::pair<Eigen::Vector3d, int>>> toVisit;

    std::vector<std::vector<int>> toBreak; // visit idx and go back

    int eval;  // > 0
    bool flag; // false

    Solution() : eval(0), flag(false) {}
};

class Solver
{
public:
    Solution solution;
    boost::mutex param_mutex;
    Bounds mapBounds;

    Solver();
    Solver(Eigen::Vector3d base, octomap::OcTree octree, int radius, int num_drones) : baseStation(base), octree(octree), radius(radius), num_drones(num_drones) {}

    void initialSetup()
    {
        octreeToBinaryArray();
        reduceResolution(2);
        markInterior();
        for (int i = 0; i < 12; ++i)
            addBoundary();
        markFaces();
        markCorner();
        makeAdjacency();
        runBFS();
        visited.resize(dimArray.x(), std::vector<std::vector<bool>>(dimArray.y(), std::vector<bool>(dimArray.z(), false)));

        int n = nodes_graph.size();
        for (int i = 0; i < n; i++)
        {
            if (distance[i] < 5)
            {
                binaryArray[nodes_graph[i].x()][nodes_graph[i].y()][nodes_graph[i].z()] = 4;
                auto get = getAdjacentFace(i);
                std::cout << get.first.size() << std::endl;
                for (auto j : get.first)
                {
                    binaryArray[j.pos.x()][j.pos.y()][j.pos.z()] = 5;
                }
                for (auto j : get.second)
                {
                    binaryArray[j.pos.x()][j.pos.y()][j.pos.z()] = 6;
                }
            }
            else
                binaryArray[nodes_graph[i].x()][nodes_graph[i].y()][nodes_graph[i].z()] = 7;
        }

        saveToCSV("first");
        std::cout << "mapBounds: " << mapBounds << std::endl;
        std::cout << "dimArray: " << dimArray << std::endl;
    }

private:
    Eigen::Vector3d baseStation;
    octomap::OcTree octree;
    int radius;
    int num_drones;

    Array3D binaryArray;
    BoolArray3D visited;
    double resolution;
    Eigen::Vector3i dimArray;

    std::vector<std::tuple<int, int, Eigen::Vector3i>> node_dirs{std::make_tuple(0, 0, Eigen::Vector3i(0, 0, 0))};

    std::vector<Eigen::Vector3i> nodes_graph{Eigen::Vector3i(0, 0, 0)};
    std::vector<int> distance;
    std::vector<std::vector<bool>> adjacency_matrix;

    std::pair<std::vector<DronePos>, std::vector<DronePos>> getAdjacentFace(int node)
    {
        int dirx = std::get<0>(node_dirs[node]);
        int diry = std::get<1>(node_dirs[node]);

        if (!dirx && !diry)
            return std::make_pair(std::vector<DronePos>(), std::vector<DronePos>());

        std::pair<std::vector<DronePos>, std::vector<DronePos>> solution;

        int x = std::get<2>(node_dirs[node]).x();
        int y = std::get<2>(node_dirs[node]).y();
        int z = std::get<2>(node_dirs[node]).z();

        int i = 0;
        int j = 0;

        while (1)
        {
            i = 0;
            while (1)
            {
                if (check2points_octree(Eigen::Vector3i(x + dirx, y + i, z + j), nodes_graph[node], radius) && binaryArray[x][y + i][z + j] == 2 && (!visited[x][y + i][z + j] || !i))
                {
                    solution.first.push_back(DronePos(Eigen::Vector3i(x + dirx, y + i, z + j), (dirx + 1) >> 1));
                    visited[x][y + i][z + j] = true;
                }
                else
                    break;
                diry == 1 ? i-- : i++;
            }
            if (!i || !(z + j))
                break;
            j--;
        }

        j = 0;
        while (1)
        {
            i = 0;
            while (1)
            {
                if (check2points_octree(Eigen::Vector3i(x + i, y + diry, z + j), nodes_graph[node], radius) && binaryArray[x + i][y][z + j] == 2 && (!visited[x + i][y][z + j] || !i))
                {
                    solution.second.push_back(DronePos(Eigen::Vector3i(x + i, y + diry, z + j), (diry + 5) >> 1));
                    visited[x + i][y][z + j] = true;
                }
                else
                    break;
                dirx == 1 ? i-- : i++;
            }
            if (!i || !(z + j))
                break;
            j--;
        }

        return solution;
    }

    void runBFS()
    {
        int n = nodes_graph.size();
        std::queue<int> queue;
        std::vector<bool> visited(n, false);
        distance.resize(n, INT_MAX);

        queue.push(0);
        visited[0] = true;
        distance[0] = 0;

        std::vector<std::vector<bool>> adjacency_matrix_new(n, std::vector<bool>(n, 0));

        int count = 0;

        while (!queue.empty() && distance[queue.front()] < num_drones - 1)
        {
            int node = queue.front();
            Eigen::Vector3i curr = nodes_graph[node];
            queue.pop();

            for (int i = 0; i < n; ++i)
            {
                if (adjacency_matrix[node][i] && !visited[i])
                {
                    queue.push(i);
                    visited[i] = true;
                    distance[i] = distance[node] + 1;
                    adjacency_matrix_new[i][node] = 1;
                    adjacency_matrix_new[node][i] = 1;
                    count++;
                }
            }
        }
        adjacency_matrix = adjacency_matrix_new;

        std::cout << "Number of edges: " << count << std::endl;
    }

    void makeAdjacency()
    {
        int n = nodes_graph.size();
        int count = 0;
        adjacency_matrix.resize(n, std::vector<bool>(n, false));
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                if (i != j && check2points_octree(nodes_graph[i], nodes_graph[j], radius))
                {
                    adjacency_matrix[i][j] = 1;
                    count++;
                }
        std::cout << "Number of edges: " << count << std::endl;
    }

    void markCorner()
    {
        for (int i = 0; i < dimArray.x(); ++i)
            for (int j = 0; j < dimArray.y(); ++j)
                for (int k = 0; k < dimArray.z(); ++k)
                    if (binaryArray[i][j][k] == 2)
                    {
                        bool adjacent[6] = {0, 0, 0, 0, 0, 0};
                        if (i > 0)
                            adjacent[0] = binaryArray[i - 1][j][k];
                        if (i < dimArray.x() - 1)
                            adjacent[1] = binaryArray[i + 1][j][k];
                        if (j > 0)
                            adjacent[2] = binaryArray[i][j - 1][k];
                        if (j < dimArray.y() - 1)
                            adjacent[3] = binaryArray[i][j + 1][k];
                        if (k > 0)
                            adjacent[4] = binaryArray[i][j][k - 1];
                        if (k < dimArray.z() - 1)
                            adjacent[5] = binaryArray[i][j][k + 1];

                        int dirx, diry;
                        adjacent[1] ? dirx = -1 : dirx = 1;
                        adjacent[3] ? diry = -1 : diry = 1;

                        int total = 0;
                        for (int x = 0; x < 6; x++)
                            if (adjacent[x] == 1)
                                total++;

                        if (total == 3 && !adjacent[5] && adjacent[4] && !binaryArray[i + dirx][j + diry][k + 1])
                        {
                            nodes_graph.push_back(Eigen::Vector3i(i + dirx, j + diry, k + 1));
                            node_dirs.push_back(std::make_tuple(dirx, diry, Eigen::Vector3i(i, j, k)));
                        }
                    }
    }

    // Converts indextopoint
    Eigen::Vector3d indexToPoint(Eigen::Vector3i index)
    {
        return index.cast<double>() * resolution + mapBounds.min;
    }

    // Saves 3DArray to csv file
    void saveToCSV(std::string file_name)
    {
        std::ofstream outfile(file_name + ".csv");

        for (int z = 0; z < dimArray.z(); ++z)
            for (int y = 0; y < dimArray.y(); ++y)
            {
                for (int x = 0; x < dimArray.x(); ++x)
                {
                    outfile << binaryArray[x][y][z];
                    if (x < dimArray.x() - 1)
                        outfile << ",";
                }
                outfile << "\n";
            }
        std::cout << "CSV Exported!" << std::endl;
    }

    // Converts OctTree to 3DArray
    void octreeToBinaryArray()
    {
        resolution = octree.getResolution();

        octree.getMetricMin(mapBounds.min.x(), mapBounds.min.y(), mapBounds.min.z());
        octree.getMetricMax(mapBounds.max.x(), mapBounds.max.y(), mapBounds.max.z());

        dimArray = ((mapBounds.max - mapBounds.min) / resolution).unaryExpr([](double val)
                                                                            { return static_cast<int>(std::round(val)); });

        binaryArray.resize(dimArray.x(), std::vector<std::vector<int>>(dimArray.y(), std::vector<int>(dimArray.z(), 0)));

        for (octomap::OcTree::leaf_iterator it = octree.begin_leafs(), end = octree.end_leafs(); it != end; ++it)
        {
            Eigen::Vector3d point3d(it.getX(), it.getY(), it.getZ());

            if (it->getOccupancy() > 0.5)
            {
                double half_size = it.getSize() / 2.0;
                Eigen::Vector3d leaf_min = point3d - Eigen::Vector3d(half_size, half_size, half_size);
                Eigen::Vector3d leaf_max = point3d + Eigen::Vector3d(half_size, half_size, half_size);

                Eigen::Vector3i start_idx = ((leaf_min - mapBounds.min) / resolution).unaryExpr([](double val)
                                                                                                { return static_cast<int>(std::round(val)); });
                Eigen::Vector3i end_idx = ((leaf_max - mapBounds.min) / resolution).unaryExpr([](double val)
                                                                                              { return static_cast<int>(std::round(val)); });

                start_idx = start_idx.cwiseMax(Eigen::Vector3i(0, 0, 0));
                end_idx = end_idx.cwiseMin(dimArray - Eigen::Vector3i(1, 1, 1));

                // start_idx_x = max(0, start_idx_x);
                // end_idx_x = min(dimArray.x - 1, end_idx_x);
                // start_idx_y = max(0, start_idx_y);
                // end_idx_y = min(dimArray.y - 1, end_idx_y);
                // start_idx_z = max(0, start_idx_z);
                // end_idx_z = min(dimArray.z - 1, end_idx_z);
                // Check if required

                for (int i = start_idx.x(); i <= end_idx.x(); ++i)
                    for (int j = start_idx.y(); j <= end_idx.y(); ++j)
                        for (int k = start_idx.z(); k <= end_idx.z(); ++k)
                            binaryArray[i][j][k] = 1;
            }
        }
    }

    // Fills the interior of buildings as occupied
    void markInterior()
    {
        int x = 0, y = 0, z = 0;

        std::queue<Eigen::Vector3i> queue;
        queue.push(Eigen::Vector3i(x, y, z));

        while (!queue.empty())
        {
            Eigen::Vector3i curr = queue.front();
            queue.pop();

            if (curr.x() < 0 || curr.x() >= dimArray.x() || curr.y() < 0 || curr.y() >= dimArray.y() || curr.z() < 0 || curr.z() >= dimArray.z() || binaryArray[curr.x()][curr.y()][curr.z()])
                continue;

            binaryArray[curr.x()][curr.y()][curr.z()] = 2;

            queue.push(curr - Eigen::Vector3i(1, 0, 0));
            queue.push(curr + Eigen::Vector3i(1, 0, 0));
            queue.push(curr - Eigen::Vector3i(0, 1, 0));
            queue.push(curr + Eigen::Vector3i(0, 1, 0));
            queue.push(curr - Eigen::Vector3i(0, 0, 1));
            queue.push(curr + Eigen::Vector3i(0, 0, 1));
        }

        for (int i = 0; i < dimArray.x(); ++i)
            for (int j = 0; j < dimArray.y(); ++j)
                for (int k = 0; k < dimArray.z(); ++k)
                {
                    if (binaryArray[i][j][k] == 0)
                        binaryArray[i][j][k] = 1;
                    if (binaryArray[i][j][k] == 2)
                        binaryArray[i][j][k] = 0;
                }
    }

    // Reduce resolution by factor
    void reduceResolution(int factor)
    {
        Eigen::Vector3i newDim = dimArray / factor;

        mapBounds.max -= resolution * (((dimArray - newDim * factor).cast<double>()));
        resolution *= factor;
        dimArray = newDim;

        std::vector<std::vector<std::vector<int>>> reducedArray(dimArray.x(), std::vector<std::vector<int>>(dimArray.y(), std::vector<int>(dimArray.z(), 0)));

        for (int i = 0; i < dimArray.x(); ++i)
            for (int j = 0; j < dimArray.y(); ++j)
                for (int k = 0; k < dimArray.z(); ++k)
                {
                    int startIdxX = i * factor;
                    int startIdxY = j * factor;
                    int startIdxZ = k * factor;
                    int endIdxX = (i + 1) * factor;
                    int endIdxY = (j + 1) * factor;
                    int endIdxZ = (k + 1) * factor;

                    for (int x = startIdxX; x < endIdxX; ++x)
                        for (int y = startIdxY; y < endIdxY; ++y)
                            for (int z = startIdxZ; z < endIdxZ; ++z)
                                if (binaryArray[x][y][z])
                                    reducedArray[i][j][k] = 1;
                }

        binaryArray = reducedArray;
    }

    // Add buffer
    void addBoundary()
    {
        mapBounds.max += Eigen::Vector3d(resolution, resolution, resolution);
        mapBounds.min.x() -= resolution;
        mapBounds.min.y() -= resolution;

        dimArray.x() += 2;
        dimArray.y() += 2;
        dimArray.z() += 1;

        std::vector<std::vector<std::vector<int>>> newArray(dimArray.x(), std::vector<std::vector<int>>(dimArray.y(), std::vector<int>(dimArray.z(), 0)));

        for (int i = 1; i < dimArray.x() - 1; ++i)
            for (int j = 1; j < dimArray.y() - 1; ++j)
                for (int k = 0; k < dimArray.z() - 1; ++k)
                    newArray[i][j][k] = binaryArray[i - 1][j - 1][k];

        binaryArray = newArray;
    }

    // Mark Horizontal faces as 3 and Verical as 2. Call after adding x,y buffer and above z
    void markFaces()
    {
        int total_vertical = 0;
        for (uint i = 1; i < binaryArray.size() - 1; ++i)
            for (uint j = 1; j < binaryArray[0].size() - 1; ++j)
                for (uint k = 0; k < binaryArray[0][0].size() - 1; ++k)
                    if (binaryArray[i][j][k] == 1)
                    {
                        if (!k)
                            binaryArray[i][j][k] = 3; // Mark as horizontal face on z=0 boundary
                        else
                        {
                            int plane[4] = {0, 0, 0, 0};
                            int total = 0;
                            for (int x = -1; x <= 1; ++x)
                                for (int y = -1; y <= 1; ++y)
                                    for (int z = -1; z <= 1; ++z)
                                    {
                                        if (binaryArray[i + x][j + y][k + z])
                                            total++;
                                        if (!z && !binaryArray[i + x][j + y][k + z] && (!x || !y))
                                            if (!z && !binaryArray[i + x][j + y][k + z] && (!x || !y))
                                            {
                                                if (x < 0)
                                                    plane[0]++;
                                                if (x > 0)
                                                    plane[1]++;
                                                if (y < 0)
                                                    plane[2]++;
                                                if (y > 0)
                                                    plane[3]++;
                                            }
                                    }
                            if (total == 27)
                                continue;
                            if (plane[0] || plane[1] || plane[2] || plane[3])
                            {
                                binaryArray[i][j][k] = 2; // Mark as vertical face
                                total_vertical++;
                            }
                            else
                                binaryArray[i][j][k] = 3; // Mark as horizontal face
                        }
                    }
        std::cout << "Total = " << total_vertical << std::endl;
    }

    // // Check LOS via octree
    bool check2points_octree(Eigen::Vector3i p1, Eigen::Vector3i p2, double radius = 10000)
    {
        if (p1 == p2)
            return true;
        Eigen::Vector3d startidx = indexToPoint(p1);
        Eigen::Vector3d endidx = indexToPoint(p2);
        if ((endidx - startidx).norm() > radius)
            return false;
        octomap::point3d start(startidx.x(), startidx.y(), startidx.z());
        octomap::point3d end(endidx.x(), endidx.y(), endidx.z());
        octomap::point3d hit;
        bool hits = octree.castRay(start, end - start, hit, true, (end - start).norm());
        return !hits;
    }
};

int main()
{
    Solver solver = Solver(Eigen::Vector3d(0, 0, 0), octomap::OcTree("city_1.binvox.bt"), 43, 5);
    solver.initialSetup();

    return 0;
}