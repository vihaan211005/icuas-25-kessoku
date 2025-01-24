
#include <octomap/octomap.h>
#include <vector>
#include <iostream>
#include <queue>
#include <cmath>
#include <Eigen/Dense>
#define Array3D vector<vector<vector<int>>>

using namespace std;
using namespace octomap;
using Eigen::Vector3d;
using Eigen::Vector3i;

class Bounds
{
public:
    Vector3d min, max;
    Bounds(Vector3d min = Vector3d(0, 0, 0), Vector3d max = Vector3d(0, 0, 0)) : min(min), max(max) {}

    friend ostream &operator<<(ostream &os, const Bounds &b)
    {
        os << "Bounds(" << b.min << ", " << b.max << ")";
        return os;
    }
};

class Solution
{
public:
    bool flag = false;
    vector<Vector3d> path;
    vector<Vector3d> final;

    Solution() : flag(false) {}
};

class DronePos
{
public:
    Vector3i pos;
    int yaw;
    double theta;
    double phai;
    DronePos(Vector3i pos = Vector3i(0, 0, 0), int yaw = 0, double theta = 0, double phai = 0) : pos(pos), yaw(yaw), theta(theta), phai(phai) {}
};

class Solver
{
public:
    Solver(Vector3d base, OcTree octree, int radius) : baseStation(baseStation), octree(octree), radius(radius) {}

private:
    Vector3d baseStation;
    OcTree octree;
    int radius;

    Bounds mapBounds;
    Array3D binaryArray;
    double resolution;
    Vector3i dimArray;

    // Saves 3DArray to csv file
    void saveToCSV(string file_name)
    {
        ofstream outfile(file_name + ".csv");

        for (int z = 0; z < binaryArray[0][0].size(); ++z)
            for (int y = 0; y < binaryArray[0].size(); ++y)
            {
                for (int x = 0; x < binaryArray.size(); ++x)
                {
                    outfile << binaryArray[x][y][z];
                    if (x < binaryArray.size() - 1)
                        outfile << ",";
                }
                outfile << "\n";
            }
        // cout << "CSV Exported!" << endl;
    }

    // Converts OctTree to 3DArray
    void octreeToBinaryArray()
    {
        resolution = octree.getResolution();

        octree.getMetricMin(mapBounds.min.x(), mapBounds.min.y(), mapBounds.min.z());
        octree.getMetricMax(mapBounds.max.x(), mapBounds.max.y(), mapBounds.max.z());

        dimArray = ((mapBounds.max - mapBounds.min) / resolution).unaryExpr([](double val)
                                                                            { return static_cast<int>(std::round(val)); });

        binaryArray.resize(dimArray.x(), vector<vector<int>>(dimArray.y(), vector<int>(dimArray.z(), 0)));

        for (OcTree::leaf_iterator it = octree.begin_leafs(), end = octree.end_leafs(); it != end; ++it)
        {
            Vector3d point3d(it.getX(), it.getY(), it.getZ());

            if (it->getOccupancy() > 0.5)
            {
                double half_size = it.getSize() / 2.0;
                Vector3d leaf_min = point3d - Vector3d(half_size, half_size, half_size);
                Vector3d leaf_max = point3d + Vector3d(half_size, half_size, half_size);

                Vector3i start_idx = ((leaf_min - mapBounds.min) / resolution).unaryExpr([](double val)
                                                                                         { return static_cast<int>(std::round(val)); });
                Vector3i end_idx = ((leaf_max - mapBounds.min) / resolution).unaryExpr([](double val)
                                                                                       { return static_cast<int>(std::round(val)); });

                start_idx = start_idx.cwiseMax(Vector3i(0, 0, 0));
                end_idx = end_idx.cwiseMin(dimArray - Vector3i(1, 1, 1));

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
        int x, y = 0, z = dimArray.z() - 1;

        for (int i = 0; i < dimArray.x(); ++i)
            if (!binaryArray[i][0][dimArray.z() - 1])
            {
                x = i;
                break;
            }

        queue<Vector3i> queue;
        queue.push(Vector3i(x, y, z));

        while (!queue.empty())
        {
            Vector3i curr = queue.front();
            queue.pop();

            if (curr.x() < 0 || curr.x() >= dimArray.x() || curr.y() < 0 || curr.y() >= dimArray.y() || curr.z() < 0 || curr.z() >= dimArray.z() || binaryArray[curr.x()][curr.y()][curr.z()])
                continue;

            binaryArray[curr.x()][curr.y()][curr.z()] = 2;

            queue.push(curr - Vector3i(1, 0, 0));
            queue.push(curr + Vector3i(1, 0, 0));
            queue.push(curr - Vector3i(0, 1, 0));
            queue.push(curr + Vector3i(0, 1, 0));
            queue.push(curr - Vector3i(0, 0, 1));
            queue.push(curr + Vector3i(0, 0, 1));
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
        Vector3i newDim = dimArray / factor;

        mapBounds.max -= resolution * (((dimArray - newDim * factor).cast<double>()));
        resolution *= factor;
        dimArray = newDim;

        vector<vector<vector<int>>> reducedArray(dimArray.x(), vector<vector<int>>(dimArray.y(), vector<int>(dimArray.z(), 0)));

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

                    bool isOccupied = false;
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
        mapBounds.max += Vector3d(resolution, resolution, resolution);
        mapBounds.min.x() -= resolution;
        mapBounds.min.y() -= resolution;

        dimArray.x() += 2;
        dimArray.y() += 2;
        dimArray.z() += 1;

        vector<vector<vector<int>>> newArray(dimArray.x(), vector<vector<int>>(dimArray.y(), vector<int>(dimArray.z(), 0)));

        for (int i = 1; i < dimArray.x() - 1; ++i)
            for (int j = 1; j < dimArray.y() - 1; ++j)
                for (int k = 0; k < dimArray.z() - 1; ++k)
                    newArray[i][j][k] = binaryArray[i][j][k];
    }

    // Mark Horizontal faces as 3 and Verical as 2. Call after adding x,y buffer and above z
    void markFaces()
    {
        for (int i = 1; i < binaryArray.size() - 1; ++i)
            for (int j = 1; j < binaryArray[0].size() - 1; ++j)
                for (int k = 0; k < binaryArray[0][0].size() - 1; ++k)
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
                                        if (!z && !binaryArray[i + x][j + y][k + z])
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
                                binaryArray[i][j][k] = 2; // Mark as vertical face
                            else
                                binaryArray[i][j][k] = 3; // Mark as horizontal face
                        }
                    }
    }

    // Check LOS btwn 2 points
    bool check2points(Vector3i p1, Vector3i p2)
    {
        Vector3i dp = p1 - p2;
        int steps = max({abs(dp.x()), abs(dp.y()), abs(dp.z())});

        for (int i = 1; i <= steps; ++i)
        {
            Vector3i curr = p1 + dp * i / steps;

            if (binaryArray[curr.x()][curr.y()][curr.z()])
                return false;
        }

        return true;
    }

    // Returns points in LOS
    vector<Vector3i> pointsInLOS(Vector3i centre)
    {
        vector<Vector3i> pointsInSphere;
        vector<Vector3i> inLOS;

        for (int i = -radius; i <= radius; ++i)
            for (int j = -radius; j <= radius; ++j)
                for (int k = -radius; k <= radius; ++k)
                {
                    Vector3i curr = centre + Vector3i(i, j, k);

                    if (curr.x() < 0 || curr.x() >= dimArray.x() || curr.y() < 0 || curr.y() >= dimArray.y() || curr.z() < 0 || curr.z() >= dimArray.z())
                        continue;

                    double distance = sqrt(pow(i, 2) + pow(j, 2) + pow(k, 2));
                    if (distance <= radius)
                        pointsInSphere.push_back(curr);
                }

        for (auto &point : pointsInSphere)
        {
            bool isClearPath = true;
            Vector3i d = point - centre;
            int steps = max({abs(d.x()), abs(d.y()), abs(d.z())});

            for (int i = 1; i < steps; ++i)
            {
                Vector3i curr = centre + d * i / steps;

                if (binaryArray[curr.x()][curr.y()][curr.z()])
                {
                    isClearPath = false;
                    break;
                }
            }

            if (isClearPath)
                inLOS.push_back(point);
        }

        return inLOS;
    }

    // Return empty point in LOS
    Vector3i getRandomPointFromLOS(vector<Vector3i> &pts, Vector3i centre)
    {
        vector<pair<double, Vector3i>> weightedPts;

        for (auto &point : pts)
        {
            Vector3i d = point - centre;
            double distance = sqrt(pow(d.x(), 2) + pow(d.y(), 2) + pow(d.z(), 2));
            weightedPts.push_back(make_pair(distance, point));
        }

        double sumWeights = 0.0;
        for (auto &weightedPt : weightedPts)
        {
            sumWeights += weightedPt.first;
        }

        srand(time(0));
        double randomValue = (double)rand() / RAND_MAX * sumWeights;

        double cumulativeWeight = 0.0;

        for (auto &weightedPt : weightedPts)
        {
            cumulativeWeight += weightedPt.first;
            if (randomValue <= cumulativeWeight)
            {
                return weightedPt.second;
            }
        }

        // Return the last point if no point is selected
        return weightedPts.back().second;
    }

    // Return adjacent point of a point
    DronePos getAdjacentPoint(Vector3i point, Vector3i centre)
    {
        vector<DronePos> adjacentPoints = {
            DronePos(point - Vector3i(1, 0, 0), 1, atan2(point.y() - centre.y(), point.x() - 1 - centre.x()), acos((point.z() - centre.z()) / sqrt(pow(point.x() - 1 - centre.x(), 2) + pow(point.y() - centre.y(), 2) + pow(point.z() - centre.z(), 2)))),
            DronePos(point + Vector3i(1, 0, 0), 2, atan2(point.y() - centre.y(), point.x() + 1 - centre.x()), acos((point.z() - centre.z()) / sqrt(pow(point.x() + 1 - centre.x(), 2) + pow(point.y() - centre.y(), 2) + pow(point.z() - centre.z(), 2)))),
            DronePos(point - Vector3i(0, 1, 0), 3, atan2(point.y() - 1 - centre.y(), point.x() - centre.x()), acos((point.z() - centre.z()) / sqrt(pow(point.x() - centre.x(), 2) + pow(point.y() - 1 - centre.y(), 2) + pow(point.z() - centre.z(), 2)))),
            DronePos(point + Vector3i(0, 1, 0), 4, atan2(point.y() + 1 - centre.y(), point.x() - centre.x()), acos((point.z() - centre.z()) / sqrt(pow(point.x() - centre.x(), 2) + pow(point.y() + 1 - centre.y(), 2) + pow(point.z() - centre.z(), 2)))),
            DronePos(point + Vector3i(1, 1, 0), 5, atan2(point.y() + 1 - centre.y(), point.x() + 1 - centre.x()), acos((point.z() - centre.z()) / sqrt(pow(point.x() + 1 - centre.x(), 2) + pow(point.y() + 1 - centre.y(), 2) + pow(point.z() - centre.z(), 2)))),
            DronePos(point + Vector3i(1, -1, 0), 6, atan2(point.y() - 1 - centre.y(), point.x() + 1 - centre.x()), acos((point.z() - centre.z()) / sqrt(pow(point.x() + 1 - centre.x(), 2) + pow(point.y() - 1 - centre.y(), 2) + pow(point.z() - centre.z(), 2)))),
            DronePos(point + Vector3i(-1, 1, 0), 7, atan2(point.y() + 1 - centre.y(), point.x() - 1 - centre.x()), acos((point.z() - centre.z()) / sqrt(pow(point.x() - 1 - centre.x(), 2) + pow(point.y() + 1 - centre.y(), 2) + pow(point.z() - centre.z(), 2)))),
            DronePos(point + Vector3i(-1, -1, 0), 8, atan2(point.y() - 1 - centre.y(), point.x() - 1 - centre.x()), acos((point.z() - centre.z()) / sqrt(pow(point.x() - 1 - centre.x(), 2) + pow(point.y() - 1 - centre.y(), 2) + pow(point.z() - centre.z(), 2))))};

        for (auto &dronepos : adjacentPoints)
        {

            if (binaryArray[dronepos.pos.x()][dronepos.pos.y()][dronepos.pos.z()] == 0)
                return dronepos;
        }

        return DronePos();
    }
};

int main()
{
    return 0;
}