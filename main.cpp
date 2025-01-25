#include <octomap/octomap.h>
#include <vector>
#include <iostream>
#include <queue>
#include <cmath>
#include <Eigen/Dense>
#include <chrono>

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

class DronePos
{
public:
    Vector3i pos;
    int yaw;
    double theta;
    double phai;
    DronePos(Vector3i pos = Vector3i(0, 0, 0), int yaw = 0, double theta = 0, double phai = 0) : pos(pos), yaw(yaw), theta(theta), phai(phai) {}

    bool operator<(const DronePos &other) const
    {
        if (theta != other.theta)
            return theta < other.theta;
        return phai < other.phai;
    }
};

class Solution
{
public:
    vector<Vector3d> startPts;
    vector<vector<pair<Vector3d, int>>> toVisit;
    vector<vector<int>> toBreak;
    vector<Vector3i> facesVisited;
    int eval;

    Solution() : eval(0) {}
};

class Solver
{
public:
    Solution solution;

    Solver(Vector3d base, OcTree octree, int radius, int num_drones) : baseStation(baseStation), octree(octree), radius(radius), num_drones(num_drones) {}

    void initialSetup()
    {
        octreeToBinaryArray();
        reduceResolution(2);
        markInterior();
        addBoundary();
        markFaces();

        saveToCSV("first");
        cout << "Dimensions : " << dimArray << endl;

        while (1)
        {
            for (int i = 0; i < 50; i++)
                mainLogic();
            cout << "Best Eval after 50 iter = " << solution.eval << endl;
            solution.eval = 0;
            for (int i = 0; i < solution.facesVisited.size(); i++)
            {
                binaryArray[solution.facesVisited[i].x()][solution.facesVisited[i].y()][solution.facesVisited[i].z()] = 3;
            }
        }
    }

    void mainLogic()
    {
        vector<Vector3i> startPts(num_drones);
        startPts[0] = Vector3i(0, 0, 0); // base
        vector<vector<DronePos>> poses(num_drones - 1);
        vector<vector<int>> toBreak(num_drones - 1);
        vector<Vector3i> facesVisited;

        int local_eval = 0;
        for (int i = 0; i < 4; i++)
        {
            vector<Vector3i> all_points = pointsInLOS(startPts[i]);
            vector<Vector3i> empty_points;

            int all = 0;
            int in_sight = 0;

            for (auto &point : all_points)
            {

                if (binaryArray[point.x()][point.y()][point.z()] == 2)
                {
                    all++;
                    DronePos dronepos = getAdjacentPoint(point, startPts[i]);
                    if (check2points(dronepos.pos, startPts[i]))
                    {
                        in_sight++;
                        local_eval++;
                        poses[i].push_back(dronepos);
                        facesVisited.push_back(point);
                    }
                }
                else if (binaryArray[point.x()][point.y()][point.z()] == 0)
                    empty_points.push_back(point);
            }
            startPts[i + 1] = getRandomPointFromLOS(empty_points, startPts[i]);

            sort(poses[i].begin(), poses[i].end());
            for (int j = 0; j < poses[i].size() - 1; j++)
                if (!check2points(poses[i][j].pos, poses[i][j+1].pos))
                    toBreak[i].push_back(j);

            cout<<"All "<<all<<endl;
            cout<<"LOS "<<in_sight<<endl;
            cout<<"brk "<<toBreak[i].size()<<endl;
        
        }
        vector<Vector3d> centres(num_drones);
        for (int i = 0; i < num_drones; i++)
        {
            centres[i] = indexToPoint(startPts[i]);
        }

        vector<vector<pair<Vector3d, int>>> toVisit(num_drones - 1);
        for (int i = 0; i < num_drones - 1; i++)
            for (auto dronepos : poses[i])
                toVisit[i].push_back(make_pair(indexToPoint(dronepos.pos), dronepos.yaw));

        if (solution.eval < local_eval)
        {
            solution.eval = local_eval;
            solution.startPts = centres;
            solution.toVisit = toVisit;
            solution.toBreak = toBreak;
            solution.facesVisited = facesVisited;
        }
    }

private:
    Vector3d baseStation;
    OcTree octree;
    int radius;
    int num_drones;

    Bounds mapBounds;
    Array3D binaryArray;
    double resolution;
    Vector3i dimArray;

    // Converts indextopoint
    Vector3d indexToPoint(Vector3i index)
    {
        return index.cast<double>() * resolution + mapBounds.min;
    }

    // Saves 3DArray to csv file
    void saveToCSV(string file_name)
    {
        ofstream outfile(file_name + ".csv");

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
        cout << "CSV Exported!" << endl;
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
        int x = 0, y = 0, z = 0;

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
                    newArray[i][j][k] = binaryArray[i - 1][j - 1][k];

        binaryArray = newArray;
    }

    // Mark Horizontal faces as 3 and Verical as 2. Call after adding x,y buffer and above z
    void markFaces()
    {
        int total_vertical = 0;
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
                            {
                                binaryArray[i][j][k] = 2; // Mark as vertical face
                                total_vertical++;
                            }
                            else
                                binaryArray[i][j][k] = 3; // Mark as horizontal face
                        }
                    }
        cout << "Total = " << total_vertical << endl;
    }

    // Check LOS btwn 2 points
    bool check2points(Vector3i p1, Vector3i p2)
    {
        Vector3i dp = p1 - p2;
        int steps = max({abs(dp.x()), abs(dp.y()), abs(dp.z())});

        for (int i = 1; i < steps; ++i)
        {
            Vector3i curr = p2 + dp * i / steps;

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

        srand(chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count());
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
        vector<DronePos> adjacentPoints;
        for (int i = -1; i <= 1; i++)
            for (int j = -1; j <= 1; j++)
            {
                if (!i & !j)
                    continue;
                Vector3i cur = point + Vector3i(i, j, 0);
                adjacentPoints.push_back(DronePos(cur, (i + 1) * 3 + (j + 1), atan2(cur.y(), cur.x()), acos((cur.z()) / sqrt(pow(cur.x(), 2) + pow(cur.y(), 2) + pow(cur.z(), 2)))));
            }

        for (auto &dronepos : adjacentPoints)
        {
            if (!binaryArray[dronepos.pos.x()][dronepos.pos.y()][dronepos.pos.z()])
                return dronepos;
        }

        cout << "Big Problem";
        return DronePos();
    }
};

int main()
{
    Solver solver = Solver(Vector3d(0, 0, 0), OcTree("city_1.binvox.bt"), 43, 5);
    solver.initialSetup();
}