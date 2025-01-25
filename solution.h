#ifndef SOLUTION_H
#define SOLUTION_H

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
    Bounds(Vector3d min, Vector3d max);
    friend ostream &operator<<(ostream &os, const Bounds &b);
};

class DronePos
{
public:
    Vector3i pos;
    int yaw;
    double theta;
    double phai;
    DronePos(Vector3i pos, int yaw, double theta, double phai);
    bool operator<(const DronePos &other) const;
};

class Solution
{
public:
    vector<Vector3d> startPts;
    vector<vector<pair<Vector3d, int>>> toVisit;
    vector<vector<int>> toBreak;
    int eval;
    Solution();
};

class Solver
{
public:
    Solution solution;
    Solver(Vector3d base, OcTree octree, int radius, int num_drones);
    void initialSetup();
    void mainLogic();
private:
    Vector3d baseStation;
    OcTree octree;
    int radius;

    Bounds mapBounds;
    Array3D binaryArray;
    double resolution;
    Vector3i dimArray;

    // Converts indextopoint
    Vector3d indexToPoint(Vector3i index);
    // Saves 3DArray to csv file
    void saveToCSV(string file_name);
    // Converts OctTree to 3DArray
    void octreeToBinaryArray();
    // Fills the interior of buildings as occupied
    void markInterior();
    // Reduce resolution by factor
    void reduceResolution(int factor);
    // Add buffer
    void addBoundary();
    // Mark Horizontal faces as 3 and Verical as 2. Call after adding x,y buffer and above z
    void markFaces();
    // Check LOS btwn 2 points
    bool check2points(Vector3i p1, Vector3i p2);
    // Returns points in LOS
    vector<Vector3i> pointsInLOS(Vector3i centre);
    // Return empty point in LOS
    Vector3i getRandomPointFromLOS(vector<Vector3i> &pts, Vector3i centre);
    // Return adjacent point of a point
    DronePos getAdjacentPoint(Vector3i point, Vector3i centre);    
}
#endif
