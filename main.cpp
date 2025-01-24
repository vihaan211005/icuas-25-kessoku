#include <octomap/octomap.h>
#include <vector>
#include <iostream>
#include <queue>
#include <cmath>
#define Array3D vector<vector<vector<int>>>

using namespace std;
using namespace octomap;

class Point3D
{
public:
    double x, y, z;
    Point3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}

    Point3D operator+(const Point3D &p) const
    {
        return Point3D(x + p.x, y + p.y, z + p.z);
    }

    Point3D operator+(const double &p) const
    {
        return Point3D(x + p, y + p, z + p);
    }

    Point3D operator-(const Point3D &p) const
    {
        return Point3D(x - p.x, y - p.y, z - p.z);
    }

    Point3D operator-(const double &p) const
    {
        return Point3D(x - p, y - p, z - p);
    }

    Point3D operator/(const double &s) const
    {
        return Point3D(x / s, y / s, z / s);
    }

    Point3D operator*(const double &s) const
    {
        return Point3D(x * s, y * s, z * s);
    }

    Index3D nearestIndex()
    {
        return Index3D(round(x), round(y), round(z));
    }

    friend ostream &operator<<(ostream &os, const Point3D &p)
    {
        os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
        return os;
    }
};

class Index3D
{
public:
    int x, y, z;
    Index3D(int x = 0, int y = 0, int z = 0) : x(x), y(y), z(z) {}

    Index3D operator-(const Index3D &other) const
    {
        return Index3D(x - other.x, y - other.y, z - other.z);
    }

    Index3D operator+(const Index3D &other) const
    {
        return Index3D(x + other.x, y + other.y, z + other.z);
    }

    Index3D operator*(const int &s) const
    {
        return Index3D(x * s, y * s, z * s);
    }

    Point3D operator*(const double &s) const
    {
        return Point3D(x * s, y * s, z * s);
    }

    Index3D operator/(const int &s) const
    {
        return Index3D(x / s, y / s, z / s);
    }

    int maxInt()
    {
        return max(x, max(y, z));
    }

    int maxAbsInt()
    {
        return max(abs(x), max(abs(y), abs(z)));
    }

    double dist()
    {
        return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
    }

    friend ostream &operator<<(ostream &os, const Index3D &i)
    {
        os << "(" << i.x << ", " << i.y << ", " << i.z << ")";
        return os;
    }
};

class Bounds
{
public:
    Point3D min, max;
    Bounds(Point3D min = Point3D(), Point3D max = Point3D()) : min(min), max(max) {}

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
    vector<Point3D> path;
    vector<Point3D> final;

    Solution() : flag(false) {}
};

class DronePos
{
public:
    Index3D pos;
    int yaw;
    double theta;
    double phai;
    DronePos(Index3D pos = Index3D(), int yaw = 0, double theta = 0, double phai = 0) : pos(pos), yaw(yaw), theta(theta), phai(phai) {}
};

class Solver
{
public:
    Solver(Point3D base, OcTree octree, int radius) : baseStation(baseStation), octree(octree), radius(radius) {}

private:
    Point3D baseStation;
    OcTree octree;
    int radius;

    Bounds mapBounds;
    Array3D binaryArray;
    double resolution;
    Index3D dimArray;

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

        octree.getMetricMin(mapBounds.min.x, mapBounds.min.y, mapBounds.min.z);
        octree.getMetricMax(mapBounds.max.x, mapBounds.max.y, mapBounds.max.z);

        dimArray = ((mapBounds.max - mapBounds.min) / resolution).nearestIndex();

        binaryArray.resize(dimArray.x, vector<vector<int>>(dimArray.y, vector<int>(dimArray.z, 0)));

        for (OcTree::leaf_iterator it = octree.begin_leafs(), end = octree.end_leafs(); it != end; ++it)
        {
            Point3D point3d;
            point3d.x = it.getX();
            point3d.y = it.getY();
            point3d.z = it.getZ();

            if (it->getOccupancy() > 0.5) // doubt : can use isOccupied();
            {
                double half_size = it.getSize() / 2.0;
                Point3D leaf_min = point3d - half_size;
                Point3D leaf_max = point3d + half_size;

                Index3D start_idx = ((leaf_min - mapBounds.min) / resolution).nearestIndex();
                Index3D end_idx = ((leaf_max - mapBounds.min) / resolution).nearestIndex();

                // start_idx_x = max(0, start_idx_x);
                // end_idx_x = min(dimArray.x - 1, end_idx_x);
                // start_idx_y = max(0, start_idx_y);
                // end_idx_y = min(dimArray.y - 1, end_idx_y);
                // start_idx_z = max(0, start_idx_z);
                // end_idx_z = min(dimArray.z - 1, end_idx_z);
                // Check if required

                for (int i = start_idx.x; i <= end_idx.x; ++i)
                    for (int j = start_idx.y; j <= end_idx.y; ++j)
                        for (int k = start_idx.z; k <= end_idx.z; ++k)
                            binaryArray[i][j][k] = 1;
            }
        }
    }

    // Fills the interior of buildings as occupied
    void markInterior()
    {
        int x, y = 0, z = dimArray.z - 1;

        for (int i = 0; i < dimArray.x; ++i)
            if (!binaryArray[i][0][dimArray.z - 1])
            {
                x = i;
                break;
            }

        queue<Index3D> queue;
        queue.push(Index3D(x, y, z));

        while (!queue.empty())
        {
            Index3D curr = queue.front();
            queue.pop();

            if (curr.x < 0 || curr.x >= dimArray.x || curr.y < 0 || curr.y >= dimArray.y || curr.z < 0 || curr.z >= dimArray.z || binaryArray[curr.x][curr.y][curr.z])
                continue;

            binaryArray[curr.x][curr.y][curr.z] = 2;

            queue.push(curr - Index3D(1, 0, 0));
            queue.push(curr + Index3D(1, 0, 0));
            queue.push(curr - Index3D(0, 1, 0));
            queue.push(curr + Index3D(0, 1, 0));
            queue.push(curr - Index3D(0, 0, 1));
            queue.push(curr + Index3D(0, 0, 1));
        }

        for (int i = 0; i < dimArray.x; ++i)
            for (int j = 0; j < dimArray.y; ++j)
                for (int k = 0; k < dimArray.z; ++k)
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
        Index3D newDim = dimArray / factor;

        mapBounds.max = mapBounds.max - ((dimArray - newDim * factor) * resolution);
        resolution *= factor;
        dimArray = newDim;

        vector<vector<vector<int>>> reducedArray(dimArray.x, vector<vector<int>>(dimArray.y, vector<int>(dimArray.z, 0)));

        for (int i = 0; i < dimArray.x; ++i)
            for (int j = 0; j < dimArray.y; ++j)
                for (int k = 0; k < dimArray.z; ++k)
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
        mapBounds.max = mapBounds.max + resolution;
        mapBounds.min.x -= resolution;
        mapBounds.min.y -= resolution;

        dimArray.x += 2;
        dimArray.y += 2;
        dimArray.z += 1;

        vector<vector<vector<int>>> newArray(dimArray.x, vector<vector<int>>(dimArray.y, vector<int>(dimArray.z, 0)));

        for (int i = 1; i < dimArray.x - 1; ++i)
            for (int j = 1; j < dimArray.y - 1; ++j)
                for (int k = 0; k < dimArray.z - 1; ++k)
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
    bool check2points(Index3D p1, Index3D p2)
    {
        Index3D dp = p1 - p2;
        int steps = dp.maxInt();

        for (int i = 1; i <= steps; ++i)
        {
            Index3D curr = p1 + dp * i / steps;

            if (binaryArray[curr.x][curr.y][curr.z])
                return false;
        }

        return true;
    }

    // Returns points in LOS
    vector<Index3D> pointsInLOS(Index3D centre)
    {
        vector<Index3D> pointsInSphere;
        vector<Index3D> inLOS;

        for (int i = -radius; i <= radius; ++i)
            for (int j = -radius; j <= radius; ++j)
                for (int k = -radius; k <= radius; ++k)
                {
                    Index3D curr = centre + Index3D(i, j, k);

                    if (curr.x < 0 || curr.x >= dimArray.x || curr.y < 0 || curr.y >= dimArray.y || curr.z < 0 || curr.z >= dimArray.z)
                        continue;

                    double distance = sqrt(pow(i, 2) + pow(j, 2) + pow(k, 2));
                    if (distance <= radius)
                        pointsInSphere.push_back(curr);
                }

        for (auto &point : pointsInSphere)
        {
            bool isClearPath = true;
            Index3D d = point - centre;
            int steps = d.maxAbsInt();

            for (int i = 1; i < steps; ++i)
            {
                Index3D curr = centre + d * i / steps;

                if (binaryArray[curr.x][curr.y][curr.z])
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
    Index3D getRandomPointFromLOS(vector<Index3D> &pts, Index3D centre)
    {
        vector<pair<double, Index3D>> weightedPts;

        for (auto &point : pts)
        {
            Index3D d = point - centre;
            double distance = d.dist();
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
    DronePos getAdjacentPoint(Index3D point, Index3D centre)
    {
        vector<DronePos> adjacentPoints = {
            DronePos(point - Index3D(1, 0, 0), 1, atan2(point.y - centre.y, point.x - 1 - centre.x), acos((point.z - centre.z) / sqrt(pow(point.x - 1 - centre.x, 2) + pow(point.y - centre.y, 2) + pow(point.z - centre.z, 2)))),
            DronePos(point + Index3D(1, 0, 0), 2, atan2(point.y - centre.y, point.x + 1 - centre.x), acos((point.z - centre.z) / sqrt(pow(point.x + 1 - centre.x, 2) + pow(point.y - centre.y, 2) + pow(point.z - centre.z, 2)))),
            DronePos(point - Index3D(0, 1, 0), 3, atan2(point.y - 1 - centre.y, point.x - centre.x), acos((point.z - centre.z) / sqrt(pow(point.x - centre.x, 2) + pow(point.y - 1 - centre.y, 2) + pow(point.z - centre.z, 2)))),
            DronePos(point + Index3D(0, 1, 0), 4, atan2(point.y + 1 - centre.y, point.x - centre.x), acos((point.z - centre.z) / sqrt(pow(point.x - centre.x, 2) + pow(point.y + 1 - centre.y, 2) + pow(point.z - centre.z, 2)))),
            DronePos(point + Index3D(1, 1, 0), 5, atan2(point.y + 1 - centre.y, point.x + 1 - centre.x), acos((point.z - centre.z) / sqrt(pow(point.x + 1 - centre.x, 2) + pow(point.y + 1 - centre.y, 2) + pow(point.z - centre.z, 2)))),
            DronePos(point + Index3D(1, -1, 0), 6, atan2(point.y - 1 - centre.y, point.x + 1 - centre.x), acos((point.z - centre.z) / sqrt(pow(point.x + 1 - centre.x, 2) + pow(point.y - 1 - centre.y, 2) + pow(point.z - centre.z, 2)))),
            DronePos(point + Index3D(-1, 1, 0), 7, atan2(point.y + 1 - centre.y, point.x - 1 - centre.x), acos((point.z - centre.z) / sqrt(pow(point.x - 1 - centre.x, 2) + pow(point.y + 1 - centre.y, 2) + pow(point.z - centre.z, 2)))),
            DronePos(point + Index3D(-1, -1, 0), 8, atan2(point.y - 1 - centre.y, point.x - 1 - centre.x), acos((point.z - centre.z) / sqrt(pow(point.x - 1 - centre.x, 2) + pow(point.y - 1 - centre.y, 2) + pow(point.z - centre.z, 2))))};

        for (auto &point : adjacentPoints)
        {

            if (binaryArray[point.pos.x][point.pos.y][point.pos.z] == 0)
                return point;
        }

        return DronePos();
    }
};

// int main()
{
    OcTree octree("city_1.binvox.bt");

    vector<vector<vector<int>>> binaryArray;

    double voxel_size = octree.getResolution();
    // double voxel_size = 1.0f;

    octreeToBinaryArray(octree, binaryArray, voxel_size);
    reduceResolution(binaryArray, 2);
    markUnvisitedCells(binaryArray, 0, 0, 0);
    binaryArray = addBoundary(binaryArray);
    markFaces(binaryArray);
    saveToCSV(binaryArray, "first");

    int x_dim = binaryArray.size();
    int y_dim = binaryArray[0].size();
    int z_dim = binaryArray[0][0].size();
    cout << "Dimensions: " << x_dim << "x" << y_dim << "x" << z_dim << "\n";

    int threshold = 1000;
    int iter = 1;
    int radius = 43;
    while (1)
    {
        vector<vector<vector<bool>>> prevVisitArray(x_dim, vector<vector<bool>>(y_dim, vector<bool>(z_dim, false)));
        tuple<int, int, int> startPt = make_tuple(0, 0, 0);
        bool to_cont = false;
        for (int i = 0; i < 4; ++i)
        {
            binaryArray[get<0>(startPt)][get<1>(startPt)][get<2>(startPt)] = 4;
            auto pts = checkLOS(binaryArray, get<0>(startPt), get<1>(startPt), get<2>(startPt), radius);
            vector<tuple<int, int, int>> new_pts;
            for (auto &point : pts)
                if (!prevVisitArray[get<0>(point)][get<1>(point)][get<2>(point)])
                    new_pts.push_back(point);
            for (auto &point : new_pts)
                prevVisitArray[get<0>(point)][get<1>(point)][get<2>(point)] = true;

            if (!new_pts.size())
            {
                cout << "No Points";
                to_cont = true;
                break;
            }

            // cout << "Number of points in LOS: " << pts.size() << endl;
            startPt = getRandomPointFromLOS(new_pts, get<0>(startPt), get<1>(startPt), get<2>(startPt));
            // cout << "New starting point: (" << get<0>(startPt) << ", " << get<1>(startPt) << ", " << get<2>(startPt) << ")" << endl;
        }
        if (to_cont)
            continue;
        binaryArray[get<0>(startPt)][get<1>(startPt)][get<2>(startPt)] = 4;
        auto pts = checkLOS(binaryArray, get<0>(startPt), get<1>(startPt), get<2>(startPt), radius, true);
        int cnt = 0;
        for (auto &point : pts)
            if (binaryArray[get<0>(point)][get<1>(point)][get<2>(point)] == 2) // vertical face points only
                cnt++;
        cout << "Number of points in LOS that are faces: " << cnt << endl;
        if (cnt < threshold)
            continue;

        vector<tuple<int, int, int, int, float, float>> to_visit_pts;
        for (auto &point : pts)
            if (binaryArray[get<0>(point)][get<1>(point)][get<2>(point)] == 2)
            {
                binaryArray[get<0>(point)][get<1>(point)][get<2>(point)] = 3;
                to_visit_pts.push_back(getAdjacentPoint(binaryArray, get<0>(point), get<1>(point), get<2>(point), get<0>(startPt), get<1>(startPt), get<2>(startPt)));
            }

        vector<tuple<int, int, int, int, float, float>> new_to_visit_pts;
        for (auto &point : to_visit_pts)
            if (check2points(binaryArray, get<0>(startPt), get<1>(startPt), get<2>(startPt), get<0>(point), get<1>(point), get<2>(point)))
                new_to_visit_pts.push_back(point);

        cout << "Size of to_visit_pts: " << to_visit_pts.size() << endl;
        cout << "Size of new_to_visit_pts: " << new_to_visit_pts.size() << endl;

        sort(new_to_visit_pts.begin(), new_to_visit_pts.end(), [](const auto &a, const auto &b)
             { return make_pair(get<4>(a), get<5>(a)) < make_pair(get<4>(b), get<5>(b)); });

        vector<int> to_break;
        for (int i = 1; i < new_to_visit_pts.size(); ++i)
        {
            if (!check2points(binaryArray, get<0>(new_to_visit_pts[i - 1]), get<1>(new_to_visit_pts[i - 1]), get<2>(new_to_visit_pts[i - 1]), get<0>(new_to_visit_pts[i]), get<1>(new_to_visit_pts[i]), get<2>(new_to_visit_pts[i])))
                to_break.push_back(i);
        }
        cout << "Size of to_break: " << to_break.size() << endl;

        saveToCSV(binaryArray, "array_" + to_string(iter++));
    }

    return 0;
}
