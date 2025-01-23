#include <octomap/octomap.h>
#include <vector>
#include <iostream>
#include <queue>

using namespace std;
using namespace octomap;

void saveToCSV(vector<vector<vector<int>>> &binaryArray, string file_name)
{
    // Open a CSV file for output
    ofstream outfile(file_name + ".csv");

    // Write 3D array data to CSV
    for (int z = 0; z < binaryArray[0][0].size(); ++z)
        for (int y = 0; y < binaryArray[0].size(); ++y)
        {
            for (int x = 0; x < binaryArray.size(); ++x)
            {
                outfile << binaryArray[x][y][z];
                if (x < binaryArray.size() - 1)
                    outfile << ","; // Avoid comma at the end of line
            }
            outfile << "\n";
        }
    cout << "CSV Exported!" << endl;
}

void octreeToBinaryArray(const OcTree &octree, vector<vector<vector<int>>> &binaryArray, double voxel_size)
{
    // Get the bounding box of the OctoMap
    double min_x, max_x, min_y, max_y, min_z, max_z;
    octree.getMetricMin(min_x, min_y, min_z);
    octree.getMetricMax(max_x, max_y, max_z);

    // Calculate the size of the 3D grid (based on the voxel size)
    int size_x = static_cast<int>((max_x - min_x) / voxel_size);
    int size_y = static_cast<int>((max_y - min_y) / voxel_size);
    int size_z = static_cast<int>((max_z - min_z) / voxel_size);

    // Resize the binary array to the correct size
    binaryArray.resize(size_x, vector<vector<int>>(size_y, vector<int>(size_z, 0)));

    // Iterate through the octree and fill the binary array
    for (OcTree::leaf_iterator it = octree.begin_leafs(), end = octree.end_leafs(); it != end; ++it)
    {
        // Get the position of the leaf node's center
        float x = it.getX();
        float y = it.getY();
        float z = it.getZ();

        // Only consider occupied voxels
        if (it->getOccupancy() > 0.5)
        {
            // Calculate the extents of the occupied region represented by the leaf
            float half_size = it.getSize() / 2.0; // Each leaf node has a size (side length)

            // Calculate the bounding box of the leaf node
            float leaf_min_x = x - half_size;
            float leaf_max_x = x + half_size;
            float leaf_min_y = y - half_size;
            float leaf_max_y = y + half_size;
            float leaf_min_z = z - half_size;
            float leaf_max_z = z + half_size;

            // Convert the leaf bounding box into indices in the binary array
            int start_idx_x = static_cast<int>((leaf_min_x - min_x) / voxel_size);
            int end_idx_x = static_cast<int>((leaf_max_x - min_x) / voxel_size);
            int start_idx_y = static_cast<int>((leaf_min_y - min_y) / voxel_size);
            int end_idx_y = static_cast<int>((leaf_max_y - min_y) / voxel_size);
            int start_idx_z = static_cast<int>((leaf_min_z - min_z) / voxel_size);
            int end_idx_z = static_cast<int>((leaf_max_z - min_z) / voxel_size);

            // Ensure the indices are within bounds
            start_idx_x = max(0, start_idx_x);
            end_idx_x = min(size_x - 1, end_idx_x);
            start_idx_y = max(0, start_idx_y);
            end_idx_y = min(size_y - 1, end_idx_y);
            start_idx_z = max(0, start_idx_z);
            end_idx_z = min(size_z - 1, end_idx_z);

            // Mark all grid points inside the leaf's bounding box as occupied
            for (int i = start_idx_x; i <= end_idx_x; ++i)
                for (int j = start_idx_y; j <= end_idx_y; ++j)
                    for (int k = start_idx_z; k <= end_idx_z; ++k)
                        binaryArray[i][j][k] = 1; // Mark as occupied
        }
    }
}

void dfs(vector<vector<vector<int>>> &binaryArray, int x, int y, int z)
{
    queue<tuple<int, int, int>> queue;
    queue.push(make_tuple(x, y, z));

    while (!queue.empty())
    {
        auto [curr_x, curr_y, curr_z] = queue.front();
        queue.pop();

        if (curr_x < 0 || curr_x >= binaryArray.size() || curr_y < 0 || curr_y >= binaryArray[0].size() || curr_z < 0 || curr_z >= binaryArray[0][0].size() || binaryArray[curr_x][curr_y][curr_z] != 0)
            continue;

        binaryArray[curr_x][curr_y][curr_z] = 2;

        queue.push(make_tuple(curr_x - 1, curr_y, curr_z));
        queue.push(make_tuple(curr_x + 1, curr_y, curr_z));
        queue.push(make_tuple(curr_x, curr_y - 1, curr_z));
        queue.push(make_tuple(curr_x, curr_y + 1, curr_z));
        queue.push(make_tuple(curr_x, curr_y, curr_z - 1));
        queue.push(make_tuple(curr_x, curr_y, curr_z + 1));
    }
}

void markUnvisitedCells(vector<vector<vector<int>>> &binaryArray, int x, int y, int z)
{
    dfs(binaryArray, x, y, z);

    for (int i = 0; i < binaryArray.size(); ++i)
        for (int j = 0; j < binaryArray[0].size(); ++j)
            for (int k = 0; k < binaryArray[0][0].size(); ++k)
            {
                if (binaryArray[i][j][k] == 0)
                    binaryArray[i][j][k] = 1;
                if (binaryArray[i][j][k] == 2)
                    binaryArray[i][j][k] = 0;
            }
}

void markFaces(vector<vector<vector<int>>> &binaryArray)
{
    int cnt = 0;
    for (int i = 1; i < binaryArray.size() - 1; ++i)
        for (int j = 1; j < binaryArray[0].size() - 1; ++j)
            for (int k = 1; k < binaryArray[0][0].size() - 1; ++k)
                if (binaryArray[i][j][k] == 1)
                {
                    if (k == 1)
                        binaryArray[i][j][k] = 3; // Mark as horizontal face on z=0 boundary
                    else
                    {
                        int plane[4] = {0, 0, 0, 0};
                        int total = 0;
                        for (int x = -1; x <= 1; ++x)
                            for (int y = -1; y <= 1; ++y)
                                for (int z = -1; z <= 1; ++z)
                                {
                                    if (binaryArray[i + x][j + y][k + z] != 0)
                                        total++;
                                    if (!z && binaryArray[i + x][j + y][k + z] == 0)
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
                        if (plane[0] >= 1 || plane[1] >= 1 || plane[2] >= 1 || plane[3] >= 1)
                        {
                            cnt++;
                            binaryArray[i][j][k] = 2; // Mark as vertical face
                        }
                        else
                        {
                            binaryArray[i][j][k] = 3; // Mark as horizontal face
                        }
                    }
                }
    cout<<cnt<<endl;
}

tuple<int, int, int, int, float, float> getAdjacentPoint(vector<vector<vector<int>>> &binaryArray, int x, int y, int z, int c_x, int c_y, int c_z)
{
    vector<tuple<int, int, int, int, float, float>> adjacentPoints = {
        make_tuple(x - 1, y, z, 1, atan2(y - c_y, x - 1 - c_x), acos(z - c_z / sqrt(pow(x - 1 - c_x, 2) + pow(y - c_y, 2) + pow(z - c_z, 2)))),
        make_tuple(x + 1, y, z, 2, atan2(y - c_y, x + 1 - c_x), acos(z - c_z / sqrt(pow(x + 1 - c_x, 2) + pow(y - c_y, 2) + pow(z - c_z, 2)))),
        make_tuple(x, y - 1, z, 3, atan2(y - 1 - c_y, x - c_x), acos(z - c_z / sqrt(pow(x - c_x, 2) + pow(y - 1 - c_y, 2) + pow(z - c_z, 2)))),
        make_tuple(x, y + 1, z, 4, atan2(y + 1 - c_y, x - c_x), acos(z - c_z / sqrt(pow(x - c_x, 2) + pow(y + 1 - c_y, 2) + pow(z - c_z, 2)))),
        make_tuple(x + 1, y + 1, z, 5, atan2(y + 1 - c_y, x + 1 - c_x), acos(z - c_z / sqrt(pow(x + 1 - c_x, 2) + pow(y + 1 - c_y, 2) + pow(z - c_z, 2)))),
        make_tuple(x + 1, y - 1, z, 6, atan2(y - 1 - c_y, x + 1 - c_x), acos(z - c_z / sqrt(pow(x + 1 - c_x, 2) + pow(y - 1 - c_y, 2) + pow(z - c_z, 2)))),
        make_tuple(x - 1, y + 1, z, 7, atan2(y + 1 - c_y, x - 1 - c_x), acos(z - c_z / sqrt(pow(x - 1 - c_x, 2) + pow(y + 1 - c_y, 2) + pow(z - c_z, 2)))),
        make_tuple(x - 1, y - 1, z, 8, atan2(y - 1 - c_y, x - 1 - c_x), acos(z - c_z / sqrt(pow(x - 1 - c_x, 2) + pow(y - 1 - c_y, 2) + pow(z - c_z, 2)))),
    };

    for (auto &point : adjacentPoints)
    {
        int curr_x = get<0>(point);
        int curr_y = get<1>(point);
        int curr_z = get<2>(point);

        if (binaryArray[curr_x][curr_y][curr_z] == 0)
            return point;
    }

    // Return the original point if no adjacent point is 0
    return make_tuple(0, 0, 0, 0.0, 0, 0);
}

vector<vector<vector<int>>> addBoundary(vector<vector<vector<int>>> &array)
{
    int x_dim = array.size();
    int y_dim = array[0].size();
    int z_dim = array[0][0].size();

    vector<vector<vector<int>>> newArray(x_dim + 2, vector<vector<int>>(y_dim + 2, vector<int>(z_dim + 2, 0)));

    for (int i = 0; i < x_dim; ++i)
        for (int j = 0; j < y_dim; ++j)
            for (int k = 0; k < z_dim; ++k)
                newArray[i + 1][j + 1][k + 1] = array[i][j][k];

    return newArray;
}

void reduceResolution(vector<vector<vector<int>>> &binaryArray, int factor)
{
    int newSizeX = binaryArray.size() / factor;
    int newSizeY = binaryArray[0].size() / factor;
    int newSizeZ = binaryArray[0][0].size() / factor;

    vector<vector<vector<int>>> reducedArray(newSizeX, vector<vector<int>>(newSizeY, vector<int>(newSizeZ, 0)));

    for (int i = 0; i < newSizeX; ++i)
        for (int j = 0; j < newSizeY; ++j)
            for (int k = 0; k < newSizeZ; ++k)
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
                            if (binaryArray[x][y][z] == 1)
                                isOccupied = true;

                if (isOccupied)
                    reducedArray[i][j][k] = 1;
            }

    binaryArray = reducedArray;
}

bool check2points(vector<vector<vector<int>>> &binaryArray, int x1, int y1, int z1, int x2, int y2, int z2)
{
    int dx = x2 - x1;
    int dy = y2 - y1;
    int dz = z2 - z1;
    int steps = max(abs(dx), max(abs(dy), abs(dz)));

    for (int i = 1; i <= steps; ++i)
    {
        int curr_x = x1 + dx * i / steps;
        int curr_y = y1 + dy * i / steps;
        int curr_z = z1 + dz * i / steps;

        if (binaryArray[curr_x][curr_y][curr_z] != 0)
            return false;
    }

    return true;
}

vector<tuple<int, int, int>> checkLOS(vector<vector<vector<int>>> &binaryArray, int x, int y, int z, int radius, bool take_faces = false)
{
    vector<tuple<int, int, int>> pointsInSphere;
    vector<tuple<int, int, int>> inLOS;

    for (int i = -radius; i <= radius; ++i)
        for (int j = -radius; j <= radius; ++j)
            for (int k = -radius; k <= radius; ++k)
            {
                int curr_x = x + i;
                int curr_y = y + j;
                int curr_z = z + k;

                if (curr_x < 0 || curr_x >= binaryArray.size() || curr_y < 0 || curr_y >= binaryArray[0].size() || curr_z < 0 || curr_z >= binaryArray[0][0].size())
                    continue;

                double distance = sqrt(pow(i, 2) + pow(j, 2) + pow(k, 2));
                if (distance <= radius)
                    pointsInSphere.push_back(make_tuple(curr_x, curr_y, curr_z));
            }

    for (auto &point : pointsInSphere)
    {
        bool isClearPath = true;
        int dx = get<0>(point) - x;
        int dy = get<1>(point) - y;
        int dz = get<2>(point) - z;
        int steps = max(abs(dx), max(abs(dy), abs(dz)));

        int end = steps;
        if (take_faces)
            end--;
        for (int i = 1; i <= end; ++i)
        {
            int curr_x = x + dx * i / steps;
            int curr_y = y + dy * i / steps;
            int curr_z = z + dz * i / steps;

            if (binaryArray[curr_x][curr_y][curr_z] != 0)
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

tuple<int, int, int> getRandomPointFromLOS(vector<tuple<int, int, int>> &pts, int x, int y, int z)
{
    vector<pair<double, tuple<int, int, int>>> weightedPts;

    for (auto &point : pts)
    {
        int dx = get<0>(point) - x;
        int dy = get<1>(point) - y;
        int dz = get<2>(point) - z;
        double distance = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
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

int main()
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
        for (auto &point : new_to_visit_pts)
            if (check2points(binaryArray, get<0>(startPt), get<1>(startPt), get<2>(startPt), get<0>(point), get<1>(point), get<2>(point)))
                new_to_visit_pts.push_back(point);

        sort(new_to_visit_pts.begin(), new_to_visit_pts.end(), [](const auto &a, const auto &b)
             { return make_pair(get<4>(a), get<5>(a)) < make_pair(get<4>(b), get<5>(b)); });
        
        vector<int> to_break;
        for (int i = 1; i < new_to_visit_pts.size(); ++i){
            if (!check2points(binaryArray, get<0>(new_to_visit_pts[i-1]), get<1>(new_to_visit_pts[i-1]), get<2>(new_to_visit_pts[i-1]), get<0>(new_to_visit_pts[i]), get<1>(new_to_visit_pts[i]), get<2>(new_to_visit_pts[i])))
                to_break.push_back(i);
        }
        cout << "Size of to_break: " << to_break.size() << endl;

        saveToCSV(binaryArray, "array_" + to_string(iter++));
    }

    return 0;
}
