#include <octomap/octomap.h>
#include <vector>
#include <iostream>
#include <queue>

using namespace std;
using namespace octomap;

void saveToCSV(std::vector<std::vector<std::vector<int>>> &binaryArray)
{
    // Open a CSV file for output
    std::ofstream outfile("3d_binary_array.csv");

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
    std::cout << "CSV Exported!" << std::endl;
}

// Function to convert an octree to a 3D binary array, considering leaf node size
void octreeToBinaryArray(const OcTree &octree, std::vector<std::vector<std::vector<int>>> &binaryArray, double voxel_size)
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
    binaryArray.resize(size_x, std::vector<std::vector<int>>(size_y, std::vector<int>(size_z, 0)));

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
            start_idx_x = std::max(0, start_idx_x);
            end_idx_x = std::min(size_x - 1, end_idx_x);
            start_idx_y = std::max(0, start_idx_y);
            end_idx_y = std::min(size_y - 1, end_idx_y);
            start_idx_z = std::max(0, start_idx_z);
            end_idx_z = std::min(size_z - 1, end_idx_z);

            // Mark all grid points inside the leaf's bounding box as occupied
            for (int i = start_idx_x; i <= end_idx_x; ++i)
                for (int j = start_idx_y; j <= end_idx_y; ++j)
                    for (int k = start_idx_z; k <= end_idx_z; ++k)
                        binaryArray[i][j][k] = 1; // Mark as occupied
        }
    }
}

void dfs(std::vector<std::vector<std::vector<int>>> &binaryArray, int x, int y, int z)
{
    std::queue<std::tuple<int, int, int>> queue;
    queue.push(std::make_tuple(x, y, z));

    while (!queue.empty())
    {
        auto [curr_x, curr_y, curr_z] = queue.front();
        queue.pop();

        if (curr_x < 0 || curr_x >= binaryArray.size() || curr_y < 0 || curr_y >= binaryArray[0].size() || curr_z < 0 || curr_z >= binaryArray[0][0].size() || binaryArray[curr_x][curr_y][curr_z] != 0)
            continue;

        binaryArray[curr_x][curr_y][curr_z] = 2;

        queue.push(std::make_tuple(curr_x - 1, curr_y, curr_z));
        queue.push(std::make_tuple(curr_x + 1, curr_y, curr_z));
        queue.push(std::make_tuple(curr_x, curr_y - 1, curr_z));
        queue.push(std::make_tuple(curr_x, curr_y + 1, curr_z));
        queue.push(std::make_tuple(curr_x, curr_y, curr_z - 1));
        queue.push(std::make_tuple(curr_x, curr_y, curr_z + 1));
    }
}

void markUnvisitedCells(std::vector<std::vector<std::vector<int>>> &binaryArray, int x, int y, int z)
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

void markFaces(std::vector<std::vector<std::vector<int>>> &binaryArray)
{
    for (int i = 0; i < binaryArray.size(); ++i)
        for (int j = 0; j < binaryArray[0].size(); ++j)
            for (int k = 0; k < binaryArray[0][0].size(); ++k)
                if (binaryArray[i][j][k] == 1)
                {
                    // Check if the current cell is on the boundary
                    if (i == 0 || i == binaryArray.size() - 1 || j == 0 || j == binaryArray[0].size() - 1 || k == 0 || k == binaryArray[0][0].size() - 1)
                    {
                        if (k == 0)
                            binaryArray[i][j][k] = 3; // Mark as horizontal face on z=0 boundary
                        else
                            binaryArray[i][j][k] = 2; // Mark as face
                    }

                    // Check neighboring cells
                    else
                    {
                        bool isFace = false;
                        int unvisitedNeighbors = 0;
                        for (int x = -1; x <= 1; ++x)
                            for (int y = -1; y <= 1; ++y)
                                for (int z = -1; z <= 1; ++z)
                                {
                                    int curr_x = i + x;
                                    int curr_y = j + y;
                                    int curr_z = k + z;

                                    if (curr_x < 0 || curr_x >= binaryArray.size() || curr_y < 0 || curr_y >= binaryArray[0].size() || curr_z < 0 || curr_z >= binaryArray[0][0].size())
                                        isFace = true;
                                    else if (binaryArray[curr_x][curr_y][curr_z] == 0)
                                    {
                                        if (z == 1)
                                            unvisitedNeighbors++;
                                        isFace = true;
                                    }
                                }

                        if (isFace)
                        {
                            if (unvisitedNeighbors >= 9 && k > 0)
                                binaryArray[i][j][k] = 3; // Mark as horizontal face
                            else
                                binaryArray[i][j][k] = 2; // Mark as face
                        }
                    }
                }
}

void reduceResolution(std::vector<std::vector<std::vector<int>>> &binaryArray, int factor)
{
    int newSizeX = binaryArray.size() / factor;
    int newSizeY = binaryArray[0].size() / factor;
    int newSizeZ = binaryArray[0][0].size() / factor;

    std::vector<std::vector<std::vector<int>>> reducedArray(newSizeX, std::vector<std::vector<int>>(newSizeY, std::vector<int>(newSizeZ, 0)));

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

std::vector<std::tuple<int, int, int>> checkLOS(std::vector<std::vector<std::vector<int>>> &binaryArray, int x, int y, int z, int radius)
{
    std::vector<std::tuple<int, int, int>> pointsInSphere;
    std::vector<std::tuple<int, int, int>> inLOS;

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
                    pointsInSphere.push_back(std::make_tuple(curr_x, curr_y, curr_z));
            }
    std::cout << "Number of points in sphere: " << pointsInSphere.size() << std::endl;

    for (auto &point : pointsInSphere)
    {
        bool isClearPath = true;
        int dx = std::get<0>(point) - x;
        int dy = std::get<1>(point) - y;
        int dz = std::get<2>(point) - z;
        int steps = std::max(std::abs(dx), std::max(std::abs(dy), std::abs(dz)));

        for (int i = 1; i < steps; ++i)
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

std::tuple<int, int, int> getRandomPointFromLOS(std::vector<std::tuple<int, int, int>> &pts, int x, int y, int z)
{
    std::vector<std::pair<double, std::tuple<int, int, int>>> weightedPts;

    for (auto &point : pts)
    {
        int dx = std::get<0>(point) - x;
        int dy = std::get<1>(point) - y;
        int dz = std::get<2>(point) - z;
        double distance = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
        weightedPts.push_back(std::make_pair(distance, point));
    }

    double sumWeights = 0.0;
    for (auto &weightedPt : weightedPts)
    {
        sumWeights += weightedPt.first;
    }

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
    // Load an OctoMap from a file
    OcTree octree("city_1.binvox.bt");

    // Create a 3D array to store binary data
    std::vector<std::vector<std::vector<int>>> binaryArray;

    // Define voxel size (e.g., 0.1 meters)
    double voxel_size = octree.getResolution();
    // double voxel_size = 1.0f;

    // Convert the octree to a binary array
    octreeToBinaryArray(octree, binaryArray, voxel_size);
    reduceResolution(binaryArray, 2);
    markUnvisitedCells(binaryArray, 0, 0, 0);
    markFaces(binaryArray);

    // auto pts = checkLOS(binaryArray, 0, 0, 0, (int)(43));
    // std::cout << "Number of points in LOS: " << pts.size() << std::endl;

    std::tuple<int, int, int> startPt = std::make_tuple(0, 0, 0);
    for (int i = 0; i < 5; ++i)
    {
        binaryArray[std::get<0>(startPt)][std::get<1>(startPt)][std::get<2>(startPt)] = 3;
        auto pts = checkLOS(binaryArray, std::get<0>(startPt), std::get<1>(startPt), std::get<2>(startPt), (int)(43));
        std::cout << "Number of points in LOS: " << pts.size() << std::endl;
        startPt = getRandomPointFromLOS(pts, std::get<0>(startPt), std::get<1>(startPt), std::get<2>(startPt));
        std::cout << "New starting point: (" << std::get<0>(startPt) << ", " << std::get<1>(startPt) << ", " << std::get<2>(startPt) << ")" << std::endl;
        binaryArray[std::get<0>(startPt)][std::get<1>(startPt)][std::get<2>(startPt)] = 3;
    }

    // Format and print dimensions in 2x3x4 format
    std::cout << "Dimensions: " << binaryArray.size() << "x" << binaryArray[0].size() << "x" << binaryArray[0][0].size() << "\n";

    saveToCSV(binaryArray);

    return 0;
}
