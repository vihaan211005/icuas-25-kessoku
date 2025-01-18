#include <octomap/octomap.h>
#include <vector>
#include <iostream>

using namespace std;
using namespace octomap;

// Function to convert an octree to a 3D binary array, considering leaf node size
void octreeToBinaryArray(const OcTree& octree, std::vector<std::vector<std::vector<int>>>& binaryArray, double voxel_size) {
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
    for (OcTree::leaf_iterator it = octree.begin_leafs(), end = octree.end_leafs(); it != end; ++it) {
        // Get the position of the leaf node's center
        float x = it.getX();
        float y = it.getY();
        float z = it.getZ();
        
        // Only consider occupied voxels
        if (it->getOccupancy() > 0.5) {
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
            for (int i = start_idx_x; i <= end_idx_x; ++i) {
                for (int j = start_idx_y; j <= end_idx_y; ++j) {
                    for (int k = start_idx_z; k <= end_idx_z; ++k) {
                        binaryArray[i][j][k] = 1;  // Mark as occupied
                    }
                }
            }
        }
    }
}

int main() {
    // Load an OctoMap from a file
    OcTree octree("city_1.binvox.bt");

    // Define voxel size (e.g., 0.1 meters)
    double voxel_size = octree.getResolution();
    // double voxel_size = 1.0f;
    
    // Create a 3D array to store binary data
    std::vector<std::vector<std::vector<int>>> binaryArray;
    
    // Convert the octree to a binary array
    octreeToBinaryArray(octree, binaryArray, voxel_size);
    
    // Output the 3D binary array (for demonstration)
    // Printing part of it for visualization
    int cnt = 0;
    for (int x = 0; x < binaryArray.size(); x++) {
        for (int y = 0; y < binaryArray[x].size(); y++) {
            for (int z = 0; z < binaryArray[x][y].size(); z++) {
                if (binaryArray[x][y][z] == 1) {
                    cnt++;
                    // cout << "Occupied at (" << x << ", " << y << ", " << z << ")\n";
                }
            }
        }
    }
    cout << "Total occupied voxels: " << cnt << endl;

    // Open a CSV file for output
    std::ofstream outfile("3d_binary_array.csv");

    // Write 3D array data to CSV
    for (int z = 0; z < binaryArray[0][0].size(); ++z) {
        for (int y = 0; y < binaryArray[0].size(); ++y) {
            for (int x = 0; x < binaryArray.size(); ++x) {
                outfile << binaryArray[x][y][z];
                if (x < binaryArray.size() - 1) outfile << ",";  // Avoid comma at the end of line
            }
            outfile << "\n";
        }
    }

    std::cout<<"x_dim: "<<binaryArray.size()<<"\n";
    std::cout<<"y_dim: "<<binaryArray[0].size()<<"\n";
    std::cout<<"z_dim: "<<binaryArray[0][0].size()<<"\n";

    std::cout << "CSV Exported!" << std::endl;

    return 0;
}
