#include <octomap/octomap.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <queue>
#include <fstream>
#include <nlohmann/json.hpp>

#define LOG(x) std::cout << x << std::endl
using json = nlohmann::json;

// Structure to store circle info
struct Circle {
    double x, y;
    int index_x, index_y;
    double height;
};

const int directions[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

bool inBounds(int x, int y, int size_x, int size_y) {
    return x >= 0 && x < size_x && y >= 0 && y < size_y;
}

void bfs(int start_x, int start_y, const std::vector<std::vector<uint8_t>>& binary_slice,
         std::vector<std::vector<bool>>& visited, std::vector<std::pair<int, int>>& component) {
    std::queue<std::pair<int, int>> q;
    q.push({start_x, start_y});
    visited[start_y][start_x] = true;

    while (!q.empty()) {
        auto [x, y] = q.front();
        q.pop();
        component.push_back({x, y});

        for (const auto& dir : directions) {
            int nx = x + dir[0], ny = y + dir[1];
            if (inBounds(nx, ny, binary_slice[0].size(), binary_slice.size()) &&
                binary_slice[ny][nx] == 1 && !visited[ny][nx]) {
                visited[ny][nx] = true;
                q.push({nx, ny});
            }
        }
    }
}

double computeHeightAt(int gx, int gy, const std::vector<std::vector<std::vector<uint8_t>>>& binary_slice, int size_z, double min_z, double resolution) {
    for(int i = 1; i < size_z; ++i) {
        if(!binary_slice[i][gy][gx]){
            return (i - 1) * resolution + min_z;
        }
    }
    return (size_z - 1) * resolution + min_z;
}

std::vector<Circle> findCirclesOnSlice(const std::vector<std::vector<std::vector<uint8_t>>>& binary_slice,
                                       double resolution, double min_x, double min_y, double min_z) {
    std::vector<Circle> circles;
    size_t size_z = binary_slice.size();
    size_t size_y = binary_slice[0].size();
    size_t size_x = binary_slice[0][0].size();
    std::vector<std::vector<bool>> visited(size_y, std::vector<bool>(size_x, false));

    for (size_t y = 0; y < size_y; ++y) {
        for (size_t x = 0; x < size_x; ++x) {
            if (binary_slice[2][y][x] == 1 && !visited[y][x]) {
                std::vector<std::pair<int, int>> component;
                bfs(x, y, binary_slice[2], visited, component);

                double mean_x = 0, mean_y = 0;
                for (const auto& [cx, cy] : component) {
                    mean_x += cx;
                    mean_y += cy;
                }
                mean_x /= component.size();
                mean_y /= component.size();

                double global_x = min_x + mean_x * resolution;
                double global_y = min_y + mean_y * resolution;

                int gx = static_cast<int>(mean_x);
                int gy = static_cast<int>(mean_y);
                double height = computeHeightAt(gx, gy, binary_slice, size_z, min_z, resolution);

                circles.push_back({global_x, global_y, gx, gy, height});
            }
        }
    }

    return circles;
}

void fillBinaryVolume3D(std::vector<std::vector<std::vector<uint8_t>>>& volume) {
    int depth = volume.size();
    int height = volume[0].size();
    int width = volume[0][0].size();

    std::vector<std::vector<std::vector<bool>>> visited(depth, std::vector<std::vector<bool>>(height, std::vector<bool>(width, false)));

    std::queue<std::tuple<int, int, int>> q;

    for (int z = 0; z < depth; ++z) {
        for (int y = 0; y < height; ++y) {
            if (volume[z][y][0] == 0) q.emplace(z, y, 0), visited[z][y][0] = true;
            if (volume[z][y][width - 1] == 0) q.emplace(z, y, width - 1), visited[z][y][width - 1] = true;
        }
        for (int x = 0; x < width; ++x) {
            if (volume[z][0][x] == 0) q.emplace(z, 0, x), visited[z][0][x] = true;
            if (volume[z][height - 1][x] == 0) q.emplace(z, height - 1, x), visited[z][height - 1][x] = true;
        }
    }
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (volume[0][y][x] == 0) q.emplace(0, y, x), visited[0][y][x] = true;
            if (volume[depth - 1][y][x] == 0) q.emplace(depth - 1, y, x), visited[depth - 1][y][x] = true;
        }
    }

    const int dirs[6][3] = {
        {1, 0, 0}, {-1, 0, 0},
        {0, 1, 0}, {0, -1, 0},
        {0, 0, 1}, {0, 0, -1}
    };

    while (!q.empty()) {
        auto [z, y, x] = q.front(); q.pop();
        for (const auto& d : dirs) {
            int nz = z + d[0], ny = y + d[1], nx = x + d[2];
            if (nz >= 0 && nz < depth && ny >= 0 && ny < height && nx >= 0 && nx < width &&
                volume[nz][ny][nx] == 0 && !visited[nz][ny][nx]) {
                visited[nz][ny][nx] = true;
                q.emplace(nz, ny, nx);
            }
        }
    }

    for (int z = 0; z < depth; ++z)
        for (int y = 0; y < height; ++y)
            for (int x = 0; x < width; ++x)
                if (volume[z][y][x] == 0 && !visited[z][y][x])
                    volume[z][y][x] = 1;
}

int generateOctomapJSON(octomap::OcTree tree){
    double min_x, min_y, min_z, max_x, max_y, max_z;
    tree.getMetricMin(min_x, min_y, min_z);
    tree.getMetricMax(max_x, max_y, max_z);
    double resolution = tree.getResolution();
    
    double dim_x = max_x - min_x;
    double dim_y = max_y - min_y;
    double dim_z = max_z - min_z;
    
    size_t size_x = static_cast<size_t>(dim_x / resolution) + 1;
    size_t size_y = static_cast<size_t>(dim_y / resolution) + 1;
    size_t size_z = static_cast<size_t>(dim_z / resolution) + 1;
    
    std::vector<std::vector<std::vector<uint8_t>>> binary_slice(size_z, std::vector<std::vector<uint8_t>>(size_y, std::vector<uint8_t>(size_x, 0)));
    
    for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
        if (tree.isNodeOccupied(*it)) {
            double x = it.getX(), y = it.getY(), z = it.getZ();
            size_t ix = static_cast<size_t>((x - min_x) / resolution);
            size_t iy = static_cast<size_t>((y - min_y) / resolution);
            size_t iz = static_cast<size_t>((z - min_z) / resolution);
            if (ix < size_x && iy < size_y && iz < size_z)
                binary_slice[iz][iy][ix] = 1;
        }
    }
    
    fillBinaryVolume3D(binary_slice);
    
    std::vector<Circle> circles = findCirclesOnSlice(binary_slice, resolution, min_x, min_y, min_z);
    
    json output;
    output["resolution"] = resolution;
    output["min_bound"] = {min_x, min_y, min_z};
    output["max_bound"] = {max_x, max_y, max_z};
    output["dimensions"] = {dim_x, dim_y, dim_z};
    output["array_shape"] = {size_y, size_x};
    
    output["circles"] = json::array();
    for (const auto& c : circles) {
        output["circles"].push_back({
            {"real_world_center", {c.x, c.y}},
            {"grid_index", {c.index_x, c.index_y}},
            {"height", c.height}
        });
    }
    
    std::string json_file = "/output.json";
    std::ofstream ofs(json_file);
    if (!ofs) {
        std::cerr << "Failed to open output file: " << json_file << "\n";
        return 1;
    }
    ofs << output.dump(4);
    ofs.close();

    std::cout << "Output written to " << json_file << "\n";
    
    return 0;
}


/*
int main(int argc, char** argv) {

    std::string bt_file = argv[1];
    
    if (argc > 2)
        json_file = argv[2];

    octomap::OcTree tree(bt_file); 
}
*/