#include <octomap/octomap.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <queue>
#include <fstream>
#include <nlohmann/json.hpp>
#include <iostream>

#define LOG(x) std::cout << x << std::endl
using json = nlohmann::json;

// Structure to store circle info
struct Circle {
    double x, y;
    int index_x, index_y;
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

std::vector<Circle> findCirclesOnSlice(const std::vector<std::vector<uint8_t>>& binary_slice, double resolution,
                                       double min_x, double min_y) {
    std::vector<Circle> circles;
    size_t size_y = binary_slice.size();
    size_t size_x = binary_slice[0].size();
    std::vector<std::vector<bool>> visited(size_y, std::vector<bool>(size_x, false));

    for (size_t y = 0; y < size_y; ++y) {
        for (size_t x = 0; x < size_x; ++x) {
            if (binary_slice[y][x] == 1 && !visited[y][x]) {
                std::vector<std::pair<int, int>> component;
                bfs(x, y, binary_slice, visited, component);

                double mean_x = 0, mean_y = 0;
                for (const auto& [cx, cy] : component) {
                    mean_x += cx;
                    mean_y += cy;
                }
                mean_x /= component.size();
                mean_y /= component.size();

                double global_x = min_x + mean_x * resolution;
                double global_y = min_y + mean_y * resolution;
                circles.push_back({global_x, global_y, static_cast<int>(mean_x), static_cast<int>(mean_y)});
            }
        }
    }

    return circles;
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

    std::vector<std::vector<uint8_t>> binary_slice(size_y, std::vector<uint8_t>(size_x, 0));

    double slice_z = 1.0;  // Adjustable
    for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
        if (tree.isNodeOccupied(*it)) {
            double x = it.getX(), y = it.getY(), z = it.getZ();
            if (std::fabs(z - slice_z) < resolution) {
                size_t ix = static_cast<size_t>((x - min_x) / resolution);
                size_t iy = static_cast<size_t>((y - min_y) / resolution);
                if (ix < size_x && iy < size_y)
                    binary_slice[iy][ix] = 1;
            }
        }
    }

    std::vector<Circle> circles = findCirclesOnSlice(binary_slice, resolution, min_x, min_y);

    // Generate JSON
    json output;
    output["resolution"] = resolution;
    output["min_bound"] = {min_x, min_y, min_z};
    output["max_bound"] = {max_x, max_y, max_z};
    output["dimensions"] = {dim_x, dim_y, dim_z};
    output["array_shape"] = {size_y, size_x};  // (rows, cols) = (y, x)

    output["circles"] = json::array();
    for (const auto& c : circles) {
        output["circles"].push_back({
            {"real_world_center", {c.x, c.y}},
            {"grid_index", {c.index_x, c.index_y}}
        });
    }

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
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <octomap.bt> <output.json>\n";
        return 1;
    }

    std::string bt_file = argv[1];
    std::string json_file = "/output.json";
    if(argc > 2)
        std::string json_file = argv[2];

    octomap::OcTree tree(bt_file);

    double min_x, min_y, min_z, max_x, max_y, max_z;
    tree.getMetricMin(min_x, min_y, min_z);
    tree.getMetricMax(max_x, max_y, max_z);
    double resolution = tree.getResolution();

    double dim_x = max_x - min_x;
    double dim_y = max_y - min_y;
    double dim_z = max_z - min_z;

    size_t size_x = static_cast<size_t>(dim_x / resolution) + 1;
    size_t size_y = static_cast<size_t>(dim_y / resolution) + 1;

    std::vector<std::vector<uint8_t>> binary_slice(size_y, std::vector<uint8_t>(size_x, 0));

    double slice_z = 1.0;  // Adjustable
    for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
        if (tree.isNodeOccupied(*it)) {
            double x = it.getX(), y = it.getY(), z = it.getZ();
            if (std::fabs(z - slice_z) < resolution) {
                size_t ix = static_cast<size_t>((x - min_x) / resolution);
                size_t iy = static_cast<size_t>((y - min_y) / resolution);
                if (ix < size_x && iy < size_y)
                    binary_slice[iy][ix] = 1;
            }
        }
    }

    std::vector<Circle> circles = findCirclesOnSlice(binary_slice, resolution, min_x, min_y);

    // Generate JSON
    json output;
    output["resolution"] = resolution;
    output["min_bound"] = {min_x, min_y, min_z};
    output["max_bound"] = {max_x, max_y, max_z};
    output["dimensions"] = {dim_x, dim_y, dim_z};
    output["array_shape"] = {size_y, size_x};  // (rows, cols) = (y, x)

    output["circles"] = json::array();
    for (const auto& c : circles) {
        output["circles"].push_back({
            {"real_world_center", {c.x, c.y}},
            {"grid_index", {c.index_x, c.index_y}}
        });
    }

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
*/
