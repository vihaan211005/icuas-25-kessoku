#include <traversal.hpp>

std::ostream &operator<<(std::ostream &os, const Bounds &b)
{
    os << "Bounds(" << b.min << ", " << b.max << ")";
    return os;
}

Solver::Solver(Eigen::Vector3d base, octomap::OcTree octree, int radius, int num_drones) : baseStation(base), octree(octree), radius(radius), num_drones(num_drones) {}

void Solver::initialSetup()
{
    Solver::check();
    Solver::octreeToBinaryArray();
    std::cout << "mapBounds: " << mapBounds << std::endl;
    std::cout << "Resolution: " << resolution << std::endl;
    int factor = std::max(1, static_cast<int>(std::round(1.0 / resolution)));
    if (factor - 1)
        Solver::reduceResolution(factor);
    Solver::markInterior();
    for (int i = 0; i < 2; ++i)
        Solver::addBoundary();
    if (mapBounds.min.x() > baseStation.x())
        Solver::addBoundary((mapBounds.min.x() - baseStation.x()) / resolution + 1, 0, 0, 0, 0);
    if (mapBounds.min.y() > baseStation.y())
        Solver::addBoundary(0, (mapBounds.min.y() - baseStation.y()) / resolution + 1, 0, 0, 0);
    baseIndex.x() = std::round((baseStation.x() - mapBounds.min.x()) / resolution);
    baseIndex.y() = std::round((baseStation.y() - mapBounds.min.y()) / resolution);
    baseIndex.z() = std::round((baseStation.z() - mapBounds.min.z()) / resolution);
    while (binaryArray[baseIndex.x()][baseIndex.y()][baseIndex.z()])
        baseIndex.z()++;
    std::cout << "Base Index: " << baseIndex << std::endl;
    std::cout << "Base Station: " << baseStation << std::endl;
    std::cout << "Base Point: " << indexToPoint(baseIndex) << std::endl;
    Solver::nodes_graph.push_back(baseIndex);
    Solver::node_dirs.push_back(std::make_tuple(0, 0, baseIndex));
    Solver::markFaces();
    Solver::markCorner();
    Solver::makeAdjacency();

    std::cout << "mapBounds: " << mapBounds << std::endl;
    std::cout << "dimArray: " << dimArray << std::endl;
    visited.resize(dimArray.x(), std::vector<std::vector<bool>>(dimArray.y(), std::vector<bool>(dimArray.z(), false)));

    Solver::runBFS();

    Solver::solution.n = Solver::nodes_graph.size();
    Solver::solution.adjacency_matrix = adjacency_matrix;
    Solver::solution.parent = parent;
    Solver::solution.distance = distance;
    for (auto node : Solver::nodes_graph)
        Solver::solution.nodes_graph.push_back(indexToPoint(node));

    saveEdgesToCSV("edges", adjacency_matrix);
    saveToCSV("first");
}

std::pair<std::vector<std::vector<DronePos>>, std::vector<std::vector<DronePos>>> Solver::getAdjacentFace(int node)
{
    int dirx = std::get<0>(Solver::node_dirs[node]);
    int diry = std::get<1>(Solver::node_dirs[node]);

    std::pair<std::vector<std::vector<DronePos>>, std::vector<std::vector<DronePos>>> solution_;

    if (!dirx && !diry)
        return solution_;

    int x = std::get<2>(Solver::node_dirs[node]).x();
    int y = std::get<2>(Solver::node_dirs[node]).y();
    int z = std::get<2>(Solver::node_dirs[node]).z();

    int i = 0;
    int j = 0;

    while (1)
    {
        i = 0;
        std::vector<DronePos> poses;
        while (1)
        {
            if (check2points_octree(Eigen::Vector3i(x + 2 * dirx, y + i, z + j), Solver::nodes_graph[node], radius) && Solver::binaryArray[x][y + i][z + j] == 2 && (!visited[x][y + i][z + j] || !i))
            {
                poses.push_back(DronePos(Eigen::Vector3i(x + 2 * dirx, y + i, z + j), (dirx + 1) >> 1));
                visited[x][y + i][z + j] = true;
            }
            else
                break;
            diry == 1 ? i-- : i++;
        }
        if (poses.size() > 1)
            solution_.first.push_back(poses);
        else
            break;
        j -= 2;
        if ((!i) || !(z + j))
            break;
        if (mapBounds.min.z() + (z + j) * resolution < 1)
            break;
    }

    j = 0;
    while (1)
    {
        i = 0;
        std::vector<DronePos> poses;
        while (1)
        {
            if (check2points_octree(Eigen::Vector3i(x + i, y + 2 * diry, z + j), Solver::nodes_graph[node], radius) && Solver::binaryArray[x + i][y][z + j] == 2 && (!visited[x + i][y][z + j] || !i))
            {
                poses.push_back(DronePos(Eigen::Vector3i(x + i, y + 2 * diry, z + j), (diry + 5) >> 1));
                visited[x + i][y][z + j] = true;
            }
            else
                break;
            dirx == 1 ? i-- : i++;
        }
        if (poses.size() > 1)
            solution_.second.push_back(poses);
        else
            break;
        j -= 2;
        if ((!i) || !(z + j))
            break;
        if (mapBounds.min.z() + (z + j) * resolution < 1)
            break;
    }

    return solution_;
}

void Solver::runBFS()
{
    int n = Solver::nodes_graph.size();
    std::queue<int> queue;
    std::vector<bool> visited_(n, false);
    distance.resize(n, INT_MAX);
    parent.resize(n, INT_MAX);

    queue.push(0);
    visited_[0] = true;
    distance[0] = 0;
    parent[0] = 0;

    std::vector<std::vector<bool>> adjacency_matrix_new(n, std::vector<bool>(n, 0));

    int count = 0;

    while (!queue.empty())
    {
        int node = queue.front();
        queue.pop();

        // solution.bfs_order.push_back(std::make_pair(node, std::make_pair(std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>)));
        std::pair<std::vector<std::vector<DronePos>>, std::vector<std::vector<DronePos>>> faces = getAdjacentFace(node);
        std::vector<Eigen::Vector4d> pehla;
        std::vector<Eigen::Vector4d> dusra;
        if (faces.first.size() || faces.second.size())
            Solver::binaryArray[Solver::nodes_graph[node].x()][Solver::nodes_graph[node].y()][Solver::nodes_graph[node].z()] = 4;

        for (auto face : faces.first)
        {
            for (auto l : face)
            {
                Solver::binaryArray[l.pos.x()][l.pos.y()][l.pos.z()] = 5;
            }
            Eigen::Vector3d p1 = indexToPoint(face[0].pos);
            double yaw1 = (face[0].yaw * M_PI) + (M_PI / 2) * (face[0].yaw > 1);
            Eigen::Vector3d p2 = indexToPoint(face[face.size() - 1].pos);
            double yaw2 = (face[face.size() - 1].yaw * M_PI) + (M_PI / 2) * (face[face.size() - 1].yaw > 1);
            // std::cout << "yaw1: " << yaw1 << ", yaw2: " << yaw2 << std::endl;
            pehla.push_back(Eigen::Vector4d(p1.x(), p1.y(), p1.z(), yaw1));
            pehla.push_back(Eigen::Vector4d(p2.x(), p2.y(), p2.z(), yaw2));
        }
        for (auto face : faces.second)
        {
            for (auto l : face)
            {
                Solver::binaryArray[l.pos.x()][l.pos.y()][l.pos.z()] = 5;
            }
            Eigen::Vector3d p1 = indexToPoint(face[0].pos);
            double yaw1 = (face[0].yaw * M_PI) + (M_PI / 2) * (face[0].yaw > 1);
            Eigen::Vector3d p2 = indexToPoint(face[face.size() - 1].pos);
            double yaw2 = (face[face.size() - 1].yaw * M_PI) + (M_PI / 2) * (face[face.size() - 1].yaw > 1);
            // std::cout << "yaw1: " << yaw1 << ", yaw2: " << yaw2 << std::endl;
            dusra.push_back(Eigen::Vector4d(p1.x(), p1.y(), p1.z(), yaw1));
            dusra.push_back(Eigen::Vector4d(p2.x(), p2.y(), p2.z(), yaw2));
        }
        // if(pehla.size() || dusra.size())
        solution.bfs_order.push_back(std::make_pair(node, std::make_pair(pehla, dusra)));

        if (distance[node] == num_drones - 1)
            continue;

        for (int i = 0; i < n; ++i)
        {
            if (adjacency_matrix[node][i] && !visited_[i])
            {
                queue.push(i);
                visited_[i] = true;
                distance[i] = distance[node] + 1;
                parent[i] = node;
                adjacency_matrix_new[i][node] = 1;
                adjacency_matrix_new[node][i] = 1;
                count++;
            }
        }
    }
    adjacency_matrix = adjacency_matrix_new;

    std::cout << "Number of edges: " << count << std::endl;
    std::cout << "Number of edges: " << solution.bfs_order.size() << std::endl;
}

void Solver::makeAdjacency()
{
    int n = Solver::nodes_graph.size();
    int count = 0;
    adjacency_matrix.resize(n, std::vector<bool>(n, false));
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            if (i != j && check2points_octree(Solver::nodes_graph[i], Solver::nodes_graph[j], radius))
            {
                Eigen::Vector3i direction = (Solver::nodes_graph[j] - Solver::nodes_graph[i]).normalized();

                Eigen::Vector3i arbitrary(1, 0, 0);
                if (fabs(direction.dot(arbitrary)) > 0.99)
                    arbitrary = Eigen::Vector3i(0, 1, 0);

                Eigen::Vector3i perp1 = direction.cross(arbitrary).normalized();
                Eigen::Vector3i perp2 = direction.cross(perp1).normalized();

                Eigen::Vector3i center = Solver::nodes_graph[j];
                double circle_radius = 1.0;
                int num_samples = 10;

                bool valid_edge = true;
                for (int k = 0; k < num_samples; k++)
                {
                    double angle = (2 * M_PI * k) / num_samples;
                    Eigen::Vector3i sample_point = center + ((perp1.cast<double>() * cos(angle) + perp2.cast<double>() * sin(angle)) * circle_radius).cast<int>();

                    if (!check2points_octree(Solver::nodes_graph[i], sample_point, radius))
                    {
                        valid_edge = false;
                        break;
                    }
                }

                if (valid_edge)
                {
                    adjacency_matrix[i][j] = 1;
                    count++;
                }
            }
    std::cout << "Number of edges: " << count << std::endl;
}

void Solver::saveEdgesToCSV(std::string file_name, std::vector<std::vector<bool>> adjacency_matrix)
{
    std::ofstream outfile(file_name + ".csv");

    int n = Solver::nodes_graph.size();
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            if (adjacency_matrix[i][j])
            {
                outfile << Solver::nodes_graph[i].x() << ", " << Solver::nodes_graph[i].y() << ", " << Solver::nodes_graph[i].z() << ", " << Solver::nodes_graph[j].x() << ", " << Solver::nodes_graph[j].y() << ", " << Solver::nodes_graph[j].z() << "\n";
            }
        }
    }

    std::cout << "CSV Exported!" << std::endl;
}

void Solver::markCorner()
{
    for (int i = 0; i < dimArray.x(); ++i)
        for (int j = 0; j < dimArray.y(); ++j)
            for (int k = 0; k < dimArray.z(); ++k)
                if (Solver::binaryArray[i][j][k] == 2)
                {
                    bool adjacent[6] = {0, 0, 0, 0, 0, 0};
                    if (i > 0)
                        adjacent[0] = Solver::binaryArray[i - 1][j][k];
                    if (i < dimArray.x() - 1)
                        adjacent[1] = Solver::binaryArray[i + 1][j][k];
                    if (j > 0)
                        adjacent[2] = Solver::binaryArray[i][j - 1][k];
                    if (j < dimArray.y() - 1)
                        adjacent[3] = Solver::binaryArray[i][j + 1][k];
                    if (k > 0)
                        adjacent[4] = Solver::binaryArray[i][j][k - 1];
                    if (k < dimArray.z() - 1)
                        adjacent[5] = Solver::binaryArray[i][j][k + 1];

                    int dirx = 0, diry = 0;
                    adjacent[1] ? dirx = -1 : dirx = 1;
                    adjacent[3] ? diry = -1 : diry = 1;

                    int total = 0;
                    for (int x = 0; x < 6; x++)
                        if (adjacent[x] == 1)
                            total++;

                    if (total == 3 && !adjacent[5] && adjacent[4] && !Solver::binaryArray[i + dirx][j + diry][k + 1])
                    {
                        Solver::nodes_graph.push_back(Eigen::Vector3i(i + dirx, j + diry, k + 1));
                        Solver::node_dirs.push_back(std::make_tuple(dirx, diry, Eigen::Vector3i(i, j, k)));
                    }
                }
}

// Converts indextopoint
Eigen::Vector3d Solver::indexToPoint(Eigen::Vector3i index)
{
    return index.cast<double>() * resolution + mapBounds.min;
}

// Saves 3DArray to csv file
void Solver::saveToCSV(std::string file_name)
{
    std::ofstream outfile(file_name + ".csv");

    for (int z = 0; z < dimArray.z(); ++z)
        for (int y = 0; y < dimArray.y(); ++y)
        {
            for (int x = 0; x < dimArray.x(); ++x)
            {
                outfile << Solver::binaryArray[x][y][z];
                if (x < dimArray.x() - 1)
                    outfile << ",";
            }
            outfile << "\n";
        }
    std::cout << "CSV Exported!" << std::endl;
}

// Converts OctTree to 3DArray
void Solver::octreeToBinaryArray()
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
                    {
                        binaryArray[i][j][k] = 1;
                    }
        }
    }
}

// Fills the interior of buildings as occupied
void Solver::markInterior()
{
    int x = 0, y = 0, z = dimArray.z() - 1;

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
void Solver::reduceResolution(int factor)
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
void Solver::addBoundary(int x_front, int y_front, int x_back, int y_back, int z_back)
{
    mapBounds.max += Eigen::Vector3d(x_back * resolution, y_back * resolution, z_back * resolution);
    mapBounds.min -= Eigen::Vector3d(x_front * resolution, y_front * resolution, 0);

    dimArray.x() += x_front + x_back;
    dimArray.y() += y_front + y_back;
    dimArray.z() += z_back;

    std::vector<std::vector<std::vector<int>>> newArray(dimArray.x(), std::vector<std::vector<int>>(dimArray.y(), std::vector<int>(dimArray.z(), 0)));

    for (int i = x_front; i < dimArray.x() - x_back; ++i)
        for (int j = y_front; j < dimArray.y() - y_back; ++j)
            for (int k = 0; k < dimArray.z() - z_back; ++k)
                newArray[i][j][k] = binaryArray[i - x_front][j - y_front][k];

    Solver::binaryArray = newArray;
}

// Mark Horizontal faces as 3 and Verical as 2. Call after adding x,y buffer and above z
void Solver::markFaces()
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
bool Solver::check2points_octree(Eigen::Vector3i p1, Eigen::Vector3i p2, double radius)
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
    bool hits = Solver::octree.castRay(start, end - start, hit, true, (end - start).norm());
    return !hits;
}

void Solver::check()
{
#ifdef __FAST_MATH__
    std::cout << "FAST_MATH is enabled\n";
#else
    std::cout << "FAST_MATH is NOT enabled\n";
#endif

#ifdef __OPTIMIZE__
    std::cout << "Optimization is enabled\n";
#else
    std::cout << "Optimization is NOT enabled\n";
#endif
}

int main()
{
    Solver solver = Solver(Eigen::Vector3d(0, 0, 1), octomap::OcTree("city_1.binvox.bt"), 70, 5);
    solver.initialSetup();
    return 0;
}
