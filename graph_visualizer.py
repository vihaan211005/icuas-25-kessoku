import open3d as o3d
import pandas as pd
import numpy as np
import csv

def load_csv_to_array(filename, x_dim, y_dim, z_dim):
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        data = list(reader)
        
    # Convert CSV rows into a 3D numpy array
    data_array = np.array([list(map(int, row)) for row in data], dtype=np.int32)
    data_array = data_array.reshape((z_dim, y_dim, x_dim)).transpose(2, 1, 0)  # Adjust the order of dimensions
    return data_array

def create_voxel_point_cloud(voxel_grid):
    # Get the coordinates of non-zero (occupied) voxels
    occupied_voxels = np.argwhere(voxel_grid > 0)  # Keep as integers for indexing
    occupied_voxels_float = occupied_voxels.astype(np.float64)
    
    # Create a point cloud using Open3D
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(occupied_voxels_float)

    # Extract colors based on voxel intensity
    colors = np.array([voxel_grid[tuple(point)] for point in occupied_voxels])  # Intensity values
    colors_rgb = np.zeros((len(colors), 3))  # Default color (black)

    # Assign grayscale to values in the range 0-3
    grayscale_mask = colors <= 3
    colors_normalized = (3 - colors[grayscale_mask]) / 3
    colors_rgb[grayscale_mask] = np.stack([colors_normalized] * 3, axis=1)

    # Assign specific colors for values 4-7
    color_map = {
        4: [0, 0, 1],  # Blue
        5: [1, 0, 0],  # Red
        6: [0, 1, 0],  # Green
        7: [1, 1, 0],  # Yellow
        8: [0, 1, 1]   # Cyan
    }
    
    for value, rgb in color_map.items():
        mask = colors == value
        colors_rgb[mask] = rgb

    pcd.colors = o3d.utility.Vector3dVector(colors_rgb)
    
    return pcd

def create_line_set(csv_file):
    df = pd.read_csv(csv_file, header=None)
    points = []
    lines = []
    colors = []
    
    for i, row in df.iterrows():
        p1 = [row[0], row[1], row[2]]  # x1, y1, z1
        p2 = [row[3], row[4], row[5]]  # x2, y2, z2
        points.append(p1)
        points.append(p2)
        lines.append([2 * i, 2 * i + 1])
        colors.append([1, 0, 0])  # Red color

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    return line_set

# Example usage
csv_file = "edges.csv"
x_dim, y_dim, z_dim = 129, 152, 53

binary_array = load_csv_to_array("first.csv", x_dim, y_dim, z_dim)
voxel_pcd = create_voxel_point_cloud(binary_array)
line_set = create_line_set(csv_file)

# Visualize both in the same window
o3d.visualization.draw_geometries([voxel_pcd, line_set], width=800, height=600)
