import numpy as np
import open3d as o3d
import csv
import matplotlib.pyplot as plt
from scipy.ndimage import zoom

# Load the CSV file into a 3D NumPy array
def load_csv_to_array(filename, x_dim, y_dim, z_dim):
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        data = list(reader)
        
    # Convert CSV rows into a 3D numpy array
    data_array = np.array([list(map(int, row)) for row in data], dtype=np.int32)
    data_array = data_array.reshape((z_dim, y_dim, x_dim)).transpose(2, 1, 0)  # Adjust the order of dimensions
    return data_array

# Visualize 2D
def visualize_2d(data):
    plt.imshow(data, cmap='gray')
    plt.show()

# Visualize using Open3D
def visualize_voxels(voxel_grid):
    # Get the coordinates of non-zero (occupied) voxels
    print(voxel_grid.shape)
    occupied_voxels = np.argwhere(voxel_grid > 0)  # Keep as integers for indexing

    # Convert to float for visualization
    occupied_voxels_float = occupied_voxels.astype(np.float64)
    
    # Create a point cloud using Open3D
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(occupied_voxels_float)

    # Extract colors based on voxel intensity
    colors = np.array([voxel_grid[tuple(point)] for point in occupied_voxels])  # Intensity values

    # Create an array to hold the colors
    colors_rgb = np.zeros((len(colors), 3))  # Initialize to black (default color)

    # Assign grayscale to values in the range 0-3
    grayscale_mask = colors <= 3
    colors_normalized = (3 - colors[grayscale_mask]) / 3  # Normalize grayscale
    colors_rgb[grayscale_mask] = np.stack([colors_normalized] * 3, axis=1)

    # Assign blue color to value 4
    blue_mask = colors == 4
    colors_rgb[blue_mask] = [0, 0, 1]  # RGB for blue

    # Set colors in the point cloud
    pcd.colors = o3d.utility.Vector3dVector(colors_rgb)
    
    # Visualize the points using Open3D
    o3d.visualization.draw_geometries([pcd], width=800, height=600)

def arr_to_pcd(binary_array):
    # Get the indices of the non-zero elements (point coordinates)
    points = np.argwhere(binary_array)

    # Convert indices to float32 (Open3D requires float values for points)
    points = points.astype(np.float32)

    # Create an Open3D PointCloud object
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)

    # Save the point cloud to a PCD file
    o3d.io.write_point_cloud("output.pcd", point_cloud)


import pickle

def save_array_to_pkl(array):
    with open("output.pkl", 'wb') as file:
        pickle.dump(array, file)

# Main Function
if __name__ == "__main__":
    # Define the dimensions of the 3D binary array
    x_dim = 211
    y_dim = 256
    z_dim = 83

    binary_array = load_csv_to_array(f"first.csv", x_dim, y_dim, z_dim)
    save_array_to_pkl(binary_array)
    visualize_voxels(binary_array)
    input()

    # for i in range(z_dim):
    #     visualize_2d(binary_array[:, :, i])
    
    for i in range(1,31):
    
        # Load the 3D binary array (e.g., from "3d_binary_array.csv")
        binary_array = load_csv_to_array(f"array_{i}.csv", x_dim, y_dim, z_dim)
        
        # Visualize using Open3D
        visualize_voxels(binary_array)

    # Visualize 2D
    # for i in range(z_dim):
    #     visualize_2d(binary_array[:, :, i])
