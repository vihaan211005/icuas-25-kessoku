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
    colors_normalized = (3-colors)/3  # Normalize to [0, 1]
    pcd.colors = o3d.utility.Vector3dVector(np.stack([colors_normalized] * 3, axis=1))
    
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

# Main Function
if __name__ == "__main__":
    # Define the dimensions of the 3D binary array
    x_dim = 105
    y_dim = 128
    z_dim = 41
    
    # Load the 3D binary array (e.g., from "3d_binary_array.csv")
    binary_array = load_csv_to_array("3d_binary_array.csv", x_dim, y_dim, z_dim)
    # zoom(binary_array, (0.5, 0.5, 0.5), order=0)

    for i in binary_array:
        for j in i:
            for k in j:
                if(k==4): print(1)
    
    # binary_array[binary_array == 1] = 0
    # binary_array[binary_array == 3] = 1

    # Visualize using Open3D
    visualize_voxels(binary_array)

    # Visualize 2D
    for i in range(z_dim):
        visualize_2d(binary_array[:, :, i])
