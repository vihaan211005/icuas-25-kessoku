import numpy as np
import open3d as o3d
import csv
import matplotlib.pyplot as plt
from scipy.ndimage import zoom
from scipy.spatial import ConvexHull
from scipy.ndimage import label
from sklearn.cluster import DBSCAN
from scipy.spatial import ConvexHull
import pickle

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

def save_array_to_pkl(array):
    with open("output.pkl", 'wb') as file:
        pickle.dump(array, file)

def visualize_3d_array(binary_array):
    # Create a 3D figure
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Get dimensions of the array
    x_dim, y_dim, z_dim = binary_array.shape
    print(x_dim, y_dim, z_dim)
    # Iterate through all points in the array
    for i in range(x_dim):
        for j in range(y_dim):
            for k in range(z_dim):
                value = binary_array[i, j, k]
                if value == 0 or value == 1:
                    continue  # Skip empty spaces
                
                # Assign colors based on value
                if value == 5:
                    color = 'yellow'  # Vertex
                if value == 2 or value == 3:
                    color = 'black'  
                if value == 6:
                    color = 'black'
                # Plot the point
                ax.scatter(i, j, k, color=color, marker='o')
    
    # Set axis labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Array Visualization')
    plt.show()

def save_full_array_to_csv(array, output_file):
    with open(output_file, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["x", "y", "z", "value"])  # Header
        x_dim, y_dim, z_dim = array.shape
        for i in range(x_dim):
            for j in range(y_dim):
                for k in range(z_dim):
                    if array[i, j, k] != 0:  # Exclude empty points
                        writer.writerow([i, j, k, array[i, j, k]])

def find_plane_vertices(binary_array, plane_label, jitter_amount=1e-6):
    # Locate the indices where the value matches the plane label
    coords = np.array(np.where(binary_array == plane_label)).T  # x, y, z indices

    # Label connected components
    labeled_array, num_features = label(binary_array == plane_label)

    vertices_list = []

    for component in range(1, num_features + 1):
        # Get points in this connected component
        component_coords = np.array(np.where(labeled_array == component)).T

        # Determine the projection plane
        if plane_label == 2:  # Horizontal plane
            projection = component_coords[:, :2]  # Project onto x, y
            retained_axis = 2  # Retain z-axis for reconstruction
        elif plane_label == 3:  # Vertical plane
            projection = component_coords[:, [0, 2]]  # Project onto x, z
            retained_axis = 1  # Retain y-axis for reconstruction
        else:
            continue

        # Skip if fewer than 3 unique points
        if len(projection) < 3:
            vertices_list.append(component_coords)  # Add all points as is
            continue

        # Add jitter to avoid degeneracy
        projection_with_jitter = projection + np.random.uniform(
            -jitter_amount, jitter_amount, projection.shape
        )

        # Check if points are degenerate (collinear or identical)
        if np.linalg.matrix_rank(projection_with_jitter - projection_with_jitter.mean(axis=0)) < 2:
            vertices_list.append(component_coords)  # Add all points as is
            continue

        # Compute the convex hull of the projection with jitter
        try:
            hull = ConvexHull(projection_with_jitter)

            # Map the convex hull vertices back to 3D coordinates
            hull_indices = hull.vertices
            if retained_axis == 2:  # Horizontal plane
                hull_vertices = np.column_stack((
                    projection[hull_indices],  # x, y from projection
                    component_coords[hull_indices, retained_axis]  # z from original
                ))
            elif retained_axis == 1:  # Vertical plane
                hull_vertices = np.column_stack((
                    projection[hull_indices, 0],  # x from projection
                    component_coords[hull_indices, retained_axis],  # y from original
                    projection[hull_indices, 1]  # z from projection
                ))

            vertices_list.append(hull_vertices)

        except Exception as e:
            print(f"ConvexHull failed for component {component}: {e}")
            # Add all points from this component directly
            vertices_list.append(component_coords)

    return vertices_list

def apply_majority_based_changes(binary_array, horizontal_vertices, vertical_vertices):
    # Create a copy of the binary array to avoid modifying the original
    updated_array = binary_array.copy()

    def calculate_normal(x, y, z, plane_label):
        """
        Calculate the normal vector based on the plane label.
        """
        if plane_label == 2:  # Horizontal plane
            return np.array([0, 0, 1])  # Normal is along the z-axis
        elif plane_label == 3:  # Vertical plane
            return calculate_vertical_normal(x, y, z, binary_array)  # Determine vertical normal
        else:
            return np.array([0, 0, 0])  # No normal for other labels

    # Process horizontal vertices
    for component_vertices in horizontal_vertices:
        for x, y, z in component_vertices:
            normal = calculate_normal(x, y, z, plane_label=2)
            new_point = np.array([x, y, z]) + 5 * normal  # Move 1 unit along the normal vector
            new_x, new_y, new_z = np.round(new_point).astype(int)  # Round and convert to integers
            if 0 <= new_x < binary_array.shape[0] and 0 <= new_y < binary_array.shape[1] and 0 <= new_z < binary_array.shape[2]:
                updated_array[new_x, new_y, new_z] = 6  # Mark the point along the normal

    # Process vertical vertices
    for component_vertices in vertical_vertices:
        for x, y, z in component_vertices:
            normal = calculate_normal(x, y, z, plane_label=3)
            new_point = np.array([x, y, z]) + 5 * normal  # Move 1 unit along the normal vector
            new_x, new_y, new_z = np.round(new_point).astype(int)  # Round and convert to integers
            if 0 <= new_x < binary_array.shape[0] and 0 <= new_y < binary_array.shape[1] and 0 <= new_z < binary_array.shape[2]:
                updated_array[new_x, new_y, new_z] = 6  # Mark the point along the normal

    return updated_array

def calculate_vertical_normal(x, y, z, binary_array, threshold=2):
    """
    Calculate the normal direction for a vertical plane by checking surrounding points along the x-axis and y-axis.
    """
    # Consider neighboring points along the x-axis and y-axis
    neighbors_x = [
        (x + 1, y, z),  # Neighbor in the positive x-direction
        (x - 1, y, z)   # Neighbor in the negative x-direction
    ]
    
    neighbors_y = [
        (x, y + 1, z),  # Neighbor in the positive y-direction
        (x, y - 1, z)   # Neighbor in the negative y-direction
    ]

    # Count occurrences of points in each direction for x and y
    positive_x_count = 0
    negative_x_count = 0
    positive_y_count = 0
    negative_y_count = 0

    # Check x-axis neighbors
    for nx, ny, nz in neighbors_x:
        if 0 <= nx < binary_array.shape[0] and 0 <= ny < binary_array.shape[1] and 0 <= nz < binary_array.shape[2]:
            if binary_array[nx, ny, nz] == 3:  # Check if the point is part of a vertical plane
                if nx > x:
                    positive_x_count += 1  # Point is in the positive x-direction
                elif nx < x:
                    negative_x_count += 1  # Point is in the negative x-direction

    # Check y-axis neighbors
    for nx, ny, nz in neighbors_y:
        if 0 <= nx < binary_array.shape[0] and 0 <= ny < binary_array.shape[1] and 0 <= nz < binary_array.shape[2]:
            if binary_array[nx, ny, nz] == 3:  # Check if the point is part of a vertical plane
                if ny > y:
                    positive_y_count += 1  # Point is in the positive y-direction
                elif ny < y:
                    negative_y_count += 1  # Point is in the negative y-direction

    # Determine the normal based on the majority direction along x and y
    if positive_x_count > negative_x_count:
        return np.array([1, 0, 0])  # Normal is in the positive x-direction
    elif negative_x_count > positive_x_count:
        return np.array([-1, 0, 0])  # Normal is in the negative x-direction
    elif positive_y_count > negative_y_count:
        return np.array([0, 1, 0])  # Normal is in the positive y-direction
    else:
        return np.array([0, -1, 0])  # Normal is in the negative y-direction

if __name__ == "__main__":
    # Define the dimensions of the 3D binary array
    x_dim = 107
    y_dim = 130
    z_dim = 42

    binary_array = load_csv_to_array(f"first.csv", x_dim, y_dim, z_dim)
    # Create a copy of the array to avoid modifying while iterating
    horizontal_vertices = find_plane_vertices(binary_array, plane_label=2)
    vertical_vertices = find_plane_vertices(binary_array, plane_label=3)
    for vertices in horizontal_vertices:
        for x, y, z in vertices:
            binary_array[x, y, z] = 5

    # Mark vertical vertices as 5
    for vertices in vertical_vertices:
        for x, y, z in vertices:
            binary_array[x, y, z] = 5

    updated_binary_array = apply_majority_based_changes(binary_array, horizontal_vertices, vertical_vertices)

    points = []
    colors = []

    for x in range(updated_binary_array.shape[0]):
        for y in range(updated_binary_array.shape[1]):
            for z in range(updated_binary_array.shape[2]):
                if updated_binary_array[x, y, z] > 0:  # Non-zero points
                    points.append([x, y, z])
                    if updated_binary_array[x, y, z] == 5:
                        colors.append([1, 0, 0])  # Red for marked vertices
                    elif updated_binary_array[x, y, z] == 6:
                        colors.append([0, 1, 0])
                    else:
                        colors.append([0, 0, 1])  # Blue for other points

    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))
    pcd.colors = o3d.utility.Vector3dVector(np.array(colors))

    # Visualize the point cloud
    o3d.visualization.draw_geometries([pcd])