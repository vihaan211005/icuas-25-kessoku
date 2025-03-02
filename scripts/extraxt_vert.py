import numpy as np
from stl import mesh
import math
import csv 
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

def angle_between_vectors(v1, v2):
    """Calculates the angle between two vectors in radians."""
    v1_u = v1 / np.linalg.norm(v1)
    v2_u = v2 / np.linalg.norm(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def combine_coplanar_triangles(stl_file, angle_tolerance_degrees=1.0):
    """
    Combines coplanar triangles from an STL file into quadrilaterals.

    Args:
        stl_file (str): Path to the STL file.
        angle_tolerance_degrees (float): Maximum angle difference (in degrees)
                                         between normals to consider triangles
                                         coplanar.

    Returns:
        list: A list of quadrilaterals (each represented as a list of 4 vertices).
    """

    your_mesh = mesh.Mesh.from_file(stl_file)
    triangles = your_mesh.vectors
    normals = your_mesh.normals

    angle_tolerance_radians = np.radians(angle_tolerance_degrees)
    quadrilaterals = []
    processed = [False] * len(triangles)

    for i in range(len(triangles)):
        if processed[i]:
            continue

        for j in range(i + 1, len(triangles)):
            if processed[j]:
                continue

            if angle_between_vectors(normals[i], normals[j]) <= angle_tolerance_radians:
                # Check for shared edge
                shared_vertices = 0
                shared_edge = None
                for k in range(3):
                    for l in range(3):
                        if np.all(triangles[i][k] == triangles[j][l]):
                            shared_vertices += 1
                            if shared_edge is None:
                                shared_edge = (k, l)
                            elif shared_edge != None:
                                shared_edge2 = (k,l)

                if shared_vertices == 2:
                    # Combine triangles into a quadrilateral
                    quad_vertices = []
                    for k in range(3):
                        if k != shared_edge[0] and k != ((shared_edge[0]+1)%3) and k!=((shared_edge[0]-1)%3):
                            quad_vertices.append(triangles[i][k])
                    for l in range(3):
                        if l != shared_edge[1] and l!=((shared_edge[1]+1)%3) and l!=((shared_edge[1]-1)%3):
                            quad_vertices.append(triangles[j][l])

                    quadrilaterals.append(quad_vertices)
                    processed[i] = True
                    processed[j] = True
                    break

    # Add remaining triangles (not combined)
    uncombined = 0
    for i in range(len(triangles)):
        if not processed[i]:
            quadrilaterals.append(triangles[i])
            uncombined += 1

    return quadrilaterals, len(triangles), uncombined

def plot_vertices_from_csv(stl_file_path, csv_file_path, axis_scale_factor=0.5):
    """
    Plots the vertices from a CSV file and the original STL mesh.

    Args:
        stl_file_path (str): Path to the STL file.
        csv_file_path (str): Path to the CSV file containing vertices.
        axis_scale_factor (float): Factor to scale the axis limits.
                                    Smaller values make the buildings appear larger.
    """

    # Load STL mesh
    your_mesh = mesh.Mesh.from_file(stl_file_path)

    # Load vertices from CSV
    vertices = []
    with open(csv_file_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # Skip header row
        for row in reader:
            vertices.append([float(x) for x in row])
    vertices = np.array(vertices)

    # Plot
    fig = plt.figure(figsize=(14, 14))
    ax = fig.add_subplot(111, projection='3d')

    # Plot STL mesh
    ax.add_collection3d(mplot3d.art3d.Poly3DCollection(your_mesh.vectors, alpha=0.25))

    # Plot vertices from CSV
    ax.scatter(vertices[:, 0], vertices[:, 1], vertices[:, 2], c='r', marker='o', s = 0.1)

    # Auto scale to the mesh size
    scale = your_mesh.points.flatten()
    ax.auto_scale_xyz(scale, scale, scale)  # Initial auto-scaling

    # Adjust axis limits
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()
    zlim = ax.get_zlim()
    ax.set_xlim(xlim[0] * axis_scale_factor, xlim[1] * axis_scale_factor)
    ax.set_ylim(ylim[0] * axis_scale_factor, ylim[1] * axis_scale_factor)
    ax.set_zlim(zlim[0] * axis_scale_factor, zlim[1] * axis_scale_factor)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.title("STL Mesh and CSV Vertices (Adjusted Axis Limits)")
    plt.show()

# Example usage
stl_file_path = 'worlds/city_1_small/meshes/city_1_small.stl'  # Replace with your STL file
csv_file_path = 'worlds/quadrilateral_vertices.csv' # output csv
# combined_quads, initial_triangle_count, uncombined = combine_coplanar_triangles(stl_file_path)

# print(f"Initial number of triangles: {initial_triangle_count}")
# print(f"Final number of shapes: {len(combined_quads)}")
# print(f"Number of uncombined triangles: {uncombined}")

# # Write all vertices to CSV, one per row
# with open(csv_file_path, 'w', newline='') as csvfile:
#     writer = csv.writer(csvfile)
#     writer.writerow(['x', 'y', 'z']) #header
#     for shape in combined_quads:
#         for vertex in shape:
#             writer.writerow(vertex)

# print(f"All vertices written to {csv_file_path}")

plot_vertices_from_csv(stl_file_path, csv_file_path)
