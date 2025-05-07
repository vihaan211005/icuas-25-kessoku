import matplotlib.pyplot as plt  # Corrected import
import numpy as np

def plotter(world_width, world_height, r, buildings, visible_arcs_building, poses, edge, matrix):
    fig, ax = plt.subplots(figsize=(6, 8))
    ax.set_aspect('equal')
    ax.set_xlim(0, world_width)
    ax.set_ylim(0, world_height)

    ax.set_xticks(np.arange(0, world_width + edge, edge))
    ax.set_yticks(np.arange(0, world_height + edge, edge))
    ax.grid(True)

    for bx, by in buildings:
        circle = plt.Circle((bx, by), r, color='gray')
        ax.add_patch(circle)

    for obs_x, obs_y in poses:
        ax.plot(obs_x * edge, obs_y* edge, 'ro', label='Observer')

    for i, (bx, by) in enumerate(buildings):
        for start, end in visible_arcs_building[i]:
            theta = np.linspace(start, end, 30)
            x = bx + r * np.cos(theta)
            y = by + r * np.sin(theta)
            ax.plot(x, y, 'g', linewidth=2)

    new_poses = poses.copy()
    new_poses.append((0,0))

    for i in range(len(new_poses)):
        for j in range(i + 1, len(new_poses)):
            if(matrix[new_poses[i][0]][new_poses[i][1]][new_poses[j][0]][new_poses[j][1]]):
                start_x = new_poses[i][0] * edge
                start_y = new_poses[i][1] * edge
                end_x = new_poses[j][0] * edge
                end_y = new_poses[j][1] * edge
                x_lin = np.linspace(start_x, end_x, 30)
                y_lin = np.linspace(start_y, end_y, 30)
                ax.plot(x_lin, y_lin, 'b', linewidth = 3)

    plt.title("Visible Building Arcs")
    plt.show()

def plot_matrix(world_width, world_height, r, buildings, edge, matrix):
    m = (int)(world_width / edge) + 1
    n = (int)(world_height / edge) + 1

    for i1 in range(m):
        for j1 in range(n):
            fig, ax = plt.subplots(figsize=(6, 8))
            ax.set_aspect('equal')
            ax.set_xlim(0, world_width)
            ax.set_ylim(0, world_height)

            ax.set_xticks(np.arange(0, world_width + edge, edge))
            ax.set_yticks(np.arange(0, world_height + edge, edge))
            ax.grid(True)

            for bx, by in buildings:
                circle = plt.Circle((bx, by), r, color='gray')
                ax.add_patch(circle)

            for i2 in range(m):
                for j2 in range(n):
                    if matrix[i1][j1][i2][j2]:
                        ax.plot(j2 * edge, i2 * edge, 'go')
            ax.plot(i1 * edge, j1 * edge, 'ro', ms=3)
            plt.title("Matrix")
            plt.show()

def stats(data):
    # Filter out -inf for stats and some plots
    data = np.array(data)
    mask = np.isfinite(data)
    finite_data = data[mask]

    # Print statistics
    print("Count (finite):", finite_data.size)
    print("Mean:", np.mean(finite_data))
    print("Median:", np.median(finite_data))
    print("Standard Deviation:", np.std(finite_data))
    print("Min:", np.min(finite_data))
    print("Max:", np.max(finite_data))

    # ---- Plotting ----

    # 1. Line Plot (showing -inf as a gap)
    plt.figure(figsize=(12, 4))
    plt.subplot(1, 3, 1)
    plot_data = np.copy(data)
    plot_data[~np.isfinite(plot_data)] = np.nan  # Replace -inf with NaN
    plt.plot(plot_data, marker='o', label='Data')
    plt.title('Line Plot')
    plt.xlabel('Index')
    plt.ylabel('Value')
    plt.grid(True)

    # 2. Histogram (only finite values)
    plt.subplot(1, 3, 2)
    plt.hist(finite_data, bins=5, edgecolor='black')
    plt.title('Histogram (finite only)')
    plt.xlabel('Value')
    plt.ylabel('Frequency')
    plt.grid(True)

    # 3. Box Plot (only finite values)
    plt.subplot(1, 3, 3)
    plt.boxplot(finite_data, vert=True)
    plt.title('Box Plot (finite only)')
    plt.grid(True)

    # Show all plots
    plt.tight_layout()
    plt.show()
