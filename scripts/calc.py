import numpy as np

# Initial position and goal
p0 = np.array([0, 0, 0])  # Starting position (0, 0, 0)
goal = np.array([20, 10, 15])  # Goal position (20, 10, 15)

# Step 1: Calculate tau
tau = 30 / 16  # Given tau calculation
c = 30 / tau**5  # Calculate constant c
distance_tau = c * (tau**6) / 60  # Calculate distance_tau

# Step 2: Calculate distance from p0 to the goal
dist = np.linalg.norm(goal - p0)  # Euclidean distance between start and goal

# Step 3: Calculate the final duration
duration = 2 * tau + dist - 2 * distance_tau

# Print the results
print(f"Tau: {tau}")
print(f"Constant c: {c}")
print(f"Distance Tau: {distance_tau}")
print(f"Distance to goal: {dist}")
print(f"Final Duration: {duration}")
