import matplotlib.pyplot as plt
import numpy as np

def generate_costmap(goal_x, goal_y, obstacles, start_pos, grid_size=50, field_range=3.0):
    """
    Generate a costmap based on the potential field method.
    
    :param goal_x: X position of the goal.
    :param goal_y: Y position of the goal.
    :param obstacles: List of obstacles as [(x1, y1), (x2, y2), ...].
    :param start_pos: Starting position of the robot as (x, y).
    :param grid_size: Number of points in X and Y dimensions.
    :param field_range: The range of the costmap.
    :return: X, Y, U_total (potential field values)
    """
    x_range = np.linspace(-field_range, field_range, grid_size)
    y_range = np.linspace(-field_range, field_range, grid_size)
    X, Y = np.meshgrid(x_range, y_range)

    # Compute attractive potential
    k_att = 1.0
    U_att = 0.5 * k_att * ((X - goal_x)**2 + (Y - goal_y)**2)

    # Compute repulsive potential
    k_rep = 1.0
    d_safe = 0.6
    U_rep = np.zeros_like(U_att)

    for obs_x, obs_y in obstacles:
        distance = np.sqrt((X - obs_x)**2 + (Y - obs_y)**2)
        mask = distance < d_safe
        U_rep[mask] += 0.5 * k_rep * (1.0/distance[mask] - 1.0/d_safe)**2

    # Combine potential fields
    U_total = U_att + U_rep

    return X, Y, U_total

def planner(start_pos, goal_pos, obstacles, step_size=0.1, max_steps=100):
    """
    Generate a local path using a gradient descent method in the potential field.

    :param start_pos: Initial position of the robot (x, y).
    :param goal_pos: Goal position (x, y).
    :param obstacles: List of obstacles [(x1, y1), (x2, y2), ...].
    :param step_size: Step size for movement.
    :param max_steps: Maximum number of steps allowed.
    :return: List of waypoints representing the local path.
    """
    path = [start_pos]
    current_pos = np.array(start_pos)
    goal_pos = np.array(goal_pos)

    for _ in range(max_steps):
        # Compute attractive force
        F_att = -(current_pos - goal_pos)

        # Compute repulsive force
        F_rep = np.array([0.0, 0.0])
        for obs_x, obs_y in obstacles:
            obs_pos = np.array([obs_x, obs_y])
            dist = np.linalg.norm(current_pos - obs_pos)
            if dist < 0.6:  # Safe distance threshold
                rep_mag = (1.0/dist - 1.0/0.6) * (1.0/(dist**2))
                rep_vec = (current_pos - obs_pos) / dist
                F_rep += rep_mag * rep_vec

        # Compute total force
        F_total = F_att + F_rep

        # Normalize and move along the force direction
        F_norm = np.linalg.norm(F_total)
        if F_norm > 0:
            F_total = (F_total / F_norm) * step_size

        current_pos += F_total
        path.append(tuple(current_pos))

        # Stop if close to goal
        if np.linalg.norm(current_pos - goal_pos) < 0.2:
            break

    return path

# Define goal, obstacles, and start position
goal_x, goal_y = 2.0, 2.0
start_x, start_y = -2.0, -2.0
obstacles = [(0.5, 1.0), (1.5, 0.5), (1.0, 1.5)]

# Generate costmap
X, Y, U_total = generate_costmap(goal_x, goal_y, obstacles, (start_x, start_y))

# Generate path
path = planner((start_x, start_y), (goal_x, goal_y), obstacles)

# Extract path coordinates
path_x, path_y = zip(*path)

# Plot costmap
plt.figure(figsize=(8, 6))
plt.contourf(X, Y, U_total, levels=50, cmap='viridis')
plt.colorbar(label='Potential Field Value')

# Plot start, goal, and obstacles
plt.scatter(*zip(*obstacles), color='red', marker='x', label='Obstacles')
plt.scatter(goal_x, goal_y, color='blue', marker='o', label='Goal')
plt.scatter(start_x, start_y, color='green', marker='s', label='Start')

# Plot local planner path
plt.plot(path_x, path_y, color='white', linestyle='dashed', linewidth=2, label='Local Path')

# Labels and grid
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Potential Field Costmap with Local Path Planning')
plt.legend()
plt.grid()
plt.show()
