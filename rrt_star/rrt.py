import math
import numpy as np
import random
import matplotlib.pyplot as plt
import warnings

warnings.filterwarnings('ignore')

# Environment setup
SPACE_WIDTH_X = 25   # X-axis space dimension
SPACE_WIDTH_Y = 12   # Y-axis space dimension
OBSTACLE_COLUMN = 10 # Vertical obstacle column at x=10
OBSTACLE_ROWS = range(2, 9)  # Obstacle rows from y=2 to y=8

# Start and goal points
START = (6, 4)
GOAL = (17, 5)
STEP_SIZE = 0.1      # Stepping size for node expansion
TREE = []            # Tree structure to store nodes and parent connections

# RRT* parameters
SEARCH_RADIUS = 1.0  # Radius for searching nearby nodes
GOAL_THRESHOLD = 1.0 # Threshold to consider reaching the goal

# Initialize environment grid
grid = np.zeros((SPACE_WIDTH_Y, SPACE_WIDTH_X), dtype=int)
# Mark obstacles
for row in OBSTACLE_ROWS:
    grid[row, OBSTACLE_COLUMN] = 1
# Mark start and goal in grid
grid[START[1], START[0]] = 4
grid[GOAL[1], GOAL[0]] = 3

# Plotting setup
plt.figure(figsize=(10, 6))
plt.xlim(-1, SPACE_WIDTH_X)
plt.ylim(-1, SPACE_WIDTH_Y)
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.xticks(np.arange(SPACE_WIDTH_X))
plt.yticks(np.arange(SPACE_WIDTH_Y))
plt.grid(True, linestyle='--', alpha=0.7)
plt.plot(START[0], START[1], 'ro', label='Start')  # Plot start point
plt.plot(GOAL[0], GOAL[1], 'yo', label='Goal')    # Plot goal point
# Plot vertical obstacle
plt.plot([OBSTACLE_COLUMN]*len(OBSTACLE_ROWS), list(OBSTACLE_ROWS), 
         'k-', linewidth=5, label='Obstacle')

def generate_random_point(tree_nodes):
    """Generate random valid point within the space"""
    while True:
        x = random.uniform(0, SPACE_WIDTH_X - 1)
        y = random.uniform(0, SPACE_WIDTH_Y - 1)
        point = (x, y)
        
        # Ensure point is not too close to existing nodes
        if not any(math.hypot(node[0] - x, node[1] - y) < 0.5 for node in tree_nodes):
            return point

def find_nearest_node(tree_nodes, target_point):
    """Find nearest node in tree to target point using Euclidean distance"""
    min_distance = math.inf
    nearest_node = None
    
    for node in tree_nodes:
        distance = math.hypot(node[0] - target_point[0], node[1] - target_point[1])
        if distance < min_distance:
            min_distance = distance
            nearest_node = node
    return nearest_node  # Return (x, y) of nearest node

def steer_towards(target, start, step):
    """Steer from start point towards target with given step size"""
    dx = target[0] - start[0]
    dy = target[1] - start[1]
    distance = math.hypot(dx, dy)
    
    if distance == 0:
        return start  # Avoid division by zero
    
    # Calculate new point along the direction
    new_point = (
        start[0] + (dx / distance) * step,
        start[1] + (dy / distance) * step
    )
    return new_point

def is_path_obstacle_free(near_node, new_node):
    """Check if line between near_node and new_node crosses obstacle"""
    x1, y1 = near_node
    x2, y2 = new_node
    
    # Vertical line check (x1 == x2)
    if abs(x1 - x2) < 1e-6:
        y_min = min(y1, y2)
        y_max = max(y1, y2)
        return not any(grid[int(round(y)), int(round(x1))] == 1 
                      for y in np.linspace(y_min, y_max, 20))
    
    # Horizontal line check (y1 == y2)
    if abs(y1 - y2) < 1e-6:
        x_min = min(x1, x2)
        x_max = max(x1, x2)
        return not any(grid[int(round(y1)), int(round(x))] == 1 
                      for x in np.linspace(x_min, x_max, 20))
    
    # Diagonal line check using parametric equations
    steps = 20  # Number of points to check along the line
    for i in range(steps + 1):
        t = i / steps
        x = x1 + (x2 - x1) * t
        y = y1 + (y2 - y1) * t
        grid_x = int(round(x))
        grid_y = int(round(y))
        
        # Check if point is within grid bounds
        if 0 <= grid_x < SPACE_WIDTH_X and 0 <= grid_y < SPACE_WIDTH_Y:
            if grid[grid_y, grid_x] == 1:
                return False
    return True

def calculate_cost(node):
    """Calculate total cost from start to given node by backtracking parents"""
    cost = 0
    current_node = node
    while not np.allclose(current_node[:2], START, atol=1e-3):
        parent = next((n for n in TREE if np.allclose(n[:2], current_node[2:4], atol=1e-3)), None)
        if parent is None:
            break
        cost += math.hypot(current_node[0] - parent[0], current_node[1] - parent[1])
        current_node = parent
    return cost

def find_nearby_nodes(tree_nodes, center_point, radius):
    """Find nodes within specified radius from center_point"""
    nearby_nodes = []
    for node in tree_nodes:
        distance = math.hypot(node[0] - center_point[0], node[1] - center_point[1])
        if distance <= radius:
            nearby_nodes.append(node)
    return nearby_nodes

# Initialize tree with start node (x, y, parent_x, parent_y, cost)
TREE.append([*START, *START, 0.0])

# Main RRT* loop
MAX_ITERATIONS = 2000
for iteration in range(MAX_ITERATIONS):
    # Generate random point
    rand_point = generate_random_point([node[:2] for node in TREE])
    
    # Find nearest node in tree
    near_node = find_nearest_node([node[:2] for node in TREE], rand_point)
    
    # Steer towards random point
    new_point = steer_towards(rand_point, near_node, STEP_SIZE)
    
    # Check path validity and add to tree if valid
    if is_path_obstacle_free(near_node, new_point):
        # Check if new point is within grid bounds
        if (0 <= new_point[0] < SPACE_WIDTH_X and 
            0 <= new_point[1] < SPACE_WIDTH_Y):
            
            # Find nearby nodes within search radius
            nearby_nodes = find_nearby_nodes(TREE, new_point, SEARCH_RADIUS)
            
            # Calculate cost to new_point from near_node
            new_to_near_dist = math.hypot(new_point[0] - near_node[0], new_point[1] - near_node[1])
            tentative_cost = next(node[4] for node in TREE if np.allclose(node[:2], near_node, atol=1e-3)) + new_to_near_dist
            
            # Find best parent (minimum cost)
            best_parent = near_node
            min_cost = tentative_cost
            
            for node in nearby_nodes:
                # Calculate cost through this nearby node
                node_to_new_dist = math.hypot(node[0] - new_point[0], node[1] - new_point[1])
                cost_through_node = node[4] + node_to_new_dist
                
                if cost_through_node < min_cost and is_path_obstacle_free(node[:2], new_point):
                    min_cost = cost_through_node
                    best_parent = node[:2]
            
            # Add new node with best parent and cost
            TREE.append([*new_point, *best_parent, min_cost])
            
            # Plot nodes
            plt.plot(rand_point[0], rand_point[1], 'co', markersize=4)  # Random point
            plt.plot(new_point[0], new_point[1], 'mo', markersize=4)    # New node
            
            # Rewire: check if nearby nodes can benefit from new node as parent
            for node in nearby_nodes:
                new_to_node_dist = math.hypot(new_point[0] - node[0], new_point[1] - node[1])
                cost_through_new = min_cost + new_to_node_dist
                
                # If new path is better and obstacle-free
                if cost_through_new < node[4] and is_path_obstacle_free(new_point, node[:2]):
                    # Update parent and cost
                    for i, n in enumerate(TREE):
                        if np.allclose(n[:2], node[:2], atol=1e-3):
                            TREE[i][2:4] = new_point  # Update parent
                            TREE[i][4] = cost_through_new  # Update cost
                            break
            
            # Check if within goal threshold
            if math.hypot(new_point[0] - GOAL[0], new_point[1] - GOAL[1]) < GOAL_THRESHOLD:
                # Connect to goal if it results in lower cost
                goal_to_new_dist = math.hypot(GOAL[0] - new_point[0], GOAL[1] - new_point[1])
                goal_cost = min_cost + goal_to_new_dist
                
                # Check if there's already a path to goal and compare costs
                goal_in_tree = any(np.allclose(node[:2], GOAL, atol=1e-3) for node in TREE)
                
                if not goal_in_tree or goal_cost < next((node[4] for node in TREE if np.allclose(node[:2], GOAL, atol=1e-3)), math.inf):
                    TREE.append([*GOAL, *new_point, goal_cost])

# Find the best path to goal (lowest cost)
goal_nodes = [node for node in TREE if np.allclose(node[:2], GOAL, atol=1e-3)]
if not goal_nodes:
    print("No path found to goal!")
else:
    best_goal_node = min(goal_nodes, key=lambda n: n[4])
    
    # Reconstruct path
    path = [GOAL]
    current_node = best_goal_node
    
    while not np.allclose(current_node[:2], START, atol=1e-3):
        parent = next((node for node in TREE if np.allclose(node[:2], current_node[2:4], atol=1e-3)), None)
        if parent is None:
            break  # Exit if path not found
        path.append(parent[:2])
        current_node = parent
    
    path.reverse()  # Reverse to get start -> goal order
    
    # Plot path
    path_array = np.array(path)
    plt.plot(path_array[:, 0], path_array[:, 1], '-', linewidth=2, color='g', label='Path')

plt.legend()
plt.tight_layout()
plt.show()