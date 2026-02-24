import heapq

# Define the grid (0 = free space, 1 = obstacle)
grid = [
    [0, 1, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 1, 0, 0, 0],
    [0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0]
]

# Define the directions for movement (up, down, left, right)
directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

def heuristic(a, b):
    """Calculate the Manhattan distance heuristic."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_search(start, goal, grid):
    """A* search algorithm."""
    # Priority queue (min-heap) for open nodes
    open_list = []
    heapq.heappush(open_list, (0 + heuristic(start, goal), 0, start))  # (f, g, node)
    
    # Dictionary to store the parent of each node (for path reconstruction)
    came_from = {}
    
    # Dictionary to store the cost of the path to each node
    g_costs = {start: 0}
    
    while open_list:
        # Get the node with the lowest f-value
        _, g, current = heapq.heappop(open_list)
        
        # If the current node is the goal, reconstruct the path
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]  # Return reversed path
        
        # Explore neighbors
        for direction in directions:
            neighbor = (current[0] + direction[0], current[1] + direction[1])
            
            # Check if the neighbor is within the bounds and is not an obstacle
            if 0 <= neighbor[0] < len(grid) and 0 <= neighbor[1] < len(grid[0]) and grid[neighbor[0]][neighbor[1]] == 0:
                tentative_g = g + 1  # Assuming all moves have a cost of 1
                
                # If this path to the neighbor is better, record it
                if neighbor not in g_costs or tentative_g < g_costs[neighbor]:
                    g_costs[neighbor] = tentative_g
                    f_cost = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_cost, tentative_g, neighbor))
                    came_from[neighbor] = current
    
    # If there's no solution
    return None

# Example usage
start = (0, 0)  # Starting position (top-left corner)
goal = (4, 4)   # Goal position (bottom-right corner)

path = a_star_search(start, goal, grid)

if path:
    print("Path found:", path)
else:
    print("No path found")