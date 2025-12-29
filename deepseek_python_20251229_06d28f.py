import heapq

def astar(grid, start, goal):
    """Simplified A* implementation"""
    rows, cols = len(grid), len(grid[0])
    
    # Heuristic function (Manhattan distance)
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    # Directions: up, down, left, right
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    
    # Priority queue: (f_score, g_score, position, path)
    open_set = [(heuristic(start, goal), 0, start, [start])]
    closed_set = set()
    
    while open_set:
        f, g, current, path = heapq.heappop(open_set)
        
        if current in closed_set:
            continue
            
        if current == goal:
            return path
        
        closed_set.add(current)
        
        for dx, dy in directions:
            nx, ny = current[0] + dx, current[1] + dy
            
            if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] == 0:
                new_g = g + 1
                new_f = new_g + heuristic((nx, ny), goal)
                new_path = path + [(nx, ny)]
                heapq.heappush(open_set, (new_f, new_g, (nx, ny), new_path))
    
    return None  # No path found

# Create a 7x7 grid (0 = walkable, 1 = obstacle)
grid_7x7 = [
    [0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 0, 1, 1, 0],
    [0, 1, 0, 0, 0, 1, 0],
    [0, 0, 0, 1, 0, 0, 0],
    [0, 1, 0, 0, 0, 1, 0],
    [0, 1, 1, 0, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0]
]

# Test the algorithm
start = (0, 0)
goal = (6, 6)

path = astar(grid_7x7, start, goal)

if path:
    print(f"Path found! Length: {len(path)} steps")
    print("Path coordinates:")
    for i, (x, y) in enumerate(path):
        print(f"  Step {i}: ({x}, {y})")
    
    # Visualize path
    print("\nGrid with path (P = path, # = obstacle):")
    for x in range(7):
        for y in range(7):
            if (x, y) == start:
                print(" S ", end="")
            elif (x, y) == goal:
                print(" G ", end="")
            elif (x, y) in path:
                print(" P ", end="")
            elif grid_7x7[x][y] == 1:
                print(" # ", end="")
            else:
                print(" . ", end="")
        print()
else:
    print("No path found!")