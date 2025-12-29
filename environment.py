
moves = [(0, -1), (0, 1), (-1, 0), (1, 0)] # up, down, left, right

class GridEnvironment:
    def __init__(self, width, height, obstacles=None, parent=None, g=0, h=0):
        self.width = width
        self.height = height
        self.obstacles = obstacles if obstacles else set()
        self.parent = parent
        self.g = g  # cost from start to current
        self.h = h  # heuristic to goal
        self.f = g + h  # total cost
    
    def __lt__(self, other):
        return self.f < other.f

    def get_neighbors(self, node):
        x, y = node
        neighbors = []
        for dx, dy in moves:
            nx, ny = x + dx, y + dy
            # Skip out-of-bounds or obstacles
            if 0 <= nx < self.width and 0 <= ny < self.height:
                if (nx, ny) not in self.obstacles:
                    neighbors.append(((nx, ny), 1))
        return neighbors

  
    
    def get_heuristic_neighbors(self,node):
        x,y=node
        neighbors=[]
      
        for dx, dy in moves:
            nx, ny = x + dx, y + dy
            # Skip out-of-bounds or obstacles
            if 0 <= nx < self.width and 0 <= ny < self.height :
                if (nx, ny) not in self.obstacles:
                    neighbors.append(((nx, ny), 1)) 
        return neighbors
                    
        