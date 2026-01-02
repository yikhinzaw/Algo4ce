import heapq
import math


   
def astar_search(env, start, goal, mode='graph'):
       
    """
    grid: 2D list (0 = free, 1 = obstacle)
    start, goal: (row, col)
    """
    @staticmethod
    ## calculate heuristic value by using Pythagorean Theorem - Euclidean Heuristic
    def calc_heuristic_e(a, b):
        # weight of the heuristic
        w = 1
        d = w * math.hypot(a[0] - b[0], a[1] - b[1])
        return d

    @staticmethod
    ## calculate heuristic value by using Pythagorean Theorem - Euclidean Heuristic
    def heuristic(a, b):
        # Manhattan distance heuristic
        return math.fabs(a[0] - b[0]) + math.fabs(a[1] - b[1])   
        # generate final path
    def calc_final_path(self, goal_node, closed_set):
        # generate final path
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    # Priority queue: (f_score, g_score, position, path)
       # frontier: (cumulative_cost, current_node, path_history)
    frontier= [(calc_heuristic_e(start, goal), 0, start, [start])]
    #frontier = [(0, start, [])]
    explored = {}  # Stores min cost to reach a node
    nodes_expanded = 0
    all_edges = []
    edge_counts = {} 
      #Astar Search
    
    # Tracks how many times an edge is traversed
    #heapq.heappush(frontier, GridEnvironment(start,g=0,h=env.heuristic(start,goal)))
  
    while frontier:
        (totalcost,cost, current_node, path) = heapq.heappop(frontier) #f=  total cost
        # Goal Test
        if current_node == goal:
          return path + [current_node], cost, nodes_expanded, all_edges, edge_counts
        # Graph Search: Check if we've already found a cheaper way here
        if mode == 'graph':
            if current_node in explored and explored[current_node] <= cost:
                continue
            explored[current_node] = cost

        nodes_expanded += 1
        for neighbor, step_cost in env.get_heuristic_neighbors(current_node):
            new_cost = cost + step_cost
            # Create a unique key for the edge to track redundancy
            edge = tuple(sorted((current_node, neighbor)))
            edge_counts[edge] = edge_counts.get(edge, 0) + 1
            all_edges.append((current_node, neighbor))
            toalcost = new_cost+ calc_heuristic_e(neighbor, goal) # total cost start to current+ heuristic to goal
            new_path=path + [neighbor]
            heapq.heappush(frontier, (toalcost,new_cost, neighbor, new_path))

            # Yield progress for real-time visualization
            yield {
                "explored": list(explored.keys()) if mode == 'graph' else [],
                "edges": all_edges,
                "edge_counts": edge_counts,
                "nodes": nodes_expanded
            }

    return None, float('inf'), nodes_expanded, all_edges, edge_counts