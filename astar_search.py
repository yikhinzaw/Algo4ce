import heapq
import math


   
def astar_search(env, start, goal, mode='graph'):
       
    """
    grid: 2D list (0 = free, 1 = obstacle)
    start, goal: (row, col)
    """
    # Heuristic function (Manhattan distance) 
    def heuristic(a, b):
        # Manhattan distance heuristic
        return abs(a[0] - b[0]) + abs(a[1] - b[1])   
    # Priority queue: (f_score, g_score, position, path)
       # frontier: (cumulative_cost, current_node, path_history)
    frontier= [(heuristic(start, goal), 0, start, [start])]
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
            toalcost = new_cost+ heuristic(neighbor, goal) # total cost start to current+ heuristic to goal
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