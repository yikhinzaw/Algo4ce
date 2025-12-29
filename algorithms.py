import heapq


def uniform_cost_search(env, start, goal, mode='graph'):
    # frontier: (cumulative_cost, current_node, path_history)
    frontier = [(0, start, [])]
    explored = {}  # Stores min cost to reach a node
    nodes_expanded = 0
    all_edges = []
    edge_counts = {}  # Tracks how many times an edge is traversed

    while frontier:
        (cost, current_node, path) = heapq.heappop(frontier)

        # Goal Test
        if current_node == goal:
            return path + [current_node], cost, nodes_expanded, all_edges, edge_counts

        # Graph Search: Check if we've already found a cheaper way here
        if mode == 'graph':
            if current_node in explored and explored[current_node] <= cost:
                continue
            explored[current_node] = cost

        nodes_expanded += 1

        for neighbor, step_cost in env.get_neighbors(current_node):
            new_cost = cost + step_cost

            # Create a unique key for the edge to track redundancy
            edge = tuple(sorted((current_node, neighbor)))
            edge_counts[edge] = edge_counts.get(edge, 0) + 1
            all_edges.append((current_node, neighbor))

            heapq.heappush(frontier, (new_cost, neighbor, path + [current_node]))

            # Yield progress for real-time visualization
            yield {
                "explored": list(explored.keys()) if mode == 'graph' else [],
                "edges": all_edges,
                "edge_counts": edge_counts,
                "nodes": nodes_expanded
            }

    return None, float('inf'), nodes_expanded, all_edges, edge_counts