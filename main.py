import pygame
import sys
import time
import random
import os
import tracemalloc
from environment import GridEnvironment
from algorithms import uniform_cost_search, bfs_search, dfs_search
from astar_search import astar_search

# --- CONFIGURATION ---
SEARCH_DELAY = 30
MOVE_DELAY = 150
HUD_H = 80

# Colors
BG, LINE, OBS = (245, 245, 245), (200, 200, 200), (40, 40, 40)
EXPLORED_C, PATH_RED = (210, 235, 255), (255, 100, 100)
BOOK_C, SHELF_C = (255, 50, 50), (40, 200, 40)


def load_and_scale(path, size):
    if os.path.exists(path):
        try:
            img = pygame.image.load(path).convert_alpha()
            return pygame.transform.scale(img, (size, size))
        except:
            return None
    return None


def generate_fixed_world(grid_size):
    # Fixed Layout from User's original code
    obs = set()
    books = []
    shelf = (grid_size-1, grid_size-1)

    if grid_size == 12: # GRAPH SEARCH
        # 1. Obstacles
        obs = {
            (2, 2), (2, 3), (2, 4), (2, 5), # Wall 1
            (8, 8), (8, 9), (9, 8),         # Block 2
            (5, 5), (5, 6), (6, 5)          # Center Cluster
        }
        
        # 2. Books
        books = [
            (1, 10),  # Bottom Left
            (10, 1),  # Top Right
            (10, 10)  # Bottom Right
        ]
        shelf = (1, 1) # Top Left Goal

    else: # TREE SEARCH (7x7)
        # 1. Obstacles
        obs = {
            (1, 1), (1, 2), # Vertical
            (4, 4), (5, 4), # Horizontal
            (3, 2)
        }

        # 2. Books
        # UPDATED POSITION based on previous task: Book 2 at (0, 5)
        books = [
            (0, 6), # Book 1: Bottom Left
            (0, 5), # Book 2: Near Book 1 (Speed up)
            (3, 6)  # Book 3: Bottom Middle
        ]
        shelf = (6, 6) # Shelf: Bottom Right

    # Filter bounds
    obs = {pos for pos in obs if pos[0] < grid_size and pos[1] < grid_size}
    books = [b for b in books if b[0] < grid_size and b[1] < grid_size and b not in obs]
    
    return obs, books, shelf


def draw_all(screen, env, agent, books, shelf, nodes, mode, cell_size, sh, sw, static_walls, icons, explored=None,
             edges=None, edge_counts=None, elapsed_time=0, cost=0):
    screen.fill(BG)

    # 1. Draw Explored Background (Graph Mode Only)
    if explored:
        for n in explored:
            pygame.draw.rect(screen, EXPLORED_C, (n[0] * cell_size, n[1] * cell_size, cell_size, cell_size))

    # 2. Draw Search Tree
    if edges and edge_counts:
        for s_node, e_node in edges:
            edge_key = tuple(sorted((s_node, e_node)))
            thickness = min(8, edge_counts.get(edge_key, 1))

            s_pos = (s_node[0] * cell_size + cell_size // 2, s_node[1] * cell_size + cell_size // 2)
            e_pos = (e_node[0] * cell_size + cell_size // 2, e_node[1] * cell_size + cell_size // 2)
            pygame.draw.line(screen, PATH_RED, s_pos, e_pos, thickness)

    # 3. Draw Grid & Static Walls
    for x in range(env.width):
        for y in range(env.height):
            pygame.draw.rect(screen, LINE, (x * cell_size, y * cell_size, cell_size, cell_size), 1)
            if (x, y) in static_walls:
                pygame.draw.rect(screen, OBS, (x * cell_size, y * cell_size, cell_size, cell_size))

    # 4. Draw Icons
    if icons.get('shelf'):
        screen.blit(icons['shelf'], (shelf[0] * cell_size, shelf[1] * cell_size))

    for b in books:
        if icons.get('book'):
            screen.blit(icons['book'], (b[0] * cell_size, b[1] * cell_size))

    if icons.get('robot'):
        screen.blit(icons['robot'], (agent[0] * cell_size, agent[1] * cell_size))

    # 5. HUD
    pygame.draw.rect(screen, (220, 220, 220), (0, sh - HUD_H, sw, HUD_H))
    font = pygame.font.SysFont("Arial", 16, bold=True)
    status_text = f"MODE: {mode.upper()} | Nodes: {nodes} | Time: {elapsed_time:.2f}s | Cost: {cost}"
    screen.blit(font.render(status_text, True, (0, 0, 0)), (20, sh - 55))
    screen.blit(font.render(f"Books Left: {len(books)}", True, (0, 0, 0)), (20, sh - 30))


def run_simulation(mode, algorithm_name, search_func, search_kwargs=None):
    grid_size = 12 if mode == 'graph' else 7
    cell_size = 50 if mode == 'graph' else 80
    sw, sh = grid_size * cell_size, (grid_size * cell_size) + HUD_H
    screen = pygame.display.set_mode((sw, sh))
    pygame.display.set_caption(f"Library Robot - {mode.upper()} - {algorithm_name}")

    icons = {'robot': load_and_scale("robot.png", cell_size),
             'book': load_and_scale("book.png", cell_size),
             'shelf': load_and_scale("shelf.png", cell_size)}

    # Use Fixed World Generation
    static_walls, current_books, shelf = generate_fixed_world(grid_size)
    
    agent_pos = (0, 0)
    total_nodes = 0
    total_cost = 0

    # Timer Start
    start_time = time.time()

    # Mission: All books first, THEN the shelf
    mission_targets = list(current_books) + [shelf]

    for target in mission_targets:
        dynamic_obs = set(static_walls)
        for t in mission_targets:
            if t != target:
                dynamic_obs.add(t)
        
        env = GridEnvironment(grid_size, grid_size, static_walls, None, g=0, h=0)        
        env.obstacles = dynamic_obs
        
        search_gen = search_func(env, agent_pos, target, mode=mode, **(search_kwargs or {}))
        path, edge_counts = None, {}
        current_cost = 0

        searching = True
        while searching:
            for event in pygame.event.get():
                if event.type == pygame.QUIT: pygame.quit(); sys.exit()
            
            try:
                data = next(search_gen)
                edge_counts = data["edge_counts"]
                
                # Calculate metrics for display
                elapsed = time.time() - start_time
                
                draw_all(screen, env, agent_pos, current_books, shelf,
                         total_nodes + data["nodes"], mode, cell_size, sh, sw,
                         static_walls, icons, data["explored"], data["edges"], edge_counts, elapsed, total_cost)
                pygame.display.flip()
                pygame.time.delay(SEARCH_DELAY)
            
            except StopIteration as e:
                # Search finished for this target
                if e.value and e.value[0]:
                    path, path_cost, nodes_count, _, edge_counts = e.value
                    total_nodes += nodes_count
                    current_cost = path_cost
                searching = False

        if path:
            total_cost += current_cost
            for step in path[1:]:
                agent_pos = step
                elapsed = time.time() - start_time
                draw_all(screen, env, agent_pos, current_books, shelf, total_nodes, mode, cell_size, sh, sw,
                         static_walls, icons, None, None, None, elapsed, total_cost)
                pygame.display.flip()
                pygame.time.delay(MOVE_DELAY)

            if agent_pos in current_books:
                current_books.remove(agent_pos)

    # Final Stats
    end_time = time.time()
    final_time = end_time - start_time
      ##calculate storage size 
    current, peak = tracemalloc.get_traced_memory()
    print(f"Current memory usage is {current }; Peak was {peak / 10**3}KB")
    tracemalloc.stop()
    
    print(f"[{mode.upper()}] Finished in {final_time:.4f} seconds. Total Cost: {total_cost}")
   
    
    # Pause to show result
    time.sleep(2)


def main():
    pygame.init()
    tracemalloc.start()
    print("--- Library Robot Algorithm Selection ---")
    print("1. BFS (Breadth-First Search)")
    print("2. UCS (Uniform Cost Search)")
    print("3. A* (A-Star Search)")
    print("4. DFS (Depth-First Search)")
    
    choice = input("Select Algorithm (1-4): ").strip()
    
    algo_map = {
        '1': ('BFS', bfs_search),
        '2': ('UCS', uniform_cost_search),
        '3': ('A*', astar_search),
        '4': ('DFS', dfs_search)
    }
    
    selected = algo_map.get(choice, ('BFS', bfs_search))
    print(f"Selected: {selected[0]}")
    
    # Run user requested sequence: Graph then Tree
    run_simulation('graph', selected[0], selected[1])

    # For tree mode, pass a depth_limit for DFS to ensure termination on small grids
    tree_search_kwargs = {'depth_limit': 20} if selected[0] == 'DFS' else None
    run_simulation('tree', selected[0], selected[1], search_kwargs=tree_search_kwargs)
    
    pygame.quit()


if __name__ == "__main__":
    main()