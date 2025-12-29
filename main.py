import pygame
import sys
import time
import random
import os
from environment import GridEnvironment
from algorithms import uniform_cost_search
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


def generate_random_world(grid_size, num_obstacles, num_books):
    obs = set()
    while len(obs) < num_obstacles:
        candidate = (random.randint(0, grid_size - 1), random.randint(0, grid_size - 1))
        if candidate != (0, 0): obs.add(candidate)

    all_spots = [(x, y) for x in range(grid_size) for y in range(grid_size)
                 if (x, y) not in obs and (x, y) != (0, 0)]
    random.shuffle(all_spots)
    books = all_spots[:num_books]
    shelf = all_spots[num_books]
    return obs, books, shelf


def draw_all(screen, env, agent, books, shelf, nodes, mode, cell_size, sh, sw, static_walls, icons, explored=None,
             edges=None, edge_counts=None):
    screen.fill(BG)

    # 1. Draw Explored Background (Graph Mode Only)
    if explored:
        for n in explored:
            pygame.draw.rect(screen, EXPLORED_C, (n[0] * cell_size, n[1] * cell_size, cell_size, cell_size))

    # 2. Draw Search Tree with Thickness for Redundancy
    if edges and edge_counts:
        for s_node, e_node in edges:
            edge_key = tuple(sorted((s_node, e_node)))
            # Thicker lines for revisited squares
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

    # 4. Draw Icons (Shelf, Books, then Robot)
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
    screen.blit(font.render(f"MODE: {mode.upper()} | Nodes: {nodes} | Books Left: {len(books)}", True, (0, 0, 0)),
                (20, sh - 45))


def run_simulation(mode):
    grid_size = 12 if mode == 'graph' else 7
    cell_size = 50 if mode == 'graph' else 80
    sw, sh = grid_size * cell_size, (grid_size * cell_size) + HUD_H
    screen = pygame.display.set_mode((sw, sh))
    pygame.display.set_caption(f"Library Robot - {mode.upper()}")

    icons = {'robot': load_and_scale("robot.png", cell_size),
             'book': load_and_scale("book.png", cell_size),
             'shelf': load_and_scale("shelf.png", cell_size)}

    num_obs = int((grid_size * grid_size) * 0.18)
    static_walls, current_books, shelf = generate_random_world(grid_size, num_obs, 3)
    
    agent_pos = (0, 0)
    total_nodes = 0

    # Mission: All books first, THEN the shelf
    mission_targets = list(current_books) + [shelf]

    for target in mission_targets:
        # DYNAMIC OBSTACLE LOGIC:
        # Treat every target (shelf/other books) as a wall UNLESS it's the current goal
        dynamic_obs = set(static_walls)
        for t in mission_targets:
            if t != target:
                dynamic_obs.add(t)
        env = GridEnvironment(grid_size, grid_size, static_walls,None,g=0,h=0)        
        env.obstacles = dynamic_obs
        #search_gen = uniform_cost_search(env, agent_pos, target, mode=mode)
        search_gen = astar_search(env, agent_pos, target, mode=mode)
        path, edge_counts = None, {}

        searching = True
        while searching:
            for event in pygame.event.get():
                if event.type == pygame.QUIT: pygame.quit(); sys.exit()
            try:
                data = next(search_gen)
                edge_counts = data["edge_counts"]
                draw_all(screen, env, agent_pos, current_books, shelf,
                         total_nodes + data["nodes"], mode, cell_size, sh, sw,
                         static_walls, icons, data["explored"], data["edges"], edge_counts)
                pygame.display.flip()
                pygame.time.delay(SEARCH_DELAY)
            except StopIteration as e:
                if e.value and e.value[0]:
                    path, _, nodes_count, _, edge_counts = e.value
                    total_nodes += nodes_count
                searching = False

        if path:
            for step in path[1:]:
                agent_pos = step
                draw_all(screen, env, agent_pos, current_books, shelf, total_nodes, mode, cell_size, sh, sw,
                         static_walls, icons)
                pygame.display.flip()
                pygame.time.delay(MOVE_DELAY)

            if agent_pos in current_books:
                current_books.remove(agent_pos)

    time.sleep(1)


def main():
    pygame.init()
    run_simulation('graph')
    run_simulation('tree')
    pygame.quit()


if __name__ == "__main__":
    main()