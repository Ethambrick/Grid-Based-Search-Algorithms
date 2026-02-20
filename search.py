import math
import time
import matplotlib.pyplot as plt

from utils import *
from grid import *


def gen_polygons(worldfilepath):
    polygons = []
    with open(worldfilepath, "r") as f:
        lines = [line.strip() for line in f.readlines()]
        for line in lines:
            polygon = []
            pts = line.split(';')
            for pt in pts:
                x, y = pt.split(',')
                polygon.append(Point(int(x), int(y)))
            polygons.append(polygon)
    return polygons


# ------------------ Helper Functions ------------------

def heuristic(p, goal):
    return math.sqrt((p.x - goal.x) ** 2 + (p.y - goal.y) ** 2)


def reconstruct_path(parent, goal):
    path = []
    curr = goal
    while curr in parent:
        path.append(curr)
        curr = parent[curr]
    path.append(curr)
    return list(reversed(path))


def compute_path_cost(path, enclosures, turfs):
    total = 0
    for i in range(1, len(path)):
        total += action_cost(path[i], enclosures, turfs)
    return total


# ------------------ BFS ------------------

def bfs(start, goal, enclosures, turfs):
    frontier = Queue()
    frontier.push(start)
    visited = set()
    parent = {}
    expanded = 0

    while not frontier.isEmpty():
        current = frontier.pop()

        if current in visited:
            continue

        visited.add(current)
        expanded += 1

        if current == goal:
            return reconstruct_path(parent, goal), expanded

        for neighbor in current.neighbors():
            cost = action_cost(neighbor, enclosures, turfs)
            if cost is not None:
                if neighbor not in visited and neighbor not in parent:
                    parent[neighbor] = current
                    frontier.push(neighbor)

    return None, expanded


# ------------------ DFS ------------------

def dfs(start, goal, enclosures, turfs):
    frontier = Stack()
    frontier.push(start)
    visited = set()
    parent = {}
    expanded = 0

    while not frontier.isEmpty():
        current = frontier.pop()

        if current in visited:
            continue

        visited.add(current)
        expanded += 1

        if current == goal:
            return reconstruct_path(parent, goal), expanded

        for neighbor in reversed(current.neighbors()):
            cost = action_cost(neighbor, enclosures, turfs)
            if cost is not None:
                if neighbor not in visited and neighbor not in parent:
                    parent[neighbor] = current
                    frontier.push(neighbor)

    return None, expanded


# ------------------ GBFS ------------------

def gbfs(start, goal, enclosures, turfs):
    frontier = PriorityQueue()
    frontier.push(start, heuristic(start, goal))
    visited = set()
    parent = {}
    expanded = 0

    while not frontier.isEmpty():
        current = frontier.pop()

        if current in visited:
            continue

        visited.add(current)
        expanded += 1

        if current == goal:
            return reconstruct_path(parent, goal), expanded

        for neighbor in current.neighbors():
            cost = action_cost(neighbor, enclosures, turfs)
            if cost is not None:
                if neighbor not in visited and neighbor not in parent:
                    parent[neighbor] = current
                    frontier.push(neighbor, heuristic(neighbor, goal))

    return None, expanded


# ------------------ A* ------------------

def astar(start, goal, enclosures, turfs):
    frontier = PriorityQueue()
    frontier.push(start, 0)
    visited = set()
    parent = {}
    g_cost = {start: 0}
    expanded = 0

    while not frontier.isEmpty():
        current = frontier.pop()

        if current in visited:
            continue

        visited.add(current)
        expanded += 1

        if current == goal:
            return reconstruct_path(parent, goal), expanded

        for neighbor in current.neighbors():
            cost = action_cost(neighbor, enclosures, turfs)
            if cost is None:
                continue

            tentative_g = g_cost[current] + cost

            if neighbor not in g_cost or tentative_g < g_cost[neighbor]:
                g_cost[neighbor] = tentative_g
                parent[neighbor] = current
                f_cost = tentative_g + heuristic(neighbor, goal)
                frontier.push(neighbor, f_cost)

    return None, expanded

# ------------------ MAIN ------------------

import os

if __name__ == "__main__":

    testing_dir = "TestingGrid"

    if not os.path.exists(testing_dir):
        print("TestingGrid directory not found.")
        exit()

    all_files = [f for f in os.listdir(testing_dir) if f.endswith(".txt")]

    enclosure_files = [f for f in all_files if "enclosure" in f.lower()]
    turf_files = [f for f in all_files if "turf" in f.lower()]

    if not enclosure_files:
        print("No enclosure files found.")
        exit()

    if not turf_files:
        print("No turf files found.")
        exit()

    # ---- Choose enclosure file ----
    print("\nAvailable Enclosure Files:")
    for i, f in enumerate(enclosure_files):
        print(f"{i + 1}. {f}")

    while True:
        try:
            choice = int(input("Select enclosure file number: "))
            if 1 <= choice <= len(enclosure_files):
                enclosure_file = enclosure_files[choice - 1]
                break
            else:
                print("Invalid selection.")
        except:
            print("Please enter a number.")

    # ---- Choose turf file ----
    print("\nAvailable Turf Files:")
    for i, f in enumerate(turf_files):
        print(f"{i + 1}. {f}")

    while True:
        try:
            choice = int(input("Select turf file number: "))
            if 1 <= choice <= len(turf_files):
                turf_file = turf_files[choice - 1]
                break
            else:
                print("Invalid selection.")
        except:
            print("Please enter a number.")

    # ---- Get start and goal ----
    while True:
        try:
            start_input = input("Enter start x y (0–49): ")
            sx, sy = map(int, start_input.split())

            goal_input = input("Enter goal x y (0–49): ")
            gx, gy = map(int, goal_input.split())

            if 0 <= sx < 50 and 0 <= sy < 50 and 0 <= gx < 50 and 0 <= gy < 50:
                break
            else:
                print("Coordinates must be between 0 and 49.")
        except:
            print("Invalid input. Please enter integers separated by space.")

    enclosure_path = os.path.join(testing_dir, enclosure_file)
    turf_path = os.path.join(testing_dir, turf_file)

    epolygons = gen_polygons(enclosure_path)
    tpolygons = gen_polygons(turf_path)

    source = Point(sx, sy)
    dest = Point(gx, gy)

    algorithms = [
        ("DFS", dfs),
        ("BFS", bfs),
        ("GBFS", gbfs),
        ("A*", astar)
    ]

    for name, algo in algorithms:

        print(f"\n--- {name} ---")

        start_time = time.perf_counter()

        path, expanded = algo(source, dest, epolygons, tpolygons)

        end_time = time.perf_counter()
        runtime = end_time - start_time

        if path is None:
            print("No path found.")
            continue

        cost = compute_path_cost(path, epolygons, tpolygons)

        print("Path cost:", cost)
        print("Nodes expanded:", expanded)
        print(f"Runtime: {runtime:.6f} seconds")

        # --------- DRAWING ---------

        fig, ax = draw_board()
        fig.canvas.manager.set_window_title(f"{name} Search")

        draw_grids(ax)
        draw_source(ax, source.x, source.y)
        draw_dest(ax, dest.x, dest.y)

        # Draw enclosures
        for polygon in epolygons:
            for p in polygon:
                draw_point(ax, p.x, p.y)
            for i in range(len(polygon)):
                draw_line(ax,
                          [polygon[i].x, polygon[(i+1)%len(polygon)].x],
                          [polygon[i].y, polygon[(i+1)%len(polygon)].y])

        # Draw turfs
        for polygon in tpolygons:
            for p in polygon:
                draw_green_point(ax, p.x, p.y)
            for i in range(len(polygon)):
                draw_green_line(ax,
                                [polygon[i].x, polygon[(i+1)%len(polygon)].x],
                                [polygon[i].y, polygon[(i+1)%len(polygon)].y])

        # Draw path
        for i in range(len(path)-1):
            draw_result_line(ax,
                             [path[i].x, path[i+1].x],
                             [path[i].y, path[i+1].y])

        plt.title(f"{name} Search", fontsize=14)
        safe_name = name.replace("*", "star")
        world_name = enclosure_file.split("_")[0]
        plt.savefig(f"TestingGrid/{safe_name}_{world_name}.png")
        plt.show()
