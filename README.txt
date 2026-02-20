Project 2 – Grid-Based Search Algorithms
-----------------------------------------

Author: Ethan Hambrick n01554194

Description:
------------
This program implements four search algorithms on a 50x50 grid world:

- Depth-First Search (DFS)
- Breadth-First Search (BFS)
- Greedy Best-First Search (GBFS)
- A* Search

All algorithms use graph-search versions.
GBFS and A* use straight-line distance (Euclidean distance) as the heuristic.


How to Run:
-----------

1. Make sure Python 3.11 (or similar) is installed.
2. Install matplotlib if not already installed.
3. Make sure the folder contains:

   search.py
   grid.py
   utils.py
   TestingGrid/
       world1_enclosures.txt
       world1_turfs.txt
       world2_enclosures.txt
       world2_turfs.txt

Program Input:
--------------

The program automatically lists all available enclosure and turf files
inside the TestingGrid directory.

- Select an enclosure file by number.
- Select a turf file by number.
- Enter start coordinates (x y).
- Enter goal coordinates (x y).

Input is validated:
- File selections must exist.
- Coordinates must be integers between 0 and 49.


Program Output:
---------------

For each algorithm, the program prints:

- Path cost
- Number of nodes expanded
- Runtime (in seconds)

A matplotlib window opens showing:

- The grid
- Enclosures (black)
- Turf regions (green)
- The start node (blue)
- The goal node (red)
- The resulting path (red)

Each figure window is labeled with the algorithm name.
Images are saved automatically into the TestingGrid directory.


Notes:
------

- DFS and BFS ignore movement cost.
- GBFS uses only the heuristic.
- A* uses both path cost and heuristic.
- Runtime is measured using high-precision timing (time.perf_counter).
