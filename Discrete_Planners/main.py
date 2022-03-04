import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import csv
import copy

from WPI_MP.Assignment_1.Assignment_1_code.search import dfs, bfs, dijkstra, astar


# Load map, start and goal point.
def load_map(file_path):
    grid = []
    start = [0, 0]
    goal = [0, 0]
    # Load from the file
    with open(file_path, 'r') as map_file:
        reader = csv.reader(map_file)
        for i, row in enumerate(reader):
            # load start and goal point
            if i == 0:
                start[0] = int(row[1])
                start[1] = int(row[2])
            elif i == 1:
                goal[0] = int(row[1])
                goal[1] = int(row[2])
            # load the map
            else:
                int_row = [int(col) for col in row]
                grid.append(int_row)
    return grid, start, goal


# Draw final results
def draw_path(grid, path, title="Path"):
    # Visualization of the found path using matplotlib
    fig, ax = plt.subplots(1)
    ax.margins()
    # Draw map
    row = len(grid)     # map size
    col = len(grid[0])  # map size
    for i in range(row):
        for j in range(col):
            if grid[i][j] == 1:
                ax.add_patch(Rectangle((j-0.5, i-0.5),1,1,edgecolor='k',facecolor='k'))  # obstacle
            # elif (grid[i][j] == -1):
            #     ax.add_patch(Rectangle((j - 0.5, i - 0.5), 1, 1, edgecolor='k', facecolor='y'))  # free space
            else:
                ax.add_patch(Rectangle((j-0.5, i-0.5),1,1,edgecolor='k',facecolor='w'))  # free space
    # Draw path
    for x, y in path:
        ax.add_patch(Rectangle((y-0.5, x-0.5),1,1,edgecolor='k',facecolor='b'))          # path
    ax.add_patch(Rectangle((start[1]-0.5, start[0]-0.5),1,1,edgecolor='k',facecolor='g'))# start
    ax.add_patch(Rectangle((goal[1]-0.5, goal[0]-0.5),1,1,edgecolor='k',facecolor='r'))  # goal
    # Graph settings
    plt.title(title)
    plt.axis('scaled')
    plt.gca().invert_yaxis()


if __name__ == "__main__":
    # Load the map
    grid, start, goal = load_map('map.csv')
    grid_bfs = copy.deepcopy(grid)
    grid_dfs = copy.deepcopy(grid)
    grid_dij = copy.deepcopy(grid)
    grid_astar = copy.deepcopy(grid)

    # start = [3, 2]

    # Search
    bfs_path, bfs_steps = bfs(grid_bfs, start, goal)
    dfs_path, dfs_steps = dfs(grid_dfs, start, goal)
    dij_path, dij_steps = dijkstra(grid_dij, start, goal)
    astar_path, astar_steps = astar(grid_astar, start, goal)

    # Show result
    draw_path(grid_bfs, bfs_path, 'BFS')
    draw_path(grid_dfs, dfs_path, 'DFS')
    draw_path(grid_dij, dij_path, 'Dijkstra')
    draw_path(grid_astar, astar_path, 'A*')
    plt.show()
