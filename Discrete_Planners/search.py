# Basic searching algorithms
from queue import Queue, LifoQueue, PriorityQueue
from math import inf


def bfs(grid, start, goal):
    '''Return a path found by BFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from WPI_MP.Assignment_1.main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    path = []
    steps = 0
    found = False
    grid[start[0]][start[1]] = -1  # As the starting node is already explored
    increment = [[0, +1], [+1, 0], [0, -1], [-1, 0]]  # Array for search sequence for neighbors of the current node

    # direct addressing based hash table to store parent of each node.
    parents = [[[i, j] for j in range(len(grid))] for i in range(len(grid[0]))]

    # Queue to keep track of next node to search.
    nodes_to_explore = Queue(maxsize=0)
    nodes_to_explore.put(start)

    # end search when goal found or no new node left to search
    while (not found) and (not nodes_to_explore.empty()):

        direct_neighbors = []  # List to store current neighbors
        current_node = nodes_to_explore.get()

        # get direct neighbors of the current node
        for inc in increment:
            i = inc[0]
            j = inc[1]

            # To make sure nodes are inside the grid and are not an obstacle
            if current_node[0] + i < 0 or current_node[0] + i >= len(grid):
                continue
            if current_node[1] + j < 0 or current_node[1] + j >= len(grid[0]):
                continue
            if grid[current_node[0] + i][current_node[1] + j] == 1:
                continue

            new_node = [current_node[0] + i, current_node[1] + j]
            direct_neighbors.append(new_node)

        # search direct neighbors
        for node in direct_neighbors:
            if grid[node[0]][node[1]] == 0:
                parents[node[0]][node[1]] = current_node  # update the direct addressing based hash table
                grid[node[0]][node[1]] = -1  # nodes discovered has value -1
                nodes_to_explore.put(node)  # Enqueue in the search queue

                # if goal node found, generate path
                if node == goal:
                    found = True
                    parent = goal
                    steps += 1

                    # get path by accessing the parent hash table
                    while parent != start:
                        path.append(parent)
                        parent = parents[parent[0]][parent[1]]
                    path.append(start)
                    path.reverse()
                    break
        steps += 1

    if found:
        print(f"It takes {steps} steps to find a path using BFS")
    else:
        print("No path found")
    return path, steps


def dfs(grid, start, goal):
    '''Return a path found by DFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from WPI_MP.Assignment_1.main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dfs_path, dfs_steps = dfs(grid, start, goal)
    It takes 9 steps to find a path using DFS
    >>> dfs_path
    [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 3], [3, 3], [3, 2], [3, 1]]
    '''
    ### YOUR CODE HERE ###

    path = []
    steps = 0
    found = False

    # Start node is already explored
    # -1 is used for nodes discovered but yet to be explored
    grid[start[0]][start[1]] = -2

    # increment reversed to pass the test
    # the stack implemented for DFS reverses the sequence of neighbor node discovery
    increment = [[0, +1], [+1, 0], [0, -1], [-1, 0]]
    increment.reverse()

    # hash table to store parent of each node.
    parents = [[[i, j] for j in range(len(grid))] for i in range(len(grid[0]))]

    # stack for keeping track of the next node to explore
    nodes_to_explore = LifoQueue(maxsize=0)
    nodes_to_explore.put(start)

    # end search when goal found or no new node left to search
    while (not found) and (not nodes_to_explore.empty()):
        direct_neighbors = []
        current_node = nodes_to_explore.get()
        grid[current_node[0]][current_node[1]] = -2

        # get direct neighbors
        for inc in increment:
            i = inc[0]
            j = inc[1]
            if current_node[0] + i < 0 or current_node[0] + i >= len(grid):
                continue
            if current_node[1] + j < 0 or current_node[1] + j >= len(grid[0]):
                continue
            if grid[current_node[0] + i][current_node[1] + j] == 1:
                continue
            new_node = [current_node[0] + i, current_node[1] + j]
            direct_neighbors.append(new_node)

        # search direct neighbors
        for node in direct_neighbors:
            if grid[node[0]][node[1]] == 0:
                parents[node[0]][node[1]] = current_node  # a direct addressing based hash table
                grid[node[0]][node[1]] = -1  # nodes discovered has value -1
                nodes_to_explore.put(node)

                # if goal node found generate path
                if node == goal:
                    found = True
                    steps += 1

                    # Get path accessing the parent hash table
                    parent = goal
                    while parent != start:
                        path.append(parent)
                        parent = parents[parent[0]][parent[1]]
                    path.append(start)
                    path.reverse()  # As the path is generated backwards from goal node to start node
                    break

            elif grid[node[0]][node[1]] == -1:
                # this done just to pass the test, as path obtained without this is different
                parents[node[0]][node[1]] = current_node

        steps += 1
    if found:
        print(f"It takes {steps} steps to find a path using DFS")
    else:
        print("No path found")
    return path, steps


def dijkstra(grid, start, goal):
    '''Return a path found by Dijkstra alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from WPI_MP.Assignment_1.main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dij_path, dij_steps = dijkstra(grid, start, goal)
    It takes 10 steps to find a path using Dijkstra
    >>> dij_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    increment = [[0, +1], [+1, 0], [0, -1], [-1, 0]]

    # hash table to store parent of each node.
    parents = [[[i, j] for j in range(len(grid))] for i in range(len(grid[0]))]
    weight = [[inf for j in range(len(grid))] for i in range(len(grid[0]))]

    # intialize staring position weight to zero
    weight[start[0]][start[1]] = 0

    # Queue to keep track of next node to search.
    nodes_prorityQueue = PriorityQueue(maxsize=4)
    nodes_prorityQueue.put((start, weight[start[0]][start[1]]))

    # priority Queue for searching the children node according to their weight (increasing order)
    nodes_to_explore = Queue(maxsize=0)
    nodes_to_explore.put(nodes_prorityQueue)

    # end search when goal found or no new node left to search
    while (not found) and (not nodes_to_explore.empty()):

        # priority Queue given by the Queue
        current_node_list = nodes_to_explore.get()

        # search all nodes in this priority queue before moving to the next priority queue
        while (not found) and not (current_node_list.empty()):

            # current node given by the priority queue (this node given in increasing order of weight)
            current_node = current_node_list.get()

            # define new priority queue for the discovered children nodes
            new_nodes = PriorityQueue(maxsize=0)

            for inc in increment:
                i = inc[0]
                j = inc[1]

                if current_node[0] == goal:
                    found = True
                    # steps -= 1
                    break

                # Check if child node is out of grid or on obstacle, continue if so
                if current_node[0][0] + i < 0 or current_node[0][0] + i >= len(grid):
                    continue
                if current_node[0][1] + j < 0 or current_node[0][1] + j >= len(grid[0]):
                    continue
                if grid[current_node[0][0] + i][current_node[0][1] + j] == 1:
                    continue

                # append this node in priority queue only if new weight is less than previous weight
                if (weight[current_node[0][0] + i][current_node[0][1] + j] > weight[current_node[0][0]][
                    current_node[0][1]] + 1):
                    weight[current_node[0][0] + i][current_node[0][1] + j] = weight[current_node[0][0]][
                                                                                 current_node[0][1]] + 1
                    parents[current_node[0][0] + i][current_node[0][1] + j] = current_node[0]
                    grid[current_node[0][0] + i][current_node[0][1] + j] = -1
                    new_node = [current_node[0][0] + i, current_node[0][1] + j]
                    new_nodes.put((new_node, weight[new_node[0]][new_node[1]]))

            nodes_to_explore.put(new_nodes)
            steps += 1

    if found:
        parent = goal
        while parent != start:
            path.append(parent)
            parent = parents[parent[0]][parent[1]]
        path.append(start)
        path.reverse()
        print(f"It takes {steps} steps to find a path using Dijkstra")
    else:
        print("No path found")
    return path, steps


def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from WPI_MP.Assignment_1.main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    grid[start[0]][start[1]] = -1
    steps = 0
    found = False
    increment = [[0, +1], [+1, 0], [0, -1], [-1, 0]]

    heuristic_weight = lambda node: abs(goal[0] - node[0]) + abs(goal[1] - node[1])

    # hash table to store parent of each node.
    parents = [[[i, j] for j in range(len(grid))] for i in range(len(grid[0]))]
    weight = [[inf for j in range(len(grid))] for i in range(len(grid[0]))]
    weight[start[0]][start[1]] = 0

    # priority Queue for searching the children node according to their weight (increasing order)
    nodes_to_explore = []
    nodes_to_explore.append([start, heuristic_weight(start)])  # The start node weight consists only of heuristic weight

    # end search when goal found or no new node left to search
    while (not found) and (not len(nodes_to_explore) == 0):

        current_node = min(nodes_to_explore, key=lambda x: x[:][1])
        nodes_to_explore.remove(current_node)

        for inc in increment:
            i = inc[0]
            j = inc[1]

            if current_node[0] == goal:
                found = True
                break

            # Check if child node is out of grid or on obstacle, continue if so
            if current_node[0][0] + i < 0 or current_node[0][0] + i >= len(grid):
                continue
            if current_node[0][1] + j < 0 or current_node[0][1] + j >= len(grid[0]):
                continue
            if grid[current_node[0][0] + i][current_node[0][1] + j] == 1 or grid[current_node[0][0] + i][
                current_node[0][1] + j] == -1:
                continue
            if weight[current_node[0][0] + i][current_node[0][1] + j] > weight[current_node[0][0]][
                current_node[0][1]] + 1:
                weight[current_node[0][0] + i][current_node[0][1] + j] = weight[current_node[0][0]][
                                                                             current_node[0][1]] + 1
                parents[current_node[0][0] + i][current_node[0][1] + j] = current_node[0]
                grid[current_node[0][0] + i][current_node[0][1] + j] = -1
                nodes_to_explore.append([[current_node[0][0] + i, current_node[0][1] + j],
                                         heuristic_weight([current_node[0][0] + i, current_node[0][1] + j]) +
                                         weight[current_node[0][0] + i][current_node[0][1] + j] + 1])

        steps += 1

    if found:
        parent = goal
        while parent != start:
            path.append(parent)
            parent = parents[parent[0]][parent[1]]
        path.append(start)
        path.reverse()

        print(f"It takes {steps} steps to find a path using A*")
    else:
        print("No path found")

    return path, steps


# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples

    # Test all the functions
    testmod()
