# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*

import matplotlib.pyplot as plt
import numpy as np
import copy
from math import inf


# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost


# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.start = Node(start[0], start[1]) # start node
        self.goal = Node(goal[0], goal[1])    # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag
        

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    
    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        point1 = [node1.row, node1.col]
        point2 = [node2.row, node2.col]
        dist = ((point1[1] - point2[1]) ** 2 + (point1[0] - point2[0]) ** 2) ** 0.5
        return dist

    
    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''

        point1 = [node1.row, node1.col]
        point2 = [node2.row, node2.col]
        dist = self.dis(node1, node2)
        if dist == 0:   # for same points no need to do collision check
            return True
        inc = 1 / dist  # dividing the length between two point into section of len ~= 1
        i = copy.deepcopy(inc)
        while i < 1:
            row = point1[0] * i + (point2[0] * (1 - i))
            col = point1[1] * i + (point2[1] * (1 - i))
            i = i + inc
            if self.map_array[int(row), int(col)] == 1:
                continue
            else:
                return False
        return True


    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        choose_goal = np.random.choice(a = [True, False], p = [goal_bias, 1-goal_bias])
        if choose_goal:
            return self.goal
        else:
            while True:
                new_sample = [np.random.randint(0, self.size_row-1), np.random.randint(0, self.size_col-1)]
                if self.map_array[new_sample[0], new_sample[1]] == 1:
                    new_node = Node(new_sample[0], new_sample[1])
                    return new_node

    
    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point (Node class)

        return:
            the nearest node
        '''

        dist = inf
        Nearest_node = None
        for n in self.vertices:
            node_dist = self.dis(n, point)
            if (dist > node_dist):
                dist = node_dist
                Nearest_node = n

        return Nearest_node, dist


    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of index of neighbors that are within the neighbor distance
        '''

        neighbors = []
        for index in range(len(self.vertices)):
            n = self.vertices[index]
            if self.dis(n, new_node) <= neighbor_size:
                neighbors.append(index)
        return neighbors

    def Node_extend(self, Node1, Node2, step_size = 10):
        current_node = Node1
        Total_dist = self.dis(Node1, Node2)
        if Total_dist == 0:
            return None
        inc = 1/ Total_dist
        i = copy.deepcopy(inc)
        while self.dis(current_node, Node2) > step_size:
            row = int(Node1.row * i + (Node2.row * (1 - i)))
            col = int(Node1.col * i + (Node2.col * (1 - i)))
            if self.map_array[row][col] == 1:
                new_node = Node(row, col)
                new_node.parent = current_node
                new_node.cost = current_node.cost +  step_size
                self.vertices.append(new_node)
                current_node = new_node
            i = i + inc
        Node2.parent = current_node
        Node2.cost = self.dis(Node2, current_node)
        self.vertices.append(Node2)

    def trim_dis(self, Node1, Node2, step_size = 10):
        '''Generate a new node within the step size defined.
                arguments:
                    Node1 - a parent node
                    Node2 - new node generated
                    step_size - max length/cost of the edge allowed

                return:
                     boolean - True if trim was successful
                '''

        Total_dist = self.dis(Node1, Node2)
        # if distance is less than step size no need to trim
        if Total_dist <= step_size and self.check_collision(Node1, Node2):
            Node2.parent = Node1
            Node2.cost = Node1.cost + self.dis(Node1, Node2)
            return True, Node2
        i = step_size/Total_dist    # if 'i' is small, new point more near to node 1
        row = int(Node2.row * i + (Node1.row * (1 - i)))
        col = int(Node2.col * i + (Node1.col * (1 - i)))
        new_node = Node(row, col)
        if self.map_array[row][col] == 1 and self.check_collision(Node1, new_node):
            new_node.parent = Node1
            new_node.cost = Node1.cost + self.dis(new_node, Node1)
            return True, new_node
        else:
            return False, None

    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of index of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        best_neigbor = None
        best_index = None

        # new node connected to node with lowest cost
        for index in neighbors:
            n = self.vertices[index]
            dist = self.dis(n, new_node)
            if new_node.cost > n.cost + dist and self.check_collision(n, new_node):
                new_node.parent = self.vertices[index]
                new_node.cost = n.cost + dist
                best_neigbor = n
                best_index = index
        self.vertices.append(new_node)

        # if rewire other nodes to the best node found if cost decreases
        if not best_index == None:
            for index in neighbors:
                n = self.vertices[index]
                dist = self.dis(n, best_neigbor)
                if not best_index==index and best_neigbor.cost + dist < n.cost and self.check_collision(n, best_neigbor):
                    self.vertices[index].parent = best_neigbor
                    self.vertices[index].cost = best_neigbor.cost + dist

    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
        
        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col and cur.row != self.start.row:
                plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')
        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')

        # show image
        plt.show()


    def RRT(self, n_pts=1000):
        '''RRT main search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        In each step, extend a new node if possible, and check if reached the goal
        '''
        # Remove previous result
        self.init_map()

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, check if reach the neighbor region of the goal.

        goal_bias = 0.1   # must be from 0 to 1
        step_size = 10    # optimal value from 5 to 15
        while not self.found and len(self.vertices) < n_pts:
            new_node = self.get_new_point(goal_bias)
            if new_node.row >= self.size_row or new_node.col >= self.size_col:      # new node must be inside the map
                continue
            Nearest_neighbor, _ = self.get_nearest_node(new_node)
            trimed, _new = self.trim_dis(Nearest_neighbor, new_node, step_size)

            if trimed:
                self.vertices.append(_new)
                # check if goal is near the new node, if so update goal cost
                if self.check_collision(self.goal, self.vertices[-1]) and self.dis(self.goal, self.vertices[-1]) < step_size:
                    new_node = self.vertices[-1]
                    self.goal.parent = new_node
                    self.goal.cost = new_node.cost + self.dis(self.goal, new_node)
                    self.vertices.append(self.goal)
                    self.found = True
                else:
                    continue

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")
        
        # Draw result
        self.draw_map()


    def RRT_star(self, n_pts=1000, neighbor_size=20):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, rewire the node and its neighbors,
        # and check if reach the neighbor region of the goal if the path is not found.

        goal_bias = 0.2  # must be from 0 to 1
        step_size = 7   # optimal value from 5 to 15
        self.goal.cost = inf    # to update cost latter
        while len(self.vertices) <= n_pts:
            new_node = self.get_new_point(goal_bias)
            if new_node.row >= self.size_row or new_node.col >= self.size_col:  # new node must be inside the map
                continue
            Nearest_neighbor, _ = self.get_nearest_node(new_node)
            trimed, _new = self.trim_dis(Nearest_neighbor, new_node, step_size)

            # the new node can not be the goal node | occurs when there are nodes near goal
            if _new == self.goal:
                continue

            if trimed:
                neighbors = self.get_neighbors(_new, neighbor_size = neighbor_size)     # get neighbors to rewire
                self.rewire(_new, neighbors)

                # check if goal is near the new node, if so update goal cost
                if self.dis(self.goal, self.vertices[-1]) + self.vertices[-1].cost < self.goal.cost and self.check_collision(self.goal, self.vertices[-1]) and self.dis(self.goal, self.vertices[-1]) < step_size :
                    self.goal.parent = self.vertices[-1]
                    self.goal.cost = self.vertices[-1].cost + self.dis(self.goal, self.vertices[-1])
                    self.found = True
                else:
                    continue

        # Output
        if self.found:
            self.vertices.append(self.goal)
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")

        # Draw result
        self.draw_map()
