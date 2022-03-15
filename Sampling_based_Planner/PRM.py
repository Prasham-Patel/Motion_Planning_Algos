# Standard Algorithm Implementation
# Sampling-based Algorithms PRM

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
from math import sin, cos, atan2
import copy
from scipy import spatial


# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.samples = []                     # list of sampled points
        self.graph = nx.Graph()               # constructed graph
        self.path = []                        # list of nodes of the found path


    def check_collision(self, p1, p2):

        '''Check if the path between two points collide with obstacles

        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            True if there are obstacles between two points
        '''
        if self.dis(p1, p2) == 0:
            return True

        inc = 1/self.dis(p1, p2)
        i = copy.deepcopy(inc)

        while i < 1:
            row = p1[0]*i + (p2[0]*(1-i))
            col = p1[1]*i + (p2[1]*(1-i))
            i = i+inc
            if self.map_array[int(row), int(col)] == 1:
                continue
            else:
                return False
        return True


    def dis(self, point1, point2):
        '''Calculate the euclidean distance between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            euclidean distance between two points
        '''

        dist = ((point1[1]-point2[1])**2 + (point1[0]-point2[0])**2)**0.5

        return dist


    def uniform_sample(self, n_pts):
        '''Use uniform sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''

        # Initialize graph

        self.graph.clear()
        sample_to_search = []
        distribution_len = int(self.size_col/n_pts**0.5)
        print(distribution_len)
        self.samples.append((0, 0))
        sample_to_search.append([0, 0])
        i = 0
        while i <= self.size_row:
            j = 0
            while j <= self.size_col:
                new_sample = (i, j)
                if self.map_array[new_sample[0], new_sample[1]] == 1:
                    self.samples.append(new_sample)
                j += distribution_len
            i += distribution_len


    def random_sample(self, n_pts):
        '''Use random sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''

        # Initialize graph
        self.graph.clear()
        index = 0
        while index < n_pts:
            new_sample = [np.random.randint(0, self.size_row-1), np.random.randint(0, self.size_col-1)]
            if (self.map_array[new_sample[0], new_sample[1]] == 0):
                continue
            else:
                self.samples.append(new_sample)
                index += 1


    def gaussian_sample(self, n_pts):
        '''Use gaussian sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()
        index = 0
        while index < n_pts:
            new_sample_1 = [np.random.randint(0, self.size_row-1), np.random.randint(0, self.size_col-1)]
            new_sample_2 = np.random.normal(new_sample_1, scale = 7)
            if new_sample_2[0] > self.size_row or new_sample_2[1] > self.size_col:
                continue
            new_sample_2 = [int(new_sample_2[0]), int(new_sample_2[1])]
            if self.map_array[new_sample_1[0], new_sample_1[1]] == 0 and self.map_array[new_sample_2[0], new_sample_2[1]] == 1:
                self.samples.append(new_sample_2)
                index += 1
            elif self.map_array[new_sample_1[0], new_sample_1[1]] == 1 and self.map_array[new_sample_2[0], new_sample_2[1]] == 0:
                self.samples.append(new_sample_1)
                index += 1



        ### YOUR CODE HERE ###

    def bridge_sample(self, n_pts):
        '''Use bridge sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()
        index = 0
        while index < n_pts:
            new_sample_1 = [np.random.randint(0, self.size_row - 1), np.random.randint(0, self.size_col - 1)]
            new_sample_2 = np.random.normal(new_sample_1, scale=7)
            if new_sample_2[0] > self.size_row or new_sample_2[1] > self.size_col:
                continue
            new_sample_2 = [int(new_sample_2[0]), int(new_sample_2[1])]
            if self.map_array[new_sample_1[0], new_sample_1[1]] == 0 and self.map_array[new_sample_1[0], new_sample_1[1]] == 0:
                mid_point = [int((new_sample_1[0] + new_sample_2[0])/2), int((new_sample_1[1] + new_sample_2[1])/2)]
                if self.map_array[mid_point[0], mid_point[1]] == 1:
                    # print(mid_point)
                    self.samples.append(mid_point)
                    index += 1
                    print(index)

        ### YOUR CODE HERE ###

    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict(zip(range(len(self.samples)), node_pos))
        pos['start'] = (self.samples[-2][1], self.samples[-2][0])
        pos['goal'] = (self.samples[-1][1], self.samples[-1][0])

        # draw constructed graph
        nx.draw(self.graph, pos, node_size=3, node_color='y', edge_color='y', ax=ax)

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=self.path, node_size=8, node_color='b')
            nx.draw_networkx_edges(self.graph, pos=pos, edgelist=final_path_edge, width=2, edge_color='b')

        # draw start and goal
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['start'], node_size=12, node_color='g')
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['goal'], node_size=12, node_color='r')

        # show image
        plt.axis('on')
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()

    def sample(self, n_pts=1000, sampling_method="uniform"):
        '''Construct a graph for PRM
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        '''
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)
        elif sampling_method == "random":
            self.random_sample(n_pts)
        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)

        ### YOUR CODE HERE ###
        self.kdtree = spatial.KDTree(self.samples)
        index = 0
        pairs = []
        for sample in self.samples:
            NN, NN_ID = self.kdtree.query(sample, k = 8)    # getting eight nearest neighbors of sample
            NN_ID = NN_ID[1:]   # the first element is the sample itself
            for ID in NN_ID:
                if self.check_collision(self.samples[ID], self.samples[index]):
                    pairs.append((index, ID, self.dis(self.samples[ID], self.samples[index])))
            index += 1


        # print(pairs)

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # Store them as
        # pairs = [(p_id0, p_id1, weight_01), (p_id0, p_id2, weight_02),
        #          (p_id1, p_id2, weight_12) ...]
        # pairs = []

        # Use sampled points and pairs of points to build a graph.
        # To add nodes to the graph, use
        # self.graph.add_nodes_from([p_id0, p_id1, p_id2 ...])
        # To add weighted edges to the graph, use
        # self.graph.add_weighted_edges_from([(p_id0, p_id1, weight_01),
        #                                     (p_id0, p_id2, weight_02),
        #                                     (p_id1, p_id2, weight_12) ...])
        # 'p_id' here is an integer, representing the order of
        # current point in self.samples
        # For example, for self.samples = [(1, 2), (3, 4), (5, 6)],
        # p_id for (1, 2) is 0 and p_id for (3, 4) is 1.
        # self.graph.add_nodes_from(range(0, len(self.samples)))
        self.graph.add_weighted_edges_from(pairs)

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print("The constructed graph has %d nodes and %d edges" % (n_nodes, n_edges))

    def search(self, start, goal):
        '''Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal node, edges of them and their nearest neighbors
        to graph for self.graph to search for a path.
        '''
        # Clear previous path
        self.path = []

        # Temporarily add start and goal to the graph
        self.samples.append(start)
        self.samples.append(goal)
        # start and goal id will be 'start' and 'goal' instead of some integer
        self.graph.add_nodes_from(['start', 'goal'])

        ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # You could store them as
        # start_pairs = [(start_id, p_id0, weight_s0), (start_id, p_id1, weight_s1),
        #                (start_id, p_id2, weight_s2) ...]
        start_pairs = []
        goal_pairs = []

        KNN_start_dist, KNN_start_ID = self.kdtree.query(start, k=10)
        KNN_goal_dist, KNN_goal_ID = self.kdtree.query(goal, k=10)

        for ID in KNN_start_ID:
            start_pairs.append(('start', ID, self.dis(start, self.samples[ID])))


        for ID in KNN_goal_ID:
            goal_pairs.append(('goal', ID, self.dis(goal, self.samples[ID])))

        # Add the edge to graph
        self.graph.add_weighted_edges_from(start_pairs)
        self.graph.add_weighted_edges_from(goal_pairs)

        # Seach using Dijkstra
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(self.graph, 'start', 'goal')
            path_length = nx.algorithms.shortest_paths.weighted.dijkstra_path_length(self.graph, 'start', 'goal')
            print("The path length is %.2f" % path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")

        # Draw result
        self.draw_map()

        # Remove start and goal node and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(['start', 'goal'])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)
