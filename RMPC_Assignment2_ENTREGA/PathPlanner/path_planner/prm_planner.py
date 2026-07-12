import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
from path_planner.utils import ObstaclesGrid

class Node:
    def __init__(self, x, y):
        """
        Represents a node in the PRM roadmap.

        Args:
            x (float): X-coordinate of the node.
            y (float): Y-coordinate of the node.
        """
        self.x = x
        self.y = y

class PRMPlanner:
    def __init__(self, start, goal, map_size, obstacles, num_samples=200, k_neighbors=10, step_size=5):
        """
        Initializes the PRM planner.

        Args:
            start (tuple): (x, y) coordinates of the start position.
            goal (tuple): (x, y) coordinates of the goal position.
            map_size (tuple): (width, height) of the environment.
            obstacles (ObstaclesGrid): Object that stores obstacle information.
            num_samples (int): Number of random samples for roadmap construction.
            k_neighbors (int): Number of nearest neighbors to connect in the roadmap.
            step_size (float): Step size used for collision checking.
        """
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.map_size = map_size
        self.obstacles = obstacles
        self.num_samples = num_samples
        self.k_neighbors = k_neighbors
        self.step_size = step_size
        self.roadmap = []
        self.edges = {}

    def construct_roadmap(self):
        """
        Constructs the probabilistic roadmap by sampling nodes and connecting them.

        Returns:
        None
        """
        self.roadmap.append(self.start)
        self.roadmap.append(self.goal)

        for _ in range(self.num_samples):
            p = self.sample_free_point()
            if p is not None:
                self.roadmap.append(p)

        # connect each node to k nearest neighbors
        for node in self.roadmap:
            nearest = self.find_k_nearest(node, self.k_neighbors)
            for nb in nearest:
                if not self.is_colliding(node, nb):
                    if node not in self.edges:
                        self.edges[node] = []
                    if nb not in self.edges:
                        self.edges[nb] = []
                    if nb not in self.edges[node]:
                        self.edges[node].append(nb)
                    if node not in self.edges[nb]:
                        self.edges[nb].append(node)

    def sample_free_point(self):
        """
        Samples a random collision-free point in the environment.

        Returns:
        Node: A randomly sampled node.
        """
        for _ in range(100):
            x = np.random.randint(0, self.map_size[0])
            y = np.random.randint(0, self.map_size[1])
            if not self.obstacles.map[x, y]:
                return Node(x, y)
        return None

    def find_k_nearest(self, node, k):
        """
        Finds the k-nearest neighbors of a node in the roadmap.

        Args:
            node (Node): The node for which neighbors are searched.
            k (int): The number of nearest neighbors to find.

        Returns:
            list: A list of k-nearest neighbor nodes.
        """
        pts = np.array([[n.x, n.y] for n in self.roadmap])
        tree = KDTree(pts)
        k_actual = min(k + 1, len(self.roadmap))
        _, idxs = tree.query([node.x, node.y], k=k_actual)

        result = []
        for i in idxs:
            nb = self.roadmap[i]
            if nb.x == node.x and nb.y == node.y:
                continue
            result.append(nb)
        return result[:k]

    def is_colliding(self, node1, node2):
        """
        Checks if the path between two nodes collides with an obstacle.

        Args:
            node1 (Node): The first node.
            node2 (Node): The second node.

        Returns:
            bool: True if there is a collision, False otherwise.
        """
        d = np.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
        steps = int(d / self.step_size) + 1
        for i in range(steps + 1):
            t = i / max(steps, 1)
            x = int(node1.x + t * (node2.x - node1.x))
            y = int(node1.y + t * (node2.y - node1.y))
            if x < 0 or x >= self.map_size[0] or y < 0 or y >= self.map_size[1]:
                return True
            if self.obstacles.map[x, y]:
                return True
        return False

    def plan(self):
        """
        Plans a path from start to goal using the constructed roadmap.

        Returns:
        list: A list of (x, y) tuples representing the path.
        """
        from queue import PriorityQueue

        dist = {}
        prev = {}
        visited = set()
        pq = PriorityQueue()

        sk = (self.start.x, self.start.y)
        gk = (self.goal.x, self.goal.y)

        dist[sk] = 0
        pq.put((0, sk))

        # lookup table
        lookup = {}
        for n in self.roadmap:
            lookup[(n.x, n.y)] = n

        while not pq.empty():
            c, curr = pq.get()
            if curr in visited:
                continue
            visited.add(curr)

            if curr == gk:
                path = []
                k = gk
                while k in prev:
                    path.append(k)
                    k = prev[k]
                path.append(sk)
                path.reverse()
                return path

            node = lookup.get(curr)
            if node is None or node not in self.edges:
                continue

            for nb in self.edges[node]:
                nbk = (nb.x, nb.y)
                if nbk in visited:
                    continue
                w = np.sqrt((node.x - nb.x)**2 + (node.y - nb.y)**2)
                new_c = dist[curr] + w
                if nbk not in dist or new_c < dist[nbk]:
                    dist[nbk] = new_c
                    prev[nbk] = curr
                    pq.put((new_c, nbk))

        print("PRM: no path found")
        return None
