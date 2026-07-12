import numpy as np
import yaml
import os
import matplotlib.pyplot as plt

class Graph:

    def __init__(self):

        self._vert_list = []
        self._edge_dict = {}
        self._adjacency_matrix_initialized = False

    def add_vertex(self, v):
        self._vert_list.append(v)

    def set_edge(self, v1, v2, e):
        self._edge_dict[(v1, v2)] = e

    def set_adjacency_matrix(self):
        if not self._adjacency_matrix_initialized:
            self._adjacency_matrix = np.zeros((len(self._vert_list), len(self._vert_list)))
            self._adjacency_matrix_initialized = True

        for i, v in enumerate(self._vert_list):
            for j, u in enumerate(self._vert_list):

                key = (u, v)

                if key in self._edge_dict:
                    self._adjacency_matrix[j, i] = self._edge_dict[key]

class ObstaclesGrid:
    def __init__(self, map_size):

        self.map = np.zeros(map_size, dtype=bool)
        self.map_size = map_size

    def is_edge_valid(self, edge_key, edge_val, lattice_cell_size, arc_primitives):

        pt1 = edge_key[0]
        pt2 = edge_key[1]

        if edge_val == 1:  # line
            pts = self.get_pts_from_line(pt1, pt2, lattice_cell_size)
        elif edge_val > 1:  # arc
            pts = self.get_pts_from_arc(pt1, pt2, lattice_cell_size, arc_primitives)
        else:
            pts = [(-99, -99)]

        for pt in pts:
            if not self.is_point_valid(pt):
                return False

        return True

    def get_pts_from_line(self, pt1, pt2, lattice_cell_size):

        pts = []
        dir_row = pt2[0] - pt1[0]
        dir_col = pt2[1] - pt1[1]

        for i in range(lattice_cell_size):
            row = pt1[0]*lattice_cell_size + i*dir_row
            col = pt1[1]*lattice_cell_size + i*dir_col
            pts.append((row, col))

        return pts

    def get_pts_from_arc(self, pt1, pt2, lattice_cell_size, arc_primitives):

        if pt1[2] == pt2[2]:
            pts = self.get_pts_from_line(pt1, pt2, lattice_cell_size)
        else:
            arc = arc_primitives[(pt1[2], pt2[2])]
            arc = np.array(pt1[:2]).reshape((2, 1)) * lattice_cell_size + arc

            pts = []
            for i in range(arc.shape[1]):
                pts.append((int(arc[0, i]), int(arc[1, i])))

        return pts

    def is_point_valid(self, point):

        if point[0] >= self.map_size[0] or point[1] >= self.map_size[1] or point[0] < 0 or point[1] <0:
            return False

        return not self.map[point[0], point[1]]

def write_result_to_yaml(result, filename):
    base_dir = os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', '..', '..', 'install', 'turtlebot3_navigation2', 'share', 'turtlebot3_navigation2', 'launch'))
    yaml_path = os.path.join(base_dir, filename)

    data = {
        'trajectory': [
            {
                'x': float(state.x),
                'y': float(state.y),
                'z': float(state.z) if hasattr(state, 'z') else 0.0
            }
            for state in result.states
        ]
    }
    with open(yaml_path, 'w') as yaml_file:
        yaml_file.truncate(0)
        yaml.dump(data, yaml_file, default_flow_style=False)

    print(f"Trajectory written to {yaml_path}")


def plot_map(obs, graph, arc_length):

    fig, ax = plt.subplots()
    obs_plot = 0.6*obs.map.astype(float)
    plt.imshow(obs_plot, vmin=0.0, vmax=1.0, cmap='Greys')#, cmap='gray', Greys, Oranges ,Greens, Reds, cividis
    plt.grid(True)

    return ax
