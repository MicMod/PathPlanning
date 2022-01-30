from algorytms import *

class RRTOD:
    def __init__(self, RRT_path_coords, RRT_path_nodes,  obs, cross_obs_step = 100):
        self.path_coords = RRT_path_coords
        self.path_nodes = RRT_path_nodes
        self.obs = obs
        self.cross_obs_step = cross_obs_step
        self.graph = Graph()
        self.obs_dim = 40
    
    def getDistance(self, c1, c2):
      (x1, y1) = c1
      (x2, y2) = c2
      px = (float(x1) - float(x2)) ** 2
      py = (float(y1) - float(y2)) ** 2
      return (px + py) ** (0.5)
    
    def crossObs(self, p1, p2):
        obs_ =  self.obs.copy()
        (x1, y1) = p1
        (x2, y2) = p2
        while (len(obs_) > 0):
            rectang = obs_.pop(0)
            for i in range(0, self.cross_obs_step + 1):
                u = i / self.cross_obs_step
                x = x1 * u + x2 * (1 - u)
                y = y1 * u + y2 * (1 - u)
                if rectang.collidepoint(x, y):
                    return True
        return False

    def getPath(self):
        for i in range(len(self.path_coords)-1):
            for j in range(i+1, len(self.path_coords)):
                if not self.crossObs(self.path_coords[i], self.path_coords[j]):
                    self.graph.add_edge(self.path_nodes[i], self.path_nodes[j], self.getDistance(self.path_coords[i], self.path_coords[j]))

        dijsktra_path_node_indx = dijsktra(self.graph, self.path_nodes[0], self.path_nodes[-1])
        dijsktra_path = []
        for node_indx in dijsktra_path_node_indx:
            dijsktra_path.append(self.path_coords[self.path_nodes.index(node_indx)])
        
        return dijsktra_path

