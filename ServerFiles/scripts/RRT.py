import random
import math

class RRT:
    def __init__(self, start, goal, map_dim, obs, d_max, d_cel, iter_max, f_l_c):
        (x, y) = start
        self.start = start
        self.goal = goal
        self.goal_flag = False
        self.map_h, self.map_w = map_dim
        self.x = []
        self.y = []
        self.parent = []
        # initialize the tree
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)
        # the obstacles
        self.obstacles = obs
        # path
        self.goal_state = None
        self.path = []
        self.opt_path = []
        # params
        self.iter_max = iter_max
        self.f_l_c = f_l_c
        self.d_step = d_max
        self.d_cel = d_cel

    def addNode(self, n, x, y):
        self.x.insert(n, x)
        self.y.append(y)
    
    def getNodeNums(self):
        return len(self.x)

    def removeNode(self, n):
        self.x.pop(n)
        self.y.pop(n)

    def addEdge(self, parent, child):
        self.parent.insert(child, parent)

    def removeEdge(self, n):
        self.parent.pop(n)

    def numberOfNodes(self):
        return len(self.x)

    def distance(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        px = (float(x1) - float(x2)) ** 2
        py = (float(y1) - float(y2)) ** 2
        return (px + py) ** (0.5)
    
    def distanceCor(self, c1, c2):
        (x1, y1) = c1
        (x2, y2) = c2
        px = (float(x1) - float(x2)) ** 2
        py = (float(y1) - float(y2)) ** 2
        return (px + py) ** (0.5)

    def sampleEnvir(self):
        x = int(random.uniform(0, self.map_w))
        y = int(random.uniform(0, self.map_h))
        return x, y

    def nearest(self, n):
        dmin = self.distance(0, n)
        nnear = 0
        for i in range(0, n):
            if self.distance(i, n) < dmin:
                dmin = self.distance(i, n)
                nnear = i
        return nnear

    def isFree(self):
        n = self.numberOfNodes() - 1
        (x, y) = (self.x[n], self.y[n])
        obs = self.obstacles.copy()
        while len(obs) > 0:
            rectang = obs.pop(0)
            if rectang.collidepoint(x, y):
                self.removeNode(n)
                return False
        return True

    def crossObstacle(self, x1, x2, y1, y2):
        obs = self.obstacles.copy()
        while (len(obs) > 0):
            rectang = obs.pop(0)
            for i in range(0, 101):
                u = i / 100
                x = x1 * u + x2 * (1 - u)
                y = y1 * u + y2 * (1 - u)
                if rectang.collidepoint(x, y):
                    return True
        return False

    def crossObsP(self, obs, p1, p2):
        (x1, y1) = p1
        (x2, y2) = p2
        while (len(obs) > 0):
            rectang = obs.pop(0)
            for i in range(0, 101):
                u = i / 100
                x = x1 * u + x2 * (1 - u)
                y = y1 * u + y2 * (1 - u)
                if rectang.collidepoint(x, y):
                    return True
        return False

    def shorterPath(self, path, obs):
        shorter_path = []
        num_cant_cross_obs = 0
        i = 0
        shorter_path.append(path[i])
        path_len = len(path)
        while i < (path_len - 2):
            if (self.distanceCor(path[i], path[i + 1]) +  self.distanceCor(path[i+1] + path[i+2]) > self.distanceCor(path[i], path[i+2])):
                if (self.crossObsP(obs, path[i], path[i+2])):
                    shorter_path.append(path[i+2])
                    i += 1
                else:
                    num_cant_cross_obs += 1
                    shorter_path.append(path[i+2])
                    i += 1
            else:
                shorter_path.append(path[i+1])
            i += 1
        if ((shorter_path[-1][0] != path[-1][0]) and (shorter_path[-1][1] != path[-1][1])):
            shorter_path.append(path[-1])
        return shorter_path, num_cant_cross_obs

    def crossObs(self, obs, x1, x2, y1, y2):
        while (len(obs) > 0):
            rectang = obs.pop(0)
            for i in range(0, 101):
                u = i / 100
                x = x1 * u + x2 * (1 - u)
                y = y1 * u + y2 * (1 - u)
                if rectang.collidepoint(x, y):
                    return True
        return False

    def connect(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        if self.crossObstacle(x1, x2, y1, y2):
            self.removeNode(n2)
            return False
        else:
            self.addEdge(n1, n2)
            return True

    def step(self, nnear, nrand):
        d = self.distance(nnear, nrand)
        if d > self.d_step:
            u = self.d_step / d
            (xnear, ynear) = (self.x[nnear], self.y[nnear])
            (xrand, yrand) = (self.x[nrand], self.y[nrand])
            (px, py) = (xrand - xnear, yrand - ynear)
            theta = math.atan2(py, px)
            (x, y) = (int(xnear + self.d_step * math.cos(theta)),
                      int(ynear + self.d_step * math.sin(theta)))
            self.removeNode(nrand)
            if abs(x - self.goal[0]) <= self.d_step and abs(y - self.goal[1]) <= self.d_step:
                self.addNode(nrand, self.goal[0], self.goal[1])
                self.goal_state = nrand
                self.goal_flag = True
            else:
                self.addNode(nrand, x, y)

    def bias(self, ngoal):
        n = self.numberOfNodes()
        self.addNode(n, ngoal[0], ngoal[1])
        nnear = self.nearest(n)
        self.step(nnear, n)
        self.connect(nnear, n)
        return self.x, self.y, self.parent

    def expand(self):
        n = self.numberOfNodes()
        x, y = self.sampleEnvir()
        self.addNode(n, x, y)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest, n)
            self.connect(xnearest, n)
        return self.x, self.y, self.parent

    def pathToGoal(self):
        if self.goal_flag:
            self.path = []
            self.path.append(self.goal_state)
            print(self.goal_state)
            print(len(self.parent))
            newpos = self.parent[self.goal_state]
            while (newpos != 0):
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
        return self.goal_flag

    def getPathCoords(self):
        pathCoords = []
        for node in self.path:
            x, y = (self.x[node], self.y[node])
            pathCoords.append((x, y))
        return pathCoords

    def getPath(self):
        iteration = 0
        while (not self.pathToGoal()):
            if iteration % self.f_l_c == 0:
                X, Y, Parent = self.bias(self.goal)
            else:
                X, Y, Parent = self.expand()
            iteration += 1

        return self.getPathCoords(), self.path

    def cost(self, n):
        ninit = 0
        n = n
        parent = self.parent[n]
        c = 0
        while n is not ninit:
            c = c + self.distance(n, parent)
            n = parent
            if n is not ninit:
                parent = self.parent[n]
        return c

    def waypointsToPath(self):
        oldpath = self.getPathCoords()
        path = []
        for i in range(0, len(self.path) - 1):
            if i >= len(self.path):
                break
            x1, y1 = oldpath[i]
            x2, y2 = oldpath[i + 1]
            for i in range(0, 5):
                u = i / 5
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                path.append((x, y))

        return path

        
