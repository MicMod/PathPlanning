import pygame
import random

class Map:
    def __init__(self, start, goal, map_dim, obs_dim, obs_nums):
        self.start = start
        self.goal = goal
        self.map_dim = map_dim
        self.map_h, self.map_w = self.map_dim
        
        self.obstacles = []
        self.obs_dim = obs_dim
        self.obs_nums = obs_nums
        self.d_step = 30
         # border
        self.d_step = 30
        self.border_thickness = self.d_step
        # create obstacles 
        self.makeObs()
        self.makeBorders()

    def makeRandomRect(self):
        upper_corner_x = int(random.uniform(0, self.map_w - self.obs_dim))
        upper_corner_y = int(random.uniform(0, self.map_h - self.obs_dim))

        return (upper_corner_x, upper_corner_y)

    def makeObs(self):
        obs = []
        for i in range(0, self.obs_nums):
            rectang = None
            start_goal_col = True
            while start_goal_col:
                upper = self.makeRandomRect()
                rectang = pygame.Rect(upper, (self.obs_dim, self.obs_dim))
                if rectang.collidepoint(self.start) or rectang.collidepoint(self.goal):
                    start_goal_col = True
                else:
                    start_goal_col = False
            self.obstacles.append(rectang)
        return self.obstacles


    def makeBorders(self):
        border = []
        up_border = pygame.Rect(0, 0, self.map_w, self.border_thickness)
        border.append(up_border)
        down_border = pygame.Rect(0, self.map_h - self.border_thickness, self.map_w, self.border_thickness)
        border.append(down_border)
        left_border = pygame.Rect(0, 0, self.border_thickness, self.map_h)
        border.append(left_border)
        right_border = pygame.Rect(self.map_w - self.border_thickness, 0, self.border_thickness, self.map_h)
        border.append(right_border)
        left_wall = pygame.Rect(11 * self.d_step, self.wall_gap, self.border_thickness, self.map_h - self.wall_gap)
        border.append(left_wall)
        right_wall = pygame.Rect(22 * self.d_step, 0, self.border_thickness, self.map_h - self.wall_gap)
        border.append(right_wall)
        for b in border:
          self.obstacles.append(b)
        return border

    def getObstacles(self):
        return self.obstacles

class MapDraw:
    def __init__(self, start, goal, map_dim, obs, obs_dim, obs_nums):
        self.start = start
        self.goal = goal
        self.map_dim = map_dim
        self.map_h, self.map_w = self.map_dim
        # window settings
        self.map_window_name = 'Map'
        pygame.display.set_caption(self.map_window_name)
        self.map = pygame.display.set_mode((self.map_w, self.map_h))
        self.map.fill((255, 255, 255))
        self.node_rad = 2
        self.node_thickness = 0
        self.edge_thickness = 1
        self.obstacles = obs
        self.obs_dim = obs_dim
        self.obs_nums = obs_nums
        self.d_step = 30
        # Colors
        self.grey = (70, 70, 70)
        self.blue = (0, 0, 255)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.white = (255, 255, 255)
        self.black = (0, 0, 0)

    def drawMap(self):
        pygame.draw.circle(self.map, self.green, self.start, self.node_rad + 5, 0)
        pygame.draw.circle(self.map, self.red, self.goal, self.node_rad + self.d_step / 2, 1)
        pygame.draw.circle(self.map, self.red, self.goal, self.node_rad + 5, 0)
        self.drawObs(self.obstacles)
        
    def drawPath(self, path, color = (0, 0, 0)):
        for idx in range(len(path)-1):
            pygame.draw.circle(self.map, color, path[idx], 3, 0)
            pygame.draw.line(self.map, color, path[idx], path[idx+1], 2)
        #draw last point
        pygame.draw.circle(self.map, color, path[-1], 3, 0)

    def drawObs(self, obstacles):
        obstaclesList = obstacles.copy()
        while (len(obstaclesList) > 0):
            obstacle = obstaclesList.pop(0)
            pygame.draw.rect(self.map, self.black, obstacle)

    def drawGrid(self):
        n_h = int(self.map_h / self.d_step)
        n_w = int(self.map_w / self.d_step)
        for i in range(n_h):
            pygame.draw.line(self.map, self.grey, (0, i * self.d_step), (self.map_w, i * self.d_step))
        for j in range(n_w):
            pygame.draw.line(self.map, self.grey, (j * self.d_step, 0), (j * self.d_step, self.map_h))
    
    def drawGridDots(self):
        n_h = int(self.map_h / self.d_step)
        n_w = int(self.map_w / self.d_step)
        for i in range(n_h):
            for j in range(n_w):
                pygame.draw.circle(self.map, self.grey, (j * self.d_step + 15, i * self.d_step + 15), 1)

    def drawFreeGridDots(self, obs):
        d_step =  30
        n_h = int(600 / d_step)
        n_w = int(1020/ d_step)
        dots_grid  = [[0]*n_w]*n_h
        for i in range(n_h):
            for j in range(n_w):
                if self.isCellFree((j * d_step + 15, i * d_step + 15), d_step, obs):
                    dots_grid[i][j] = 1
                    pygame.draw.circle(self.map, self.green, (j * self.d_step + 15, i * self.d_step + 15), 2)
                else:
                    dots_grid[i][j] = float('inf')

    def addText(self, text, pos):
        self.map.blit(text, dest=pos)

    def isCellFree(self, cell_center, cell_dim, obs):
        (x, y) = cell_center
        obs_ = obs.copy()
        d_to_corner = cell_dim / 2 - 1 # -1 to aviod disabling cells close to boarders
        while len(obs_) > 0:
            rectang = obs_.pop(0)
            # check upper left corner
            if rectang.collidepoint(x - d_to_corner, y - d_to_corner):
                return False
            # check upper right corner
            if rectang.collidepoint(x + d_to_corner, y - d_to_corner):
                return False
            # check down left corner
            if rectang.collidepoint(x - d_to_corner, y + d_to_corner):
                return False
            # check down right corner
            if rectang.collidepoint(x + d_to_corner, y + d_to_corner):
                return False
        return True
