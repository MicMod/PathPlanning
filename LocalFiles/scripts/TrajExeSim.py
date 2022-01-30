from Map import Map, MapDraw
from FileShareAccess import FileShareAccess
import pickle
import os
import yaml
from pathlib import Path
import time
import pygame

class TrajExeSim:
  def __init__(self):
    # setting paths
    self.root_path = self.getRootPath()
    self.file_share_param_path = self.loadYAML(str(self.root_path + "config/file_share_params.yaml"))
    self.map_param_path = str(self.root_path + "config/map_params.yaml")
    self.init_pos_param_path = str(self.root_path + "local_storage/init_positions.yaml")
    self.local_storage_path_obs = str(self.root_path + self.file_share_param_path["local_storage_path_obs"])
    self.local_storage_path_pos = str(self.root_path + self.file_share_param_path["local_storage_path_pos"])
    self.local_storage_path_traj = str(self.root_path + self.file_share_param_path["local_storage_path_traj"])
    self.virtual_storage_path_obs = str(self.file_share_param_path["virtual_storage_path_obs"])
    self.virtual_storage_path_pos = str(self.file_share_param_path["virtual_storage_path_pos"])
    self.virtual_storage_path_traj = str(self.file_share_param_path["virtual_storage_path_traj"])
    # map param
    self.map_param = self.loadYAML(self.map_param_path)
    self.map_dim = (self.map_param["height"], self.map_param["width"])
    self.obs_dim = self.map_param["obs_dim"]
    self.obs_nums = self.map_param["obs_nums"]
    # position param
    self.init_pos_param = self.loadYAML(str(self.root_path + "local_storage/init_positions.yaml"))
    self.start_pos = (self.init_pos_param["start_x"], self.init_pos_param["start_y"])
    self.goal_pos = (self.init_pos_param["goal_x"], self.init_pos_param["goal_y"])
    self.robot_pos = self.start_pos
    self.robot_pos_idx = 0
    # traj sim param
    self.map = Map(self.start_pos, self.goal_pos, self.map_dim, self.obs_dim, self.obs_nums)
    self.obstacles = []
    self.path = []
    # draw map
    self.map_img = MapDraw(self.start_pos, self.goal_pos, self.map_dim)
    pygame.init()
    # file share param
    self.file_share_access = FileShareAccess()
    self.file_indx = 0
    self.file_share_timeout = 10
    self.file_share_sleep_time = 0.1

  def getRootPath(self):
    path_file_str = os.path.dirname(os.path.abspath(__file__))
    path_file = Path(path_file_str)
    root_path = str(path_file.parent.absolute()) + "/"
    return root_path

  def loadPickle(self, file_path):
    with open(file_path, "rb") as fp1:
      data = pickle.load(fp1)
    return data 

  def savePickle(self, file_path, data):
    with open(file_path, "wb") as fp1:
      pickle.dump(data, fp1)

  def loadYAML(self, file_path):
    with open(file_path, 'r') as yamlfile:
      return yaml.load(yamlfile, Loader=yaml.FullLoader)

  def savePosYAML(self, file_path):
    pos_data = dict(robot_x = self.robot_pos[0],
                    robot_y = self.robot_pos[1],
                    goal_x = self.goal_pos[0],
                    goal_y = self.goal_pos[1])
    with open(file_path, 'w') as yamlfile:
      return yaml.dump(pos_data, yamlfile)

  def robot_move(self):
    self.robot_pos = self.path[self.robot_pos_idx]

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

  def isCollision(self, obs, path):
    if (self.robot_pos_idx < len(path)):
      return self.crossObs(obs, self.robot_pos[0], self.robot_pos[1], path[self.robot_pos_idx + 1][0], path[self.robot_pos_idx + 1][1])
    else:
      False

  def reversePath(self, path):
    new_path = []
    for node in reversed(path):
      new_path.append(node)
    return new_path

  def isGoalReach(self):
    return (self.robot_pos[0] == self.goal_pos[0] and self.robot_pos[1] == self.goal_pos[1])

  def __call__(self):
    # create new obstacles
    obstacles = self.map.makeObstacles()
    if(not self.path or self.isCollision(obstacles, self.path)):
      # files paths
      local_obs_file_path = f"{self.local_storage_path_obs}/obstacles_{self.file_indx}.txt"
      local_pos_file_path = f"{self.local_storage_path_pos}/positions_{self.file_indx}.yaml"
      local_traj_file_path = f"{self.local_storage_path_traj}/trajectory_{self.file_indx}.txt"
      virtual_obs_file_path = f"{self.virtual_storage_path_obs}obstacles_{self.file_indx}.txt"
      virtual_pos_file_path = f"{self.virtual_storage_path_pos}positions_{self.file_indx}.yaml"
      virtual_traj_file_path = f"{self.virtual_storage_path_traj}trajectory_{self.file_indx}.txt"
      virtual_traj_file_name = f"trajectory_{self.file_indx}.txt"
      # saving data to local folders
      self.savePickle(local_obs_file_path, obstacles)
      self.savePosYAML(local_pos_file_path)
      # upload data to virtual machine
      self.file_share_access.upload(local_obs_file_path, virtual_obs_file_path)
      self.file_share_access.upload(local_pos_file_path, virtual_pos_file_path)
      # check if trajectory calculated
      start_time = time.time()
      while(not self.file_share_access.checkFile(self.virtual_storage_path_traj, virtual_traj_file_name)):
        now_time = time.time()
        if(now_time - start_time > self.file_share_timeout):
          print("Cannot get trajectory from virtual server")
          return False
        time.sleep(self.file_share_sleep_time)
      # download trajectory file from virtual machine
      self.file_share_access.download(local_traj_file_path, virtual_traj_file_path)
      # update trajectory
      self.path =  self.loadPickle(local_traj_file_path)
      self.path = self.reversePath(self.path)
      # update new file indx
      self.file_indx += 1
      # update next robot position on path
      self.robot_pos_idx = 1
    else:
      # update next robot position on path
      self.robot_pos_idx += 1
  
    # update robot position
    self.robot_move()

    # display
    self.map_img.drawStartEnd()
    self.map_img.drawPath(self.path, (0, 150, 0))
    self.map_img.drawObs(obstacles)
    self.map_img.drawRobotPos(self.robot_pos)
    
    pygame.display.update()
    time.sleep(1)

      
def main():
  sim = TrajExeSim()
  while (not sim.isGoalReach()):
    sim()
  
if __name__ == '__main__':
  main()