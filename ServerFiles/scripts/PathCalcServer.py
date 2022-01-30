from RRT import RRT
from RRTOD import RRTOD
import pickle
import os
import yaml
from pathlib import Path
import time


class PathCalcServer:
  def __init__(self):
    self.root_path = self.getRootPath()
    self.map_param = self.loadYAML(self.root_path + "config/map_params.yaml")
    self.algo_param = self.loadYAML(self.root_path + "config/algo_params.yaml")
    self.file_param = self.loadYAML(self.root_path + "config/file_params.yaml")
    self.virtual_storage_path_obs = str(self.file_param["virtual_storage_path_obs"])
    self.virtual_storage_path_pos = str(self.file_param["virtual_storage_path_pos"])
    self.virtual_storage_path_traj = str(self.file_param["virtual_storage_path_traj"])
    
    self.file_indx = 0
    self.map_dim = (self.map_param["height"], self.map_param["width"])

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

  def calcPath(self,robot_pos, goal_pos, obs):
    # RRT
    rrt=RRT(robot_pos, goal_pos, self.map_dim, obs, self.algo_param["d_max"], self.algo_param["d_goal"], self.algo_param["iter_max"], self.algo_param["f_r_g"])
    rrt_path_coords, rrt_path_nodes = rrt.getPath()
    # RRTOD
    if (rrt_path_coords):
      rrtod = RRTOD(rrt_path_coords, rrt_path_nodes, obs)
      rrtod_path_coords = rrtod.getPath()
      if(rrtod_path_coords):
        return rrtod_path_coords
      else:
        return rrt_path_coords
    else:
      return None 

  def __call__(self):
    virtual_obs_file_path = f"{self.virtual_storage_path_obs}obstacles_{self.file_indx}.txt"
    virtual_pos_file_path = f"{self.virtual_storage_path_pos}positions_{self.file_indx}.yaml"
    virtual_traj_file_path = f"{self.virtual_storage_path_traj}trajectory_{self.file_indx}.txt"
    if (os.path.exists(virtual_pos_file_path)):
      # read robot/goal position and obstacles
      positions = self.loadYAML(virtual_pos_file_path)
      robot_pos = (positions["robot_x"], positions["robot_y"])
      goal_pos = (positions["goal_x"], positions["goal_y"])
      obs = self.loadPickle(virtual_obs_file_path)
      # calc trajectory
      path =  self.calcPath(robot_pos, goal_pos, obs)
      if (path):
        # save trajectory
        self.savePickle(virtual_traj_file_path, path)
        self.file_indx += 1

def main():
  path_server = PathCalcServer()
  while(True):
    path_server()
    time.sleep(0.2)

if __name__ == '__main__':
  main()
