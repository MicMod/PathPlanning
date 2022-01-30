from gc import set_debug
import os
import yaml
from pathlib import Path
from azure.storage.fileshare import ShareFileClient
from azure.storage.fileshare import ShareDirectoryClient

class FileShareAccess:
  def __init__(self):
    self.config = self.loadConfig("/config/file_share_params.yaml")

  def loadConfig(self, relative_path):
    path_current_file_str = os.path.dirname(os.path.abspath(__file__))
    path_current_file = Path(path_current_file_str)
    path_config_file = str(path_current_file.parent.absolute()) + relative_path
    with open(path_config_file, 'r') as yamlfile:
      return yaml.load(yamlfile, Loader=yaml.FullLoader)

  def upload(self, local_file_path, virtual_file_path):
    file_client = ShareFileClient.from_connection_string(conn_str=self.config["connection_string"], share_name=self.config["share_name"], file_path=virtual_file_path)
    with open(local_file_path, "rb") as source_file:
      file_client.upload_file(source_file)

  def download(self, local_file_path, virtual_file_path):
    file_client = ShareFileClient.from_connection_string(conn_str=self.config["connection_string"], share_name=self.config["share_name"], file_path=virtual_file_path)

    with open(local_file_path, "wb") as file_handle:
      data = file_client.download_file()
      data.readinto(file_handle)

  def checkFile(self, dir_name, file_name):
    parent_dir = ShareDirectoryClient.from_connection_string(conn_str=self.config["connection_string"], share_name=self.config["share_name"], directory_path=dir_name)
    
    file_list = list(parent_dir.list_directories_and_files())
    
    for file in file_list:
      if file["name"] == file_name:
        return True
    
    return False




