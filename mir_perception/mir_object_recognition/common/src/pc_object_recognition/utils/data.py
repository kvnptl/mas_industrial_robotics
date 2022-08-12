from torch.utils.data import Dataset
# import pc_utils.extract_pcd as extract_pcd
from pc_object_recognition.utils.pc_utils import extract_pcd
import numpy as np

class infer_data(Dataset): # dataloaer for inference

    def __init__(self, num_points, pcl_path, transform=None):

        self.pcl_path = pcl_path
        self.num_points = num_points
        self.data = None

    def __len__(self):
        return 1000

    def __getitem__(self, x):
        
        self.data = extract_pcd(self.pcl_path,num_points=self.num_points)
        label = np.array([0])

        return self.data, label