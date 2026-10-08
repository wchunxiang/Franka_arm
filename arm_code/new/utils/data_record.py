import numpy as np
import copy
from pathlib import Path
import os


class Datawriter(object):
    def __init__(self, data: list, file_path: str, header: str):
        output_folder = str(Path(file_path).parents[0])
        os.makedirs(output_folder, exist_ok=True)
        print('-----'+"Start recording data: "+ output_folder +'-----')
        self.file_path = file_path
        self.doc_header = header

        data_array = np.array(copy.deepcopy(data))
        self.data_save = data_array.reshape(1,-1)
    
    def write(self,data: list):
        data_array = np.array(copy.deepcopy(data)).reshape(1,-1)
        self.data_save = np.r_[self.data_save,data_array]

    def release(self):
        print('-----Finish recording data-----')
        np.savetxt(self.file_path, self.data_save, header=self.doc_header)