import os

import time

import numpy as np

# import matplotlib.pyplot as plt
from bimvee.importAe import importAe
from bimvee.exportIitYarp import exportIitYarp
from bimvee.info import info

PATH = os.path.abspath(os.getcwd())

file_path = os.path.join(PATH, "newData/out_2020-10-29_13-46-33.raw")
outPath = os.path.join(PATH, "yarp_out_txt")
container1 = importAe(filePathOrName=file_path, zeroTime=True)

info(container1)

exportIitYarp(container1,
            exportFilePath=os.path.join(outPath, "batch05_n"),
            pathForPlayback=os.path.join(outPath, "batch05_n"))
print('Done!')
