import os
import time
import numpy as np
import argparse

#### Argument Parsing ####
parser = argparse.ArgumentParser(description='To convert into yarp-readable data')
parser.add_argument(
	'--data', type=str, default = '/home/aniket/event-driven-gaze-tracking/Data/Raw_Data/out_2020-10-29_13-46-33.raw',
	help='path to the raw event-data'
)
args = parser.parse_args()

# import matplotlib.pyplot as plt
from bimvee.importAe import importAe
from bimvee.exportIitYarp import exportIitYarp
from bimvee.info import info

PATH = os.path.abspath(os.getcwd())

file_path = os.path.join(PATH, args.data)
outPath = os.path.join(PATH, "yarp_out")
container1 = importAe(filePathOrName=file_path, zeroTime=True)

info(container1)

exportIitYarp(container1,
            exportFilePath=os.path.join(outPath, "batch05"),
            pathForPlayback=os.path.join(outPath, "batch05"))
print('Done!')
