import cv2
import numpy as np
from sys import argv, exit
from test import calculate_likelihood_parDir
from tqdm import tqdm
import os

def likelihood(data_folder):
    for image_name in tqdm(os.listdir(data_folder)):
        image = cv2.imread(os.path.join(data_folder, image_name))
        image = cv2.resize(image, (640, 480))
        # To write the data for all images
        filename = os.path.join(data_folder, os.path.splitext(image_name)[0] + ".txt") 
        calculate_likelihood_parDir(image, filename)

if __name__ == '__main__':
    data_folder = argv[1]
    likelihood(data_folder)
