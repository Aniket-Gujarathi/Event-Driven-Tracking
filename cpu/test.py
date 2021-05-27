import cv2
import numpy as np 
from tqdm import tqdm
from sys import argv, exit

def  calculate_likelihood(image):
    
    height, width, _ = image.shape
    f = open("test_648_26May.txt", "w")
    idx = np.where(np.any(image != [0, 0, 0], axis=-1)) # ids of events in the image
    
    for x_c in tqdm(range(0, width - 1, 5)): # all possible Xc state
        for y_c in tqdm(range(0, height - 1, 5)): # all possible Yc state
            for r in range(10, 40, 5): # all possible radius state
                score = 0
                likelihood = 3 # min likelihood
                for i in range(0, idx[0].size):
                    vx = idx[1][i]
                    vy = idx[0][i]

                    dist_circ = (np.sqrt((x_c - vx)**2 + (y_c - vy)**2) - r)                    # use real values 
                                                                                                # dont penalize outside events, only interior
                    if(dist_circ > 2.0):
                        score -= 0.1
                        continue   

                    cval = None
                    if(-2.0 <= dist_circ <= 2.0):
                        cval = 1.0
                    elif(-3.0 <= dist_circ < -2.0):
                        cval = -1.0
                    if (cval):
                        if(cval >= 0.0): 
                            improve = cval
                            if(improve > 0):
                                score += improve
                                if(score >= likelihood):
                                    likelihood = score    
                        elif (cval < 0.0):
                            score -= 0.5

                if (score != 0):
                    # write the data to a txt file
                    f.write(str(x_c) + " " + str(y_c) + " " + str(r) + " " + str(score) + " " + str(likelihood) + "\n")

                        

if __name__ == '__main__':
    img_path = argv[1]
    image = cv2.imread(img_path)
    calculate_likelihood(image)
    
    