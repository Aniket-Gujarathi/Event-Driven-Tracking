import cv2
import numpy as np 
from tqdm import tqdm

def  calculate_likelihood(image):
    
    height, width, _ = image.shape
    f = open("data_exp_648.txt", "w")
    idx = np.where(np.any(image != [0, 0, 0], axis=-1)) # ids of events in the image
    
    for x_c in tqdm(range(0, width - 1, 10)): # all possible Xc state
        for y_c in tqdm(range(0, height - 1, 10)): # all possible Yc state
            for r in range(10, 50, 5): # all possible radius state
                score = 0
                likelihood = 3 # min likelihood
                for i in range(0, idx[0].size):
                    vx = idx[1][i]
                    vy = idx[0][i]

                    dist_circ = np.abs(np.sqrt((x_c - vx)**2 + (y_c - vy)**2) - r)                

                    if(dist_circ > 2.0):
                        continue   

                    cval = 0.0
                    if(dist_circ < 1.0):
                        cval = 1.0
                    elif(dist_circ < 2.0):
                        cval = 0.5
                    if(cval): 
                        improve = cval
                        if(improve > 0):
                            score += improve
                            if(score >= likelihood):
                                likelihood = score    
                    else:
                        score -= 0.5

                # write the data to a txt file
                f.write(str(x_c) + " " + str(y_c) + " " + str(r) + " " + str(score) + " " + str(likelihood) + "\n")

                        

if __name__ == '__main__':
    image = cv2.imread('/home/aniket/yarp-install/projects/particle-filter-tracking/cpu/test_img/00000648.jpg')
    calculate_likelihood(image)
    
    