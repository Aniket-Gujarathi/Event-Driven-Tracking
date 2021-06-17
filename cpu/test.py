import cv2
import numpy as np 
from tqdm import tqdm
from sys import argv, exit

def  calculate_likelihood(image):
    
    height, width, _ = image.shape
    f = open("test_648_circleNewTest_2736.txt", "w")
    idx = np.where(np.any(image != [0, 0, 0], axis=-1)) # ids of events in the image
    
    for x_c in tqdm(range(0, width - 1, 5)): # all possible Xc state
        for y_c in tqdm(range(0, height - 1, 5)): # all possible Yc state
            for r in range(15, 40, 5): # all possible radius state
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
                    if(-2.0 <= dist_circ and dist_circ <= 2.0):
                        cval = 1.0
                    elif(-3.0 <= dist_circ and dist_circ < -2.0):
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

def findIntersection(vx, vy, x, y, a, b):
        if (vx == x):
            return None, None
        m = (y - vy) / (x - vx)
        if(m == 0):
            return None, None
        if vy >= y:
            y_on = y + (m*a*b) / (np.sqrt(b**2 + m**2*a**2))
        else:
            y_on = y - (m*a*b) / (np.sqrt(b**2 + m**2*a**2))
        # y_ell = self.findRoots(self.b**2 + (m*self.a)**2, -2*self.b*vy - m**2*self.a**2*2*vy, (self.b*vy)**2 + (m*self.a*vy)**2 - (m*self.a*self.b)**2 - (self.b*vx*m)**2, check)
        if (np.isnan(y_on)):
            return None, None
        x_on = x + ((y_on - y) / m)

        return int(x_on), int(y_on);

def  calculate_likelihood_ellipse(image):
    height, width, _ = image.shape
    f = open("test_648_ellipseROI.txt", "w")
    idx = np.where(np.any(image != [0, 0, 0], axis=-1)) # ids of events in the image
    
    for x_c in tqdm(range(95, 278, 5)): # all possible Xc state
        for y_c in tqdm(range(53, 217, 5)): # all possible Yc state
            for a in range(15, 40, 5): # all possible radius state
                for b in range(15, 40, 5):
                    score = 0
                    likelihood = 3 # min likelihood
                    for i in range(0, idx[0].size, 10):
                        vx = idx[1][i]
                        vy = idx[0][i]

                        x_on, y_on = findIntersection(vx, vy, x_c, y_c, a, b)
                        if y_on == None or x_on == None:
                            continue
                        dist_cen_ell = np.sqrt((x_on - x_c)**2 + (y_on - y_c)**2)
                        dist = np.sqrt((vx - x_c)**2 + (vy - y_c)**2) - dist_cen_ell
                                                            
                        if(dist > 2.0):
                            score -= 0.1
                            continue   

                        cval = None
                        if(-2.0 <= dist <= 2.0):
                            cval = 1.0
                        elif(-3.0 <= dist < -2.0):
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
                        f.write(str(x_c) + " " + str(y_c) + " " + str(a) + " " + str(b) + " " + str(score) + " " + str(likelihood) + "\n")
                        

def  calculate_likelihood_2circles(image):
    
    height, width, _ = image.shape
    f = open("test_648_2circlesROI_gaussTest.txt", "w")
    idx = np.where(np.any(image != [0, 0, 0], axis=-1)) # ids of events in the image
    
    for x_c in tqdm(range(95, 278, 5)): # all possible Xc state
        for y_c in tqdm(range(53, 217, 5)): # all possible Yc state
            for r1 in range(20, 35, 5): # all possible radius state
                for r2 in range(5, 15, 5): # all possible radius state
                    score = 0
                    likelihood = 3 # min likelihood
                    for i in range(0, idx[0].size, 10):
                        vx = idx[1][i]
                        vy = idx[0][i]

                        dist_circ = (np.sqrt((x_c - vx)**2 + (y_c - vy)**2))
                        # dist_circ2 = (np.sqrt((x_c - vx)**2 + (y_c - vy)**2) - r2)                    
                        
                        cval = None
                        if(dist_circ > r1 + 2.0):
                            # score -= 0.1
                            continue 
                        elif (dist_circ <= r2 and dist_circ >= r2):
                            cval = 1.0
                        elif (dist_circ > r2 and dist_circ < r1 - 2.0):
                            cval = -0.5
                        elif (dist_circ >= r1 - 2.0 and dist_circ < r1 + 2.0):
                            cval = 0.5
                            
                        if (cval):
                            if(cval >= 0.0): 
                                score += cval
                                if(score >= likelihood):
                                    likelihood = score    
                            elif (cval < 0.0):
                                score += cval

                    if (score != 0):
                        # write the data to a txt file
                        f.write(str(x_c) + " " + str(y_c) + " " + str(r1) + " " + str(r2) + " " + str(score) + " " + str(likelihood) + "\n")


if __name__ == '__main__':
    img_path = argv[1]
    image = cv2.imread(img_path)
    # calculate_likelihood(image)
    #calculate_likelihood_ellipse(image)
    calculate_likelihood_2circles(image)
    
    