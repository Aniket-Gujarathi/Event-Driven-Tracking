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

        return int(x_on), int(y_on)

def  calculate_likelihood_ellipse(image):
    height, width, _ = image.shape
    f = open("test_2736_ellipseFull.txt", "w")
    idx = np.where(np.any(image != [0, 0, 0], axis=-1)) # ids of events in the image
    
    for x_c in tqdm(range(0, width - 1, 5)): # all possible Xc state
        for y_c in tqdm(range(0, height - 1, 5)): # all possible Yc state
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
                        if (vx > 278 or vy > 217 or vx < 95 or vy < 53):
                            continue
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
    f = open("test_648_2circlesROI_newConditions.txt", "w")
    idx = np.where(np.any(image != [0, 0, 0], axis=-1)) # ids of events in the image
    
    for x_c in tqdm(range(95, 278, 5)): # all possible Xc state
        for y_c in tqdm(range(53, 217, 5)): # all possible Yc state
            for r1 in range(18, 35, 3): # all possible radius state
                for r2 in range(5, 15, 2): # all possible radius state
                    score = 0
                    likelihood = 3 # min likelihood
                    for i in range(0, idx[0].size, 10):
                        vx = idx[1][i]
                        vy = idx[0][i]
                        dx = vx - x_c
                        dy = vy - y_c
                        
                        dist_big = (np.sqrt((dx)**2 + (dy)**2)) - r1
                        dist_small = (np.sqrt((dx)**2 + (dy)**2))                    
                        
                        cval = None
                        if(dist_big > 2.0):
                            score -= 0.1
                            continue 
                        elif (-2.0 <= dist_big and dist_big <= 2.0 or  dist_small <= r2):
                            cval = 1.0
                        elif (-4.0 <= dist_big and dist_big < -2.0 or dist_small > r2 and dist_small < r2 + 2.0):
                            cval = -0.5
                            
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
def findIntersection_par(x, y, vx, vy, m, c):
        check = 0
        if x == vx:
            return None, None
        line_m = (y - vy) / (x - vx)
        if line_m == 0:
            return None, None

        line_c = y - x * line_m

        if vy > y:
            check = 1

        y_par = int(findRoots(m*m + (2*m/line_m) + (1/(line_m*line_m)), -2*y - 2*m*m*y - 2*m*(line_c/line_m) + 2*c - 2*(x/line_m) - 2*m*m*(x/line_m) - 2*m*(c/line_m) - 2*(line_c/(line_m*line_m)), y*y + (line_c*line_c/(line_m*line_m)) + x*x + 2*x*(line_c/line_m) + m*m*y*y + m*m*x*x + 2*m*c*(line_c/line_m) - c*c + 2*x*m*m*line_c/line_m, check))
        x_par = int((y_par - line_c) / line_m)
        
        return int(x_par), int(y_par)

def findRoots(a, b, c, check):
        if (a == 0):
            return None
        
        d = b * b - 4 * a * c
        sqrt_val = np.sqrt(np.abs(d))

        if (d > 0):
            if check == 0:
                return (-b - sqrt_val) / (2 * a)
            if check == 1:
                return (-b + sqrt_val) / (2 * a)
        elif (d == 0):
                return - b / (2 * a)
        else:
            return None

def calculate_likelihood_par(image):
        # idx = np.where(np.any(image != [0, 0, 0], axis=-1))
        idx = np.where(np.any(image != [255, 255, 255], axis=-1) & np.any(image <= [200, 200, 200], axis=-1))
        print(image[102, 542])
        height, width, _ = image.shape
        f = open("test_image3_negWhiteIdxcond.txt", "w")
        for x_c in tqdm(range(150, 300, 5)): # all possible Xc state
            for y_c in tqdm(range(95, 182, 5)): # all possible Yc state
                for theta in range(-6, 6): # all possible radius state
                    for c in range(60, 100, 10): # all possible radius state
                        m = np.tan(theta * np.pi / 180)
                        score = 0
                        likelihood = 3 # min likelihood
                        for i in range(0, idx[0].size, 10):
                            vx = idx[1][i]
                            vy = idx[0][i]

                            x_on, y_on = findIntersection_par(x_c, y_c, vx, vy, m, c)
                            if (x_on == None or y_on == None):
                                continue
                            dist_foc_par = abs(np.sqrt(float((x_c - x_on)**2 + (y_c - y_on)**2)))
                            fdist_par = (np.sqrt(float((vx - x_c)**2 + (vy - y_c)**2))) - dist_foc_par

                            if y_on >= y_c or x_on > x_c + 65 or x_on < x_c - 65:
                                continue

                            
                            if(fdist_par > 20 or fdist_par < -20):
                                # cv2.circle(image, (vx, vy), 0, [0, 0, 0], 5)
                                # cv2.circle(image, (x_c, y_c), 0, [100, 100, 0], 5)
                                # cv2.imshow("a", image)
                                # cv2.waitKey(0)
                                continue
                            
                            cval = None
                            # pts = []
                            # for i in range(154, 310, 5):
                            #     y = m*i + c
                            #     pts.append([i, int(y)])
                            
                            if(-2.0 <= fdist_par and fdist_par <= 2.0):
                                cval = 1.0
                                # cv2.circle(image, (x_on, y_on), 0, [100, 100, 255], 5)
                                # cv2.circle(image, (vx, vy), 0, [0, 0, 0], 5)
                                # # cv2.circle(image, (x_c, y_c), 0, [255, 255, 255], 5)
                                # cv2.polylines(image, np.array([pts]), False, [255, 0, 0], 2)
                                # cv2.imshow("a", image)
                                # cv2.waitKey(0)
                            elif (2.0 < fdist_par and fdist_par < 20.0):
                                cval = -1.0

                            if(cval):
                                if(cval >= 0.0):
                                    score += cval
                                    if(score >= likelihood):
                                        likelihood = score
                                    
                                elif (cval < 0.0):
                                    score -= 0.5
                            else:
                                continue   
                            

                            if (score != 0):
                                # write the data to a txt file
                                f.write(str(x_c) + " " + str(y_c) + " " + str(theta) + " " + str(c) + " " + str(score) + " " + str(likelihood) + "\n")

def calculate_likelihood_parDir(image, filename):
        # idx = np.where(np.any(image != [0, 0, 0], axis=-1) & np.any(image < [100, 100, 100], axis=-1))
        idx = np.where(np.any(image != [255, 255, 255], axis=-1) & np.any(image < [200, 200, 200], axis=-1))
        
        height, width, _ = image.shape
        file = open(filename, "w")
        for x_c in (range(219, 311, 5)): # all possible Xc state
            for y_c in (range(88, 152, 5)): # all possible Yc state
                for theta in range(-6, 6): # all possible radius state
                    for c in range(60, 100, 10): # all possible radius state
                        m = np.tan(theta * np.pi / 180)
                        dist_par_dir = abs((m * x_c - y_c + c) / (np.sqrt(1 + m**2))) / 2.0
                        score = 0
                        likelihood = 3 # min likelihood
                        for i in range(0, idx[0].size, 10):
                            vx = idx[1][i]
                            vy = idx[0][i]
                            dx = vx - x_c
                            dy = vy - y_c

                            dist_focus = abs(np.sqrt(dx**2 + dy**2))
                            dist_directrix = abs((m * vx - vy + c) / (np.sqrt(1 + m**2)))

                            dist_diff = abs(dist_directrix - dist_focus)
                            # pts = []
                            # for i in range(150, 324):
                            #     yl = m*i + c
                            #     pts.append([i, int(yl)])
                                
                            cval = None
                            if(dist_focus > 60.0 or dist_par_dir < 5.0 or vy < m*vx + c or vy > y_c + 10):
                                continue
                            elif(dist_diff <= 10.0):
                                cval = 1.0
                                # cv2.circle(image, (vx, vy), 0, [0, 0, 0], 5)
                                # cv2.line(image, (vx, vy), (x_c, y_c), [255, 100, 0], 2)
                                # cv2.polylines(image, np.array([pts]), False, [255, 0, 0], 2)
                                # cv2.circle(image, (x_c, y_c), 60, [0, 0, 0], 1)
                                # cv2.imshow('a', image)
                                # cv2.waitKey(1)
                            elif (dist_directrix < 5.0):
                                cval = -0.8
                            elif(dist_directrix > dist_focus):
                                cval = 0.3
                            elif(dist_directrix < dist_focus):
                                cval = -0.3

                            if(cval):
                                if(cval >= 0.0):
                                    score += cval
                                    if(score >= likelihood):
                                        likelihood = score
                                    
                                elif (cval < 0.0):
                                    score += cval
                            else:
                                continue   
                            
                        if (score != 0):
                            # write the data to a txt file
                            file.write(str(x_c) + " " + str(y_c) + " " + str(theta) + " " + str(c) + " " + str(score) + " " + str(likelihood) + "\n")

def Gauss(sd, X, Y):
    g = (1 / np.sqrt(2 * np.pi * sd * sd)) * np.exp(-((X - np.mean(X))**2 / (2 * sd**2) + (Y - np.mean(Y))**2 / (2 * sd**2)))

    return g

def calculate_likelihood_DoG(image):
    # xl, xh = 80, 280
    # yl, yh = 40, 240
    height, width, _ = image.shape
    f = open("test_2736_DoGTest_neg.txt", "w")
    idx = np.where(np.any(image != [0, 0, 0], axis=-1)) # ids of events in the image

    for x_c in tqdm(range(95, 278, 5)):
        xl = x_c - 100
        xh = x_c + 100
        if xl < 0:
            continue
        if xh >= width:
            break
        for y_c in tqdm(range(53, 217, 5)):
            yl = y_c - 100
            yh = y_c + 100
            if (yl < 0):
                continue
            if (yh >= height):
                break
            for sd1 in range(15, 35, 5): # all possible radius state
                for sd2 in range(14, sd1, 5): # all possible radius state         
                    if (sd2 >= sd1):
                        break
                    score = 0
                    likelihood = 3 # min likelihood
                    for i in range(0, idx[0].size, 10):
                        vx = idx[1][i]
                        vy = idx[0][i]
                        
                        if (vx > xh or vx < xl or vy > yh or vy < yl):
                            continue
                        y_axis = np.arange(yl, yh)
                        x_axis = np.arange(xl, xh)
                        X, Y = np.meshgrid(x_axis, y_axis)
                        
                        g1 = Gauss(sd1, X, Y)
                        g2 = Gauss(sd2, X, Y)

                        dog = g2 - g1

                        idx_pos = np.where(dog > 0.0)
                        idx_neg = np.where(dog < -0.001)

                        cval = None
                        if (vx, vy) in zip(idx_pos[1], idx_pos[0]):
                            cval = -0.5

                        if (vx, vy) in zip(idx_neg[1], idx_neg[0]):
                            cval = 1

                        if (cval):
                            if(cval >= 0.0): 
                                score += cval
                                if(score >= likelihood):
                                    likelihood = score    
                            elif (cval < 0.0):
                                score += cval
                        else:
                            continue

                    if (score != 0):
                        # write the data to a txt file
                        f.write(str(x_c) + " " + str(y_c) + " " + str(sd1) + " " + str(sd2) + " " + str(score) + " " + str(likelihood) + "\n")


if __name__ == '__main__':
    img_path = argv[1]
    file_name = argv[2]

    image = cv2.imread(img_path)
    image = cv2.resize(image, (640, 480))
    # calculate_likelihood(image)
    # calculate_likelihood_ellipse(image)
    # calculate_likelihood_2circles(image)
    # calculate_likelihood_DoG(image)
    # calculate_likelihood_par(image)
    calculate_likelihood_parDir(image, file_name)
    
    
