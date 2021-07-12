import cv2
import numpy as np
from sys import argv, exit
from tqdm import tqdm

class Ellipse():
    def __init__(self, x, y, a, b):
        self.x = x
        self.y = y
        self.a = a
        self.b = b

    def findRoots(self, a, b, c, check):
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

    def findIntersection(self, vx, vy):
        if (vx == self.x):
            return None, None
        m = (self.y - vy) / (self.x - vx)
        if(m == 0):
            return None, None
        if vy >= self.y:
            y_on = self.y + (m*self.a*self.b) / (np.sqrt(self.b**2 + m**2*self.a**2))
        else:
            y_on = self.y - (m*self.a*self.b) / (np.sqrt(self.b**2 + m**2*self.a**2))
        
        # For circle
        if (self.a == self.b):
            if vy >= self.y:
                y_on = self.y + (m*self.a) / (np.sqrt(1 + m**2))
            else:
                y_on = self.y - (m*self.a) / (np.sqrt(1 + m**2))

        if (np.isnan(y_on)):
            return None, None
        x_on = self.x + ((y_on - self.y) / m)

        return int(x_on), int(y_on)

    def draw_ell(self, image):
        idx = np.where(np.any(image != [0, 0, 0], axis=-1))

        pts = []
        pts2 = []
        pts_on = []
        for i in range(114, 250, 10):
            for k in range(0, idx[0].size, 1):
                    vx = idx[1][k]
                    vy = idx[0][k]
                    x_on, y_on = self.findIntersection(vx, vy)
                    if y_on == None or x_on == None:
                        continue
                    pts_on.append([x_on, y_on])
                    x_on, y_on = int(x_on), int(y_on)
                    x_ell = i
                    y_ell = self.y + self.b*(np.sqrt(1 - ((self.x - i) / self.a)**2))
                    if np.isnan(y_ell):
                        continue
                    pts.append([x_ell, int(y_ell)])
                    y_ell2 = self.y - self.b*(np.sqrt(1 - ((self.x - i) / self.a)**2))                    
                    if np.isnan(y_ell2):
                        continue
                    pts2.append([x_ell, int(y_ell2)])
                    x_ell, y_ell, y_ell2 = int(x_ell), int(y_ell), int(y_ell2)
                    cv2.circle(image, (x_on, y_on), 0, [0, 0, 255], 2)
                    # cv2.circle(image, (x_ell, y_ell), 0, [255, 100, 100], 2)
                    # cv2.circle(image, (x_ell, y_ell2), 0, [255, 100, 100], 2)
                    # cv2.line(image, (self.x, self.y), (vx, vy), [255, 255, 0], 2)
                    
                    dist_cen_ell = abs(np.sqrt((x_on - self.x)**2 + (y_on - self.y)**2))
                    dist = np.sqrt((vx - self.x)**2 + (vy - self.y)**2) - dist_cen_ell
                    # if (-3.0 < dist < 3.0):
                    #     cv2.line(image, (x_on, y_on), (vx, vy), [255, 100, 100], 2)
        cv2.polylines(image, np.array([pts]), False, [255, 0, 0], 2)
        cv2.polylines(image, np.array([pts2]), False, [255, 0, 0], 2)
        
        cv2.imshow('i', image)
        cv2.waitKey(0)


class Parabola():
    def __init__(self, x, y, c, theta):
        self.x = x
        self.y = y
        self.c = c
        self.theta = theta

        self.m = np.tan(theta * np.pi / 180)
                                
    def findIntersection(self, vx, vy):
        check = 0
        if self.x == vx:
            return None, None
        line_m = (self.y - vy) / (self.x - vx)
        if line_m == 0:
            return None, None

        line_c = self.y - self.x * line_m

        if vy > self.y:
            check = 1

        y_par = int(self.findRoots(self.m*self.m + (2*self.m/line_m) + (1/(line_m*line_m)), -2*self.y - 2*self.m*self.m*self.y - 2*self.m*(line_c/line_m) + 2*self.c - 2*(self.x/line_m) - 2*self.m*self.m*(self.x/line_m) - 2*self.m*(self.c/line_m) - 2*(line_c/(line_m*line_m)), self.y*self.y + (line_c*line_c/(line_m*line_m)) + self.x*self.x + 2*self.x*(line_c/line_m) + self.m*self.m*self.y*self.y + self.m*self.m*self.x*self.x + 2*self.m*self.c*(line_c/line_m) - self.c*self.c + 2*self.x*self.m*self.m*line_c/line_m, check))
        x_par = int((y_par - line_c) / line_m)
        
        return int(x_par), int(y_par)


    def findRoots(self, a, b, c, check):
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

    def draw_para(self, image):
        # vx = 200
        # vy = 139
        # idx = np.where(np.any(image != [0, 0, 0], axis=-1))
        idx = np.where(np.any(image != [255, 255, 255], axis=-1) & np.any(image <= [200, 200, 200], axis=-1))
        # idx = np.where(np.any(image != [255, 255, 255], axis=-1))
        
        print(image[123, 169])
        pts = []
        pts_on = []
        pts_pos1 = []
        pts_pos2 = []
        check = 0
        count = 0
        dist_par_dir = abs((self.m * self.x - self.y + self.c) / (np.sqrt(1 + self.m**2))) / 2.0
        print("dist", dist_par_dir)
        for i in range(175, 323, 10):
            for k in range(0, idx[0].size, 10):
                vx = idx[1][k] 
                vy = idx[0][k] 
                if (vy > self.y):
                    continue
                x_par = i
                y_par = self.findRoots((self.m*self.m), (-2*self.y*(1 + self.m*self.m) + 2*self.c + 2*self.m*i), (i*i + i*(-2*self.x*(1 + self.m*self.m) - 2*self.m*self.c) + (self.x*self.x + self.y*self.y)*(1 + self.m*self.m) - self.c*self.c), 0)
                if (y_par == None):
                    continue
                y_par = int(y_par)
                

                x_on, y_on = self.findIntersection(vx, vy)
                if (x_on == None or y_on == None):
                    continue
                
                dist_foc_par = abs(np.sqrt(float((self.x - x_on)**2 + (self.y - y_on)**2)))
                fdist_par = (np.sqrt((vx - self.x)**2 + (vy - self.y)**2)) - dist_foc_par

                if y_on == None or x_on > self.x + 65 or x_on < self.x - 65:
                    continue
                if y_par in range(114, 183):
                    pts.append([x_par, y_par])
                    pts_pos1.append([x_par, y_par + 10])
                    pts_pos2.append([x_par, y_par - 10])
                if y_on in range(114, 150):
                    pts_on.append([x_on, y_on])
                
                pts_dir = []
                pts_bel = []
                for i in range(173, 323):
                    yl = self.m*i + self.c
                    pts_bel.append([i, int(self.y + 10)])
                    pts_dir.append([i, int(yl)])

                # cv2.circle(image, (x_on, y_on), 0, [0, 0, 255], 2)
                if (fdist_par >= -2.0 and fdist_par <= 2.0):
                    count += 1
                    cv2.line(image, (vx, vy), (x_on, y_on), [0, 255, 255], 1)
                    cv2.line(image, (self.x, self.y), (vx, vy), [255, 255, 0], 1)
                    cv2.circle(image, (x_on, y_on), 0, [100, 100, 255], 1)
                    cv2.circle(image, (vx, vy), 0, [0, 0, 0], 1)
                    cv2.circle(image, (self.x, self.y), 60, [0, 0, 0], 1)
                    cv2.polylines(image, np.array([pts_bel]), False, [0, 0, 0], 2)
                    cv2.polylines(image, np.array([pts_dir]), False, [0, 0, 0], 2)
                    if ((x_on, y_on) == (vx, vy)):
                        cv2.circle(image, (x_on, y_on), 0, [60, 100, 25], 1)   
                # if (fdist_par < 10.0 and fdist_par > 2.0):
                #     cv2.line(image, (vx, vy), (x_on, y_on), [0, 0, 0], 2)
                # cv2.line(image, (self.x, self.y), (vx, vy), [255, 255, 0], 2)
        cv2.polylines(image, np.array([pts]), False, [255, 0, 0], 2)
        cv2.polylines(image, np.array([pts_pos1]), False, [255, 255, 0], 2)
        cv2.polylines(image, np.array([pts_pos2]), False, [255, 255, 0], 2)
        cv2.circle(image, (self.x, self.y), 0, [0, 100, 255], 5)
        print(count)
        cv2.imshow('i', image)
        cv2.waitKey(0)

                            
    
                                      

if __name__ == "__main__":
    filename = argv[1]
    image = cv2.imread(filename)
    image = cv2.resize(image, (640, 480))

    par = Parabola(241, 142, 80, 6)
    # par.likelihood_parDir(image)
    # par.likelihood(image)
    par.draw_para(image)

    # ell = Ellipse(190, 140, 20, 20)
    # ell.draw_ell(image)