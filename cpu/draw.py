import cv2
import numpy as np
from sys import argv

def read_data(file):
    file = open(file, "r")
    x_c = []
    y_c = []
    r = []
    score = []
    likelihood = []
    for line in file:
        data = line.split()
        x_c.append(int(data[0]))
        y_c.append(int(data[1]))
        r.append(int(data[2]))
        score.append(float(data[3]))
        likelihood.append(data[4])

    return x_c, y_c, r, score, likelihood

def normalize(score, max, min):
    norm_score = ((score - min) * (255 / (max - min))).astype(int)

    return norm_score 

def draw(x_c, y_c, r, norm_score):
    im = cv2.imread('/home/aniket/yarp-install/projects/particle-filter-tracking/cpu/test_img/00001217.jpg')
    for i in range(len(x_c)):
        if norm_score[i] < 85:
            image = cv2.circle(im, (x_c[i], y_c[i]), r[i], (int(norm_score[i]), 0, 0), 1)
        elif norm_score[i] < 170:
            image = cv2.circle(im, (x_c[i], y_c[i]), r[i], (0, int(norm_score[i]), 0), 1)
        elif norm_score[i] < 255:
            image = cv2.circle(im, (x_c[i], y_c[i]), r[i], (0, 0, int(norm_score[i])), 1)
    cv2.imshow('image', image)
    #cv2.imwrite('/home/aniket/yarp-install/projects/particle-filter-tracking/cpu/test_img/exp_test.jpg', image)
    cv2.waitKey(0)

if __name__ == "__main__":
    file = argv[1]
    x_c, y_c, r, score, likelihood = read_data(file)
    max_score = np.amax(score)
    min_score = np.amin(score)
    norm_score = normalize(score, max_score, min_score)
    draw(x_c, y_c, r, norm_score)        


        
