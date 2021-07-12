import cv2
import numpy as np
from sys import argv
import matplotlib.pyplot as plt

def read_data(file):
    file = open(file, "r")
    x_c = []
    y_c = []
    # r = []
    theta = []
    c = []
    score = []
    likelihood = []
    for line in file:
        data = line.split()
        x_c.append(int(data[0]))
        y_c.append(int(data[1]))
        theta.append(int(data[2]))
        c.append(int(data[3]))
        score.append(float(data[4]))
        # likelihood.append(data[5])
    
    # f = open('/home/aniket/yarp-install/projects/particle-filter-tracking/cpu/ParabolaTesting/test_image3_parDir_Abs_trial.txt', 'w')
    # for i in range(len(x_c) - 1):
    #     # if score[i] == 0:
    #     #     continue
    #     if x_c[i] != x_c[i+1] or y_c[i] != y_c[i + 1]:
    #         f.write(str(x_c[i]) + " " + str(y_c[i]) + " " + str(theta[i]) + " " + str(c[i]) + " " + str(score[i]) + "\n")
    
    return x_c, y_c, theta, c, score, likelihood

def normalize(score, max, min):
    norm_score = ((score - min) * (255 / (max - min))).astype(int)

    return norm_score 


def findRoots(self, a, b, c, check=0):
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

def drawPara(image, x, y, theta, c, norm_score):
    m = np.tan(theta * np.pi / 180)
    pts = []
    for i in range(150, 324):
        y_par = findRoots((m*m), (-2*y*(1 + m*m) + 2*c + 2*m*i), (i*i + i*(-2*x*(1 + m*m) - 2*m*c) + (x*x + y*y)*(1 + m*m) - c*c), 0)
        if (y_par == None or y_par > y):
            continue
        y_par = int(y_par)
        pts.append([i, y_par])
    cv2.polylines(image, np.array([pts]), False, [0, 0, int(norm_score)], 1)

def draw(x_c, y_c, theta, c, norm_score, img_path):
    im = cv2.imread(img_path)
    im = cv2.resize(im, (640, 480))
    for i in range(0, len(x_c)):
        if norm_score[i] < 85:
            print('a')
            # drawPara(im, x_c[i], y_c[i], theta[i], c[i], norm_score[i])
            # image = cv2.circle(im, (x_c[i], y_c[i]), 0, (int(norm_score[i]), 0, 0), 5)
        elif norm_score[i] < 220:
            print('b')
            # drawPara(im, x_c[i], y_c[i], theta[i], c[i], norm_score[i])
            # image = cv2.circle(im, (x_c[i], y_c[i]), 0, (0, int(norm_score[i]), 0), 5)
        elif norm_score[i] < 255:
            drawPara(im, x_c[i], y_c[i], theta[i], c[i], norm_score[i])
            image = cv2.circle(im, (x_c[i], y_c[i]), 0, (0, 255, int(norm_score[i])), 5)
    cv2.imshow('image', image)
    cv2.waitKey(0)
    


if __name__ == "__main__":
    file = argv[1]
    img_path = argv[2]
    # file_2 = argv[3]
    # img_path_2 = argv[4]
    

    x_c, y_c, theta, c, score, likelihood = read_data(file)
    # x_c_2, y_c_2, r_2, score_2, likelihood_2 = read_data(file_2)

    # r_50_idx = np.where(np.array(r) == 45)
    # print(r_50_idx[0], r[1])
    # x_c_n = []
    # y_c_n = []
    # r_n = []
    # score_n = []
    # lk_n = []
    # for ids in r_50_idx[0]:
    #     print(ids)
    #     x_c_n.append(x_c[ids])
    #     y_c_n.append(y_c[ids])
    #     r_n.append(r[ids])
    #     score_n.append(score[ids])        
    #     lk_n.append(likelihood[ids])

    max_score = np.amax(score)
    min_score = np.amin(score)
    
    # max_score_2 = np.amax(score_2)
    # min_score_2 = np.amin(score_2)
    # print(np.amax(score), np.amax(score_2))
    # max_pix = np.where(score == np.amax(score))
    # print(r_2[int(max_pix[0])])
    # im = cv2.imread(img_path)
    # image = cv2.circle(im, (x_c[int(max_pix[0])], y_c[int(max_pix[0])]), 0, (255, 0, 0), 2)
    # cv2.imshow('image', image)
    # cv2.waitKey(0)

    # print(max_score, np.amax(score_2), min_score, np.amin(score_2))
    norm_score = normalize(score, max_score, min_score)
    print(norm_score)
    # norm_score_2 = normalize(score_2, max_score_2, min_score_2)
    draw(x_c, y_c, theta, c, norm_score, img_path)
    # im2 = draw(x_c_2, y_c_2, r_2, norm_score_2, img_path_2)        

    # f = plt.figure()
    # f.add_subplot(1, 2, 1)
    # plt.imshow(im1)
    # f.add_subplot(1, 2, 2)
    # plt.imshow(im2)
    # plt.show()
        
