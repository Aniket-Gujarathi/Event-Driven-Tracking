import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from scipy.stats import norm
import statistics
from sys import argv, exit
import cv2

def draw_gauss(image, xl, xh, yl, yh):
    crop_eye = image[xl:xh, yl:yh]
    
    y_axis = np.arange(yl, yh)
    x_axis = np.arange(xl, xh)
    X, Y = np.meshgrid(x_axis, y_axis)
    R = np.sqrt(X**2 + Y**2)
    sd_x1 = sd_y1 = 17
    sd_x2 = sd_y2 = 20
    
    g1 = (1 / np.sqrt(2 * np.pi * sd_x1 * sd_y1)) * np.exp(-((X - np.mean(X))**2 / (2 * sd_x1**2) + (Y - np.mean(Y))**2 / (2 * sd_y1**2)))
    g2 = (1 / np.sqrt(2 * np.pi * sd_x2 * sd_y2)) * np.exp(-((X - np.mean(X))**2 / (2 * sd_x2**2) + (Y - np.mean(Y))**2 / (2 * sd_y2**2)))

    dog = g1 - g2
    
    # fig = plt.figure()
    # ax = plt.axes(projection='3d')
    # ax = fig.gca(projection='3d')
    # ax.plot_surface(X, Y, dog)
    # ax.set_xlabel('x')
    # ax.set_ylabel('y')
    # ax.set_zlabel('z')
    # plt.show()
    
    idx = np.where(dog < -0.0015)
    for i in range(len(idx[0])):
        crop_eye[idx[0][i], idx[1][i]] = 255
    
    

    image[xl:xh, yl:yh] = crop_eye

    cv2.imshow('a', image)
    cv2.waitKey(0)


if __name__ == '__main__':
    filename = argv[1] # image path
    xl, xh = 80, 186
    yl, yh = 132, 243
    image = cv2.imread(filename)

    draw_gauss(image, xl, xh, yl, yh)
