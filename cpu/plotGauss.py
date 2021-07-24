import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from scipy.stats import norm
import statistics
from sys import argv, exit
import cv2

def getGaussNew(image, xl, xh, yl, yh):
    sum = ((xh - xl) / 2) * (xl + xh)
    meanx = (sum) / (xh - xl)
    
    sum = ((yh - yl) / 2) * (yl + yh)
    meany = sum / (yh - yl)
    # meanx = np.mean(np.arange(xl, xh))
    # meany = np.mean(np.arange(yl, yh))
    sd1 = 20
    sd2 = 17
    print(meanx, meany)

    for i  in range(xl, xh):
        for j in range(yl, yh):
            g1 = (1 / np.sqrt(2 * np.pi * sd1 * sd1)) * np.exp(-((i - meanx)**2 / (2 * sd1 * sd1) + (j - meany)**2 / (2 * sd1 * sd1)))
            g2 = (1 / np.sqrt(2 * np.pi * sd2 * sd2)) * np.exp(-((i - meanx)**2 / (2 * sd2 * sd2) + (j - meany)**2 / (2 * sd2 * sd2)))
            dog = g2 - g1
            if dog < -0.001:
                image[j, i] = 100
            
    cv2.rectangle(image, (xl, yl), (xh, yh), [255, 0, 0], 2)
    # image[yl:yh, xl:xh] = crop_eye
    cv2.imshow('a', image)
    cv2.waitKey(0)
    


def draw_gauss(image, xl, xh, yl, yh):
    crop_eye = image[yl:yh, xl:xh]
    
    y_axis = np.arange(yl, yh)
    x_axis = np.arange(xl, xh)
    X, Y = np.meshgrid(x_axis, y_axis)
    
    sd_x1 = sd_y1 = 20
    sd_x2 = sd_y2 = sd_x1 - 3
    sd_x3 = sd_y3 = sd_x4 = sd_y4 = 50
    
    g1 = (1 / np.sqrt(2 * np.pi * sd_x1 * sd_y1)) * np.exp(-((X - np.mean(X))**2 / (2 * sd_x1**2) + (Y - np.mean(Y))**2 / (2 * sd_y1**2)))
    g2 = (1 / np.sqrt(2 * np.pi * sd_x2 * sd_y2)) * np.exp(-((X - np.mean(X))**2 / (2 * sd_x2**2) + (Y - np.mean(Y))**2 / (2 * sd_y2**2)))
    # g3 = (1 / np.sqrt(2 * np.pi * sd_x3 * sd_y3)) * np.exp(-((X - (np.mean(X) - 40))**2 / (2 * sd_x3**2) + (Y - (np.mean(Y) - 40))**2 / (2 * sd_y3**2)))
    # g4 = (1 / np.sqrt(2 * np.pi * sd_x4 * sd_y4)) * np.exp(-((X - (np.mean(X) + 40))**2 / (2 * sd_x4**2) + (Y - (np.mean(Y) + 40))**2 / (2 * sd_y4**2)))
    dog = g2 - g1
    # dog = g2 - g1 + g3 + g4
    
    fig = plt.figure()
    ax = plt.axes()
    ax = plt.axes(projection='3d')
    # ax = fig.gca(projection='3d')
    ax.plot_surface(X, Y, dog)
    ax.plot(X, dog)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_ylabel('z')
    plt.show()
    
    # idx = np.where(dog > 0.0)
    # for i in range(len(idx[0])):
    #     if (idx[0][i] >= dog.shape[0] or idx[1][i] >= dog.shape[1]):
    #         break
    #     crop_eye[idx[0][i], idx[1][i]] = 255

    idx2 = np.where(dog < -0.001)
    for i in range(len(idx2[0])):
        if (idx2[0][i] >= dog.shape[0] or idx2[1][i] >= dog.shape[1]):
            break
        crop_eye[idx2[0][i], idx2[1][i]] = 100
    
    cv2.rectangle(image, (xl, yl), (xh, yh), [255, 0, 0], 2)

    image[yl:yh, xl:xh] = crop_eye

    cv2.imshow('a', image)
    cv2.waitKey(0)


if __name__ == '__main__':
    filename = argv[1] # image path
    xl, xh = 80, 280
    yl, yh = 40, 240
    image = cv2.imread(filename)

    draw_gauss(image, xl, xh, yl, yh)
    # getGaussNew(image, xl, xh, yl, yh)