import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import colorsys
import os
from PIL import Image
import cv2
from sklearn.cluster import KMeans
from collections import Counter

def get_image(image_path):
    image = cv2.imread(image_path)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return image

def cmap2cpt(cmap, cptfilename, ncolors=256):
    idx = np.linspace(0, ncolors+1, ncolors+1).astype(int)
    idx[-1] = ncolors
    cols=(cmap(np.linspace(0.,1.,ncolors+1))[:,:3]*255).astype(int)
    arr = np.c_[idx[:-1],cols[:-1],idx[1:],cols[1:]]
    fmt = "%d %3d %3d %3d %d %3d %3d %3d"
    
    B=cmap(0.)
    F=cmap(1.)
    N=np.array([0,0,0]).astype(float)
    ext = (np.c_[B[:3],F[:3],N[:3]].T*255).astype(int)
    extstr = "B {:3d} {:3d} {:3d}\nF {:3d} {:3d} {:3d}\nN {:3d} {:3d} {:3d}"
    ex = extstr.format(*list(ext.flatten()))
    
    np.savetxt(cptfilename, arr, fmt=fmt, header="# COLOR_MODEL = RGB", footer=ex, comments="")

"""
Based on:  https://towardsdatascience.com/color-identification-in-images-machine-learning-application-b26e770c4c71
"""
def cmapfromimage(image_path, ncolors=256, cmapname=None):
    img = get_image(image_path)
    img2 = img.reshape(img.shape[0]*img.shape[1], 3)
    # clf = KMeans(n_clusters = ncolors)
    # labels = clf.fit_predict(img2)
    # counts = Counter(labels)
    # center_colors = clf.cluster_centers_
    # # We get ordered colors by iterating through the keys
    # ordered_colors = [center_colors[i] for i in counts.keys()]
    # rgb_colors = [np.append(ordered_colors[i]/256., 1.) for i in counts.keys()]
    # unique = np.unique(img2, axis=0)
    unique = np.unique(img[0], axis=0)
    colors = [list(unique[i]) for i in range(len(unique))]
    colors.sort(key=lambda rgb: colorsys.rgb_to_hsv(*rgb))  #Note: sorting by color is very hard
    alpha = np.array([np.append(np.array(c)/256,1) for c in colors])
    cmap = matplotlib.colors.ListedColormap(alpha, cmapname)
    ## Plot example gradient
    # gradient = np.linspace(0, 1, len(alpha))
    # gradient = np.vstack((gradient, gradient))
    # plt.imshow(gradient, aspect='auto', cmap=cmap)
    # plt.show()
    return cmap

"""
If the input image is a screenshot of a color gradient
"""
def colorgradient2cpt(image_path, cptfilename, ncolors=256, cmapname=None):
    img = get_image(image_path)
    cmap = matplotlib.colors.ListedColormap(img[0]/255, cmapname)
    cmap2cpt(cmap, cptfilename, ncolors)

"""
Convert any color map from matplotlib into Color Palette Table (CPT) file
"""
def namedcmap2cpt(cmapname, cptfilename, ncolors=256):
    cmap = plt.get_cmap(cmapname)
    cmap2cpt(cmap, cptfilename, ncolors)