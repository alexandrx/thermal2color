import numpy as np
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import colorsys
import os

"""
    Loading Color Palette Table (CPT) files.
    Based on: https://github.com/j08lue/pycpt/blob/master/pycpt/load.py 
    Original author: Jonas <j08lue@gmail.com>
"""
def gmtColormap_openfile(cptf, name=None):
    """Read a GMT color map from an OPEN cpt file
    Parameters
    ----------
    cptf : open file or url handle
        path to .cpt file
    name : str, optional
        name for color map
        if not provided, the file name will be used
    """
    # generate cmap name
    if name is None:
        name = '_'.join(os.path.basename(cptf.name).split('.')[:-1])

    # process file
    x = []
    r = []
    g = []
    b = []
    lastls = None
    for l in cptf.readlines():
        ls = l.split()

        # skip empty lines
        if not ls:
            continue

        # parse header info
        if ls[0] in ["#", b"#"]:
            if ls[-1] in ["HSV", b"HSV"]:
                colorModel = "HSV"
            else:
                colorModel = "RGB"
            continue

        # skip BFN info
        if ls[0] in ["B", b"B", "F", b"F", "N", b"N"]:
            continue

        # parse color vectors
        x.append(float(ls[0]))
        r.append(float(ls[1]))
        g.append(float(ls[2]))
        b.append(float(ls[3]))

        # save last row
        lastls = ls

    x.append(float(lastls[4]))
    r.append(float(lastls[5]))
    g.append(float(lastls[6]))
    b.append(float(lastls[7]))
    
    x = np.array(x)
    r = np.array(r)
    g = np.array(g)
    b = np.array(b)

    if colorModel == "HSV":
        for i in range(r.shape[0]):
            # convert HSV to RGB
            rr,gg,bb = colorsys.hsv_to_rgb(r[i]/360., g[i], b[i])
            r[i] = rr ; g[i] = gg ; b[i] = bb
    elif colorModel == "RGB":
        r /= 255.
        g /= 255.
        b /= 255.

    red = []
    blue = []
    green = []
    xNorm = (x - x[0])/(x[-1] - x[0])
    for i in range(len(x)):
        red.append([xNorm[i],r[i],r[i]])
        green.append([xNorm[i],g[i],g[i]])
        blue.append([xNorm[i],b[i],b[i]])

    # return colormap
    cdict = dict(red=red,green=green,blue=blue)
    return mcolors.LinearSegmentedColormap(name=name,segmentdata=cdict)

def gmtColormap(cptfile, name=None):
    """Read a GMT color map from a cpt file
    Parameters
    ----------
    cptfile : str or open file-like object
        path to .cpt file
    name : str, optional
        name for color map
        if not provided, the file name will be used
    """
    with open(cptfile, 'r') as cptf:
        return gmtColormap_openfile(cptf, name=name)

