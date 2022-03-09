import numpy as np
import matplotlib

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
    

K = []
r = []
g = []
b = []
with open("bbr_color.txt", 'r') as h:
    for l in h.readlines():
            ls = l.split()
            if not ls:
                    continue
            if ls[0] in ["#", b"#"]:
                    continue
            if ls[2] == "10deg":
            	K.append(int(ls[0]))
            	r.append(int(ls[9]))
            	g.append(int(ls[10]))
            	b.append(int(ls[11]))

rgb = np.array([np.array([r[i], g[i], b[i]]) for i in range(len(r))])
rgbflip = np.flip(rgb, axis=0)
unique = np.unique(rgbflip, axis=0)
ncolors=256
alpha = np.array([np.append(np.array(c)/256,1) for c in unique])
cmap = matplotlib.colors.ListedColormap(alpha, "blackbody")
cmap2cpt(cmap, "blackbody10deg.cpt", ncolors)
