
from epipolar_geometry import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

def main():
    templeCoords = np.load('./data/templeCoords.npz')
    data = np.load('./data/some_corresp.npz')
    intrinsics = np.load('./data/intrinsics.npz')
    im1 = plt.imread('./data/im1.png')
    im2 = plt.imread('./data/im2.png')
    pts1 = data['pts1']
    pts2 = data['pts2']
    K1 = intrinsics['K1']
    K2 = intrinsics['K2']
    x1 = templeCoords['x1']
    y1 = templeCoords['y1']
    totalpts = x1.shape[0]
    x2 = np.zeros((totalpts, 1))
    y2 = np.zeros((totalpts, 1))
    epipolar = epi_geom(pts1, pts2, K1, K2)
    F = epipolar.estimate_F(im1, im2)
    E = epipolar.estimate_E(F)
    epipolar.visualize3D(totalpts, x2, y2, x1, y1, im1, im2)
    
if __name__ == '__main__':    
    main()