# 3D-Reconstruction
Implementation of stereo-reconstruction algorithm. Reprojects points in relative 3D space using triangulation and stereo-matching from a pair of calibrated images.

## Algorithm
**Given:** Stereo-rectified pair of images, given camera intrinsics (K1, K2), point correspondences between left and right images. 
A high-level overview of the algorithm is given below -
1. Estimate Fundamental Matrix (F) using given point correspondences.
2. Construct Essential Matrix (E) using F, K1, and K2.
3. 

## Results