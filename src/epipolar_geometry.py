
import numpy as np
from helper import *
# from reconstruction import *
import math
import math
from scipy.ndimage.filters import gaussian_filter
class epi_geom:
    def __init__(self, pts1, pts2, K1, K2, E = None, F = None, C1 = None, C2 = None):
        self.pts1 = pts1
        self.pts2 = pts2
        self.K1 = K1
        self.K2 = K2
        self.E = E
        self.F = F
        self.C1 = C1
        self.C2 = C2
    

    def estimate_F(self, im1, im2):
        M = max(im1.shape)
        self.pts1 = self.pts1 / M
        self.pts2 = self.pts2 / M
        totalpts = self.pts1.shape[0]

        a = np.zeros((totalpts, 9))
        a[:,0] = self.pts2[:,0]*self.pts1[:,0]
        a[:,1] = self.pts2[:,0]*self.pts1[:,1]
        a[:,2] = self.pts2[:,0] 
        a[:,3] = self.pts2[:,1]*self.pts1[:,0]
        a[:,4] = self.pts2[:,1]*self.pts1[:,1]
        a[:,5] = self.pts2[:,1]
        a[:,6] = self.pts1[:,0]
        a[:,7] = self.pts1[:,1]
        a[:,8] = np.ones(totalpts)


        u, s, v = np.linalg.svd(a)
        f_1 = v.T[:,-1].reshape((3,3)) #rank not 2
    
        #enforce rank 2
        uf_1,sf_1,vf_1 = np.linalg.svd(f_1) 
        
        s = np.diag([sf_1[0],sf_1[1],0]) #set diagonal 
        
        #recompute F
        f_1 = np.dot(np.dot(uf_1,s),vf_1)
        
        f_1 = refineF(f_1,self.pts1,self.pts2)
        
        #un-normalize
        T = np.diag([1.0/M,1.0/M,1.0])
        
        F = np.dot(np.dot(T.T,f_1),T)
        self.F = F        
        return self.F
    
    def estimate_E(self, F):
        self.E = np.dot(np.dot(self.K2.T,F),self.K1)
        return self.E
    
    
    def triangulate(self, pts1, pts2):
        # import pdb; pdb.set_trace()
        totalpts = pts1.shape[0]
        p3d = np.zeros((totalpts,3))
        a = np.zeros((4,4))
        for i in range(totalpts):
            a[0,:] = pts1[i,0]*self.C1[2,:] - self.C1[0,:]
            a[1,:] = pts1[i,1]*self.C1[2,:] - self.C1[1,:]
            a[2,:] = pts2[i,0]*self.C2[2,:] - self.C2[0,:]
            a[3,:] = pts2[i,1]*self.C2[2,:] - self.C2[1,:]
            
            u,s,v = np.linalg.svd(a)
            p = v.T[:,-1]
            p3d[i, :] = np.divide(p[0:3],p[3])
            
        w = p3d
        p3d_h = np.append(p3d, np.ones((p3d.shape[0],1)),axis =  1)
        p3d_h = p3d_h.T # obtained points
        
        
        p1_reprojection = np.dot(self.C1,p3d_h) #reprojected points
        p2_reprojection = np.dot(self.C2,p3d_h)
        
        p1_norm = np.zeros((2,totalpts))
        p2_norm = np.zeros((2,totalpts))
        
        p1_norm[0,:] = p1_reprojection[0,:]/p1_reprojection[-1]
        p1_norm[1,:] = p1_reprojection[1,:]/p1_reprojection[-1]
        
        p2_norm[0,:] = p2_reprojection[0,:]/p2_reprojection[-1]
        p2_norm[1,:] = p2_reprojection[1,:]/p2_reprojection[-1]
        
        p1_norm = p1_norm.T
        p2_norm = p2_norm.T
        
        error_pts1 = p1_norm - pts1
        error_x1 = error_pts1[:,0]
        error_y1 = error_pts1[:,1]
        
        error_pts2 = p2_norm - pts2
        error_x2 = error_pts2[:,0]
        error_y2 = error_pts2[:,1]
        
        err = np.sum(pow(error_x1,2) + pow(error_y1,2)) + np.sum(pow(error_x2,2) + pow(error_y2,2))
       
        
        return w, err
    
    def findBestM2(self, pts1, pts2):
    
        M1 = np.zeros((3,4))
        M1[0,0] = 1
        M1[1,1] = 1
        M1[2,2] = 1
    
        M2s = camera2(self.E)
    
    
        self.C1 = np.dot(self.K1, M1)
    
        min_err = math.inf
        C2_min = 0
        w_min = 0
        index_min = 0
    
        for i in range(M2s.shape[1]):
            self.C2 = np.dot(self.K2, M2s[:,:,i])
            w, err = self.triangulate(pts1, pts2)#.C1, self.pts1,C2, self.pts2)
            if err < min_err:
                min_err = err
                index_min = i
                C2_min = self.C2
                w_min = w
               # print(min_err)

        self.C2 = C2_min
        return M2s[:,:,index_min], self.C2, w_min      


    def epi_corr(self, im1, im2, x1, y1):
    
        search = 10
        
        point = np.array([[x1],[y1],[1]])
        epi_line = np.dot(self.F,point)
        
        epi_line = epi_line / np.linalg.norm(epi_line)
        a = epi_line[0]
        b = epi_line[1]
        c = epi_line[2]
        
        patch1 = im1[int(y1-search+1):int(y1+search+1),int(x1-search+1):int(x1+search+1),:]

        
        mindis = math.inf 

        for i in range(int(y1 - search + 1),int(y1 + search + 1)):
            
            x2_now = ((-b)*i-c)/a
            x2_now = x2_now.astype(int)
            
            if (x2_now - search+1) > 0 and (x2_now + search+1) < im2.shape[1] and (i - search+1) > 0 and (i + search + 1) <= im2.shape[0]:
                
                patch2 = im2[int(i-search+1):int(i + search+1),int(x2_now -search+1):int(x2_now +search+1)]
                dis = patch1 - patch2
                weighted_dis = gaussian_filter(dis, sigma = 1.0) 
                
                error = math.sqrt(np.sum(pow(weighted_dis,2)))
                
                if error < mindis:
                    mindis = error
                    x2 = float(x2_now)
                    y2 = float(np.array([i]))

        return x2, y2
    
    
    def visualize3D(self, totalpts, x2, y2, x1, y1, im1, im2):
        # import pdb; pdb.set_trace()
        for i in range(totalpts):
            x2[i], y2[i] = self.epi_corr(im1, im2, x1[i], y1[i])
        points1 = np.append(x1, y1, axis=1)
        points2 = np.append(x2, y2, axis=1)
    
        M1 = np.zeros((3,4))
        M1[0,0] = 1
        M1[1,1] = 1
        M1[2,2] = 1
    
        M2s = camera2(self.E)
    
        self.C1 = np.dot(self.K1, M1)
    
        min_error = math.inf
        min_P = 0
    
    
        for i in range(4):
            self.C2 = np.dot(self.K2, M2s[:, :, i])
            P, error = self.triangulate(points1, points2)#C1, points1, C2, points2)
            if error < min_error:
                if np.min(P[:, 2] >= 0):
                    M2, self.C2, w = self.findBestM2(points1, points2)
                    min_P = P
                    min_error = error
            
            
            P = min_P
    
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        xmin, xmax = np.min(P[:, 0]), np.max(P[:, 0])
        ymin, ymax = np.min(P[:, 1]), np.max(P[:, 1])
        zmin, zmax = np.min(P[:, 2]), np.max(P[:, 2])
    
        ax.set_xlim3d(xmin, xmax)
        ax.set_ylim3d(ymin, ymax)
        ax.set_zlim3d(zmin, zmax)
    
        ax.scatter(P[:, 0], P[:, 1], P[:, 2], c='b', marker='o')
        plt.show(block=True)

     