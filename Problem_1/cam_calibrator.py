#!/usr/bin/python

import time
import os

import cv2
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib import cm
import numpy as np
import glob

import pdb

from camera_calibration.calibrator import MonoCalibrator, ChessboardInfo, Patterns

class CameraCalibrator:

    def __init__(self):
        self.calib_flags = 0
        self.pattern = Patterns.Chessboard

    def loadImages(self, cal_img_path, name, n_corners, square_length, n_disp_img=1e5, display_flag=True):
        self.name = name
        self.cal_img_path = cal_img_path

        self.boards = []
        self.boards.append(ChessboardInfo(n_corners[0], n_corners[1], float(square_length)))
        self.c = MonoCalibrator(self.boards, self.calib_flags, self.pattern)

        if display_flag:
            fig = plt.figure('Corner Extraction', figsize=(12, 5))
            gs = gridspec.GridSpec(1, 2)
            gs.update(wspace=0.025, hspace=0.05)

        for i, file in enumerate(sorted(os.listdir(self.cal_img_path))):
            img = cv2.imread(self.cal_img_path + '/' + file, 0)     # Load the image
            img_msg = self.c.br.cv2_to_imgmsg(img, 'mono8')         # Convert to ROS Image msg
            drawable = self.c.handle_msg(img_msg)                   # Extract chessboard corners using ROS camera_calibration package

            if display_flag and i < n_disp_img:
                ax = plt.subplot(gs[0, 0])
                plt.imshow(img, cmap='gray')
                plt.axis('off')

                ax = plt.subplot(gs[0, 1])
                plt.imshow(drawable.scrib)
                plt.axis('off')

                plt.subplots_adjust(left=0.02, right=0.98, top=0.98, bottom=0.02)
                fig.canvas.set_window_title('Corner Extraction (Chessboard {0})'.format(i+1))

                plt.show(block=False)
                plt.waitforbuttonpress()

        # Useful parameters
        self.d_square = square_length                             # Length of a chessboard square
        self.h_pixels, self.w_pixels = img.shape                  # Image pixel dimensions
        self.n_chessboards = len(self.c.good_corners)             # Number of examined images
        self.n_corners_y, self.n_corners_x = n_corners            # Dimensions of extracted corner grid
        self.n_corners_per_chessboard = n_corners[0]*n_corners[1]


    def genCornerCoordinates(self, u_meas, v_meas):
        '''
        Inputs:
            u_meas: a list of arrays where each array are the u values for each board.
            v_meas: a list of arrays where each array are the v values for each board.
        Output:
            corner_coordinates: a tuple (Xg, Yg) where Xg/Yg is a list of arrays where each array are the x/y values for each board.

        HINT: u_meas, v_meas starts at the blue end, and finishes with the pink end
        HINT: our solution does not use the u_meas and v_meas values 
        HINT: it does not matter where your frame it, as long as you are consistent! 
        '''

        ########## Code starts here ##########
        length = self.d_square
        nrows  = self.n_corners_y
        ncols  = self.n_corners_x
        nboards = self.n_chessboards

        Xg = np.zeros(0)
        Yg = np.zeros(0)

        for j in range(nrows):
            for i in range(ncols):
                xtemp = length*i
                ytemp = length*j

                Xg = np.append(Xg, xtemp)
                Yg = np.append(Yg, ytemp)

        X_board = []
        Y_board = []
        for i in range(nboards):
            X_board.append(Xg)
            Y_board.append(Yg)

        corner_coordinates = (X_board, Y_board)

        ########## Code ends here ##########

        return corner_coordinates


    def estimateHomography(self, u_meas, v_meas, X, Y):    # Zhang Appendix A
        '''
        Inputs:
            u_meas: an array of the u values for a board.
            v_meas: an array of the v values for a board.
            X: an array of the X values for a board. (from genCornerCoordinates)
            Y: an array of the Y values for a board. (from genCornerCoordinates)
        Output:
            H: the homography matrix. its size is 3x3
        
        HINT: What is the size of the matrix L?
        HINT: What are the outputs of the np.linalg.svd function? Based on this, where does the eigenvector corresponding to the smallest eigen value live?
        HINT: np.stack and/or np.hstack may come in handy here.
        '''
        ########## Code starts here ##########

        n = len(X)

        # create M matrix
        M = np.vstack( (X,Y,np.ones(n)) )
        M_T = np.transpose(M)

        # initialize L matrix
        L = np.zeros((2*n,9))

        # make parameters that are used in L matrix
        zero_T = np.zeros((n,3))

        u_meas_mat = np.column_stack( (u_meas,u_meas,u_meas) )
        v_meas_mat = np.column_stack( (v_meas,v_meas,v_meas) )

        u_M_T = -1 * np.multiply(u_meas_mat,M_T) #-1 * u_meas * M_T
        v_M_T = -1 * np.multiply(v_meas_mat,M_T) #-1 * v_meas * M_T

        # create L matrix
        L_row_1 = np.column_stack( (M_T, zero_T, u_M_T) )
        L_row_2 = np.column_stack( (zero_T, M_T, v_M_T) )

        L = np.vstack( (L_row_1, L_row_2) )

        # Use SVD to find the smallest norm of Lx
        U, s, V = np.linalg.svd(L, full_matrices = False)

        # obtain solution vector, which is rightmost singular vector
        V_sol = V[-1]

        # Create H
        H = np.zeros( (3,3) )
        H[0] = V_sol[0:3] #first row of H is the first 3 values of V_sol (first 3 values of V_sol are the transpose of H's first row)
        H[1] = V_sol[3:6]
        H[2] = V_sol[6:9]
        

        ########## Code ends here ##########
        return H

    
    def getVij(self, H, i, j):
        

        '''
        # Helper function
        # i,j are written like in the equation (i.e. not zero-indexed)
        '''
       

        # subtract for zero-indexing
        i = i-1;
        j = j-1;

        # take transpose of H, since hi1 in paper means element 1 of column i, or H[1,i] in 1-index
        H = np.transpose(H)

        Vij = np.array( [ H[i,0] * H[j,0], \
            H[i,0] * H[j,1] + H[i,1] * H[j,0], \
            H[i,1] * H[j,1], \
            H[i,2] * H[j,0] + H[i,0] * H[j,2], \
            H[i,2] * H[j,1] + H[i,1] * H[j,2], \
            H[i,2] * H[j,2]  ])

        return Vij

    def getIntrinsicsFromB(self, b):
        '''

        Helper function
        Determines A from b
        '''
        B11 = b[0]
        B12 = b[1]
        B22 = b[2]
        B13 = b[3]
        B23 = b[4]
        B33 = b[5]

        #determine the coefficients
        v0 = (B12 * B13 - B11 * B23) / (B11 * B22 - (B12**2))
        lam = B33 - ((B13**2) + v0*(B12 * B13 - B11 * B23) ) / B11
        alpha = np.sqrt(lam/B11)
        beta = np.sqrt(lam * B11 / (B11 * B22 - (B12**2)) )
        gamma = -1 * B12 * (alpha**2) * beta / lam
        u0 = gamma * v0 / beta - B13 * (alpha ** 2) / lam

        #create A from coefficients
        A = np.zeros((3, 3))
        A[0,0] = alpha
        A[0,1] = gamma
        A[0,2] = u0
        A[1,1] = beta
        A[1,2] = v0
        A[2,2] = 1

        return A

    def getCameraIntrinsics(self, H):    # Zhang 3.1, Appendix B
        '''
        Input:
            H: a list of homography matrices for each board
        Output:
            A: the camera intrinsic matrix

        HINT: MAKE SURE YOU READ SECTION 3.1 THOROUGHLY!!! V. IMPORTANT
        HINT: What is the definition of h_ij?  
        HINT: It might be cleaner to write an inner function (a function inside the getCameraIntrinsics function)
        HINT: What is the size of V?
        '''
        ########## Code starts here ##########

        # Initialize V
        V = np.empty( (0,6) )

        # Iterate through all H matrices provided
        for H_i in H:
            V12 = self.getVij(H_i, 1, 2)
            V11 = self.getVij(H_i, 1, 1)
            V22 = self.getVij(H_i, 2, 2)

            V = np.vstack( (V, V12, (V11 - V22)) )

        # Use SVD to find solution
        U, s, unitary = np.linalg.svd(V, full_matrices = False)

        # solution is right-most vector
        b = unitary[-1]

        A = self.getIntrinsicsFromB(b) # UPDATE ME

        ########## Code ends here ##########
        return A

    def getExtrinsics(self, H, A):    # Zhang 3.1, Appendix C
        '''
        Inputs:
            H: a single homography matrix
            A: the camera intrinsic matrix
        Outputs:
            R: the rotation matrix
            t: the translation vector
        '''
        ########## Code starts here ##########
        # UPDATE ME

        #gather parameters
        A_inv = np.linalg.inv(A)
        h1 = H[:,0]
        h2 = H[:,1]
        h3 = H[:,2]

        #calculate R and t values
        lam = 1/np.linalg.norm(A_inv * h1)
        r1 = lam * np.matmul(A_inv, h1)
        r2 = lam * np.matmul(A_inv, h2)
        r3 = np.cross(r1, r2)

        R = np.column_stack( (r1, r2, r3) )
        #print(R.shape)
        t = lam * np.matmul(A_inv, h3)

        #refine R
        U, s, V = np.linalg.svd(R, full_matrices=False)
        R = np.matmul(U,V) 
        

        ########## Code ends here ##########
        return R, t

    def transformWorld2NormImageUndist(self, X, Y, Z, R, t):    # Zhang 2.1, Eq. (1)
        '''
        Inputs:
            X, Y, Z: the world coordinates of the points for a given board. This is an array of 63 elements
                     X, Y come from genCornerCoordinates. Since the board is planar, we assume Z is an array of zeros.
            R, t: the camera extrinsic parameters (rotation matrix and translation vector) for a given board.
        Outputs:
            x, y: the coordinates in the ideal normalized image plane

        '''
        ########## Code starts here ##########

        #(R,t) relates the world coordinate system to the camera coordinate system
        R_mat = np.column_stack( (R, t) )
        M_tilde = np.row_stack( (X, Y, Z, np.ones(len(X))) ) # TODO: check

        cam_coords = np.matmul(R_mat,M_tilde)

        X_cam = cam_coords[0]
        Y_cam = cam_coords[1]
        Z_cam = cam_coords[2]


        x = X_cam / Z_cam
        y = Y_cam / Z_cam

        ########## Code ends here ##########
        return x, y

    def transformWorld2PixImageUndist(self, X, Y, Z, R, t, A):    # Zhang 2.1, Eq. (1)
        '''
        Inputs:
            X, Y, Z: the world coordinates of the points for a given board. This is an array of 63 elements
                     X, Y come from genCornerCoordinates. Since the board is planar, we assume Z is an array of zeros.
            A: the camera intrinsic parameters
            R, t: the camera extrinsic parameters (rotation matrix and translation vector) for a given board.
        Outputs:
            u, v: the coordinates in the ideal pixel image plane
        '''
        ########## Code starts here ##########

        # M_tilde is world coordinates
        M_tilde = np.row_stack( (X, Y, Z, np.ones( len(X) ) ) )
        R_mat = np.column_stack( (R, t) )

        cam_coords = np.matmul(R_mat, M_tilde)
        pix_coords = np.matmul(A, cam_coords)
        #pix_coords = np.dot( A, np.dot(R_mat, M_tilde) )#A * R_mat * M_tilde
        
        u = pix_coords[0] / pix_coords[2] #normalized
        v = pix_coords[1] / pix_coords[2] 
        ########## Code ends here ##########
        return u, v

    def transformWorld2NormImageDist(self, X, Y, R, t, k):    # Zhang 3.3
        '''
        Inputs:
            X, Y, Z: the world coordinates of the points for a given board. This is an array of 63 elements
                     X, Y come from genCornerCoordinates. Since the board is planar, we assume Z is an array of zeros.
            R, t: the camera extrinsic parameters (rotation matrix and translation vector) for a given board.
        Outputs:
            x_br, y_br: the real, normalized image coordinates         
        '''
        ########## Code starts here ##########        
                # UPDATE ME
        X_ideal, Y_ideal = self.transformWorld2NormImageUndist(X,Y,Z,R,t)
        k1 = k[0]
        k2 = k[1]

        x_br = X_ideal + X_ideal * (k1 * (X_ideal**2 + Y_ideal**2) + k2*(X_ideal**2 + Y_ideal**2)**2)
        y_br = Y_ideal + Y_ideal * (k1 * (X_ideal**2 + Y_ideal**2) + k2*(X_ideal**2 + Y_ideal**2)**2)
        ########## Code ends here ##########        
        return x_br, y_br

    def transformWorld2PixImageDist(self, X, Y, Z, R, t, A, k):    # Zhang 3.3
        '''
        Inputs:
            X, Y, Z: the world coordinates of the points for a given board. This is an array of 63 elements
                     X, Y come from genCornerCoordinates. Since the board is planar, we assume Z is an array of zeros.
            A: the camera intrinsic parameters                     
            R, t: the camera extrinsic parameters (rotation matrix and translation vector) for a given board.
        Outputs:
            u_br, v_br: the real, observed image coordinates         
        '''
        ########## Code starts here ##########                
        # UPDATE ME
        X_ideal, Y_ideal = self.transformWorld2NormImageUndist(X,Y,Z,R,t)
        u, v = self.transformWorld2PixImageUndist(X,Y,Z,R,t,A)
        k1 = k[0]
        k2 = k[1]
        u0 = A[0,2]
        v0 = A[1,2]

        u_br = u + (u-u0) * (k1 * (X_ideal**2 + Y_ideal**2) + k2 * (X_ideal**2 + Y_ideal**2)**2)
        v_br = v + (v-v0) * (k1 * (X_ideal**2 + Y_ideal**2) + k2 * (X_ideal**2 + Y_ideal**2)**2)
        ########## Code ends here ##########
        return u_br, v_br

    def undistortImages(self, A, k = np.zeros(2), n_disp_img=1e5, scale=0): 
        Anew_no_k, roi = cv2.getOptimalNewCameraMatrix(A, np.zeros(4), (self.w_pixels, self.h_pixels), scale)
        mapx_no_k, mapy_no_k = cv2.initUndistortRectifyMap(A, np.zeros(4), None, Anew_no_k, (self.w_pixels, self.h_pixels), cv2.CV_16SC2)
        Anew_w_k, roi = cv2.getOptimalNewCameraMatrix(A, np.hstack([k, 0, 0]), (self.w_pixels, self.h_pixels), scale)
        mapx_w_k, mapy_w_k = cv2.initUndistortRectifyMap(A, np.hstack([k, 0, 0]), None, Anew_w_k, (self.w_pixels, self.h_pixels), cv2.CV_16SC2)

        if k[0] != 0:
            n_plots = 3
        else:
            n_plots = 2

        fig = plt.figure('Image Correction', figsize=(6*n_plots, 5))
        gs = gridspec.GridSpec(1, n_plots)
        gs.update(wspace=0.025, hspace=0.05)

        for i, file in enumerate(sorted(os.listdir(self.cal_img_path))):
            if i < n_disp_img:
                img_dist = cv2.imread(self.cal_img_path + '/' + file, 0)
                img_undist_no_k = cv2.undistort(img_dist, A, np.zeros(4), None, Anew_no_k)
                img_undist_w_k = cv2.undistort(img_dist, A, np.hstack([k, 0, 0]), None, Anew_w_k)

                ax = plt.subplot(gs[0, 0])
                ax.imshow(img_dist, cmap='gray')
                ax.axis('off')

                ax = plt.subplot(gs[0, 1])
                ax.imshow(img_undist_no_k, cmap='gray')
                ax.axis('off')

                if k[0] != 0:
                    ax = plt.subplot(gs[0, 2])
                    ax.imshow(img_undist_w_k, cmap='gray')
                    ax.axis('off')

                plt.subplots_adjust(left=0.02, right=0.98, top=0.98, bottom=0.02)
                fig.canvas.set_window_title('Image Correction (Chessboard {0})'.format(i+1))

                plt.show(block=False)
                plt.waitforbuttonpress()

    def plotBoardPixImages(self, u_meas, v_meas, X, Y, R, t, A, n_disp_img=1e5, k=np.zeros(2)):         # Expects X, Y, R, t to be lists of arrays, just like u_meas, v_meas

        fig = plt.figure('Chessboard Projection to Pixel Image Frame', figsize=(8, 6))
        plt.clf()

        for p in range(min(self.n_chessboards, n_disp_img)):
            plt.clf()
            ax = plt.subplot(111)
            ax.plot(u_meas[p], v_meas[p], 'r+', label='Original')
            u, v = self.transformWorld2PixImageUndist(X[p], Y[p], np.zeros(X[p].size), R[p], t[p], A)
            ax.plot(u, v, 'b+', label='Linear Intrinsic Calibration')

            box = ax.get_position()
            ax.set_position([box.x0, box.y0 + box.height * 0.15, box.width, box.height*0.85])
            if k[0] != 0:
                u_br, v_br = self.transformWorld2PixImageDist(X[p], Y[p], np.zeros(X[p].size), R[p], t[p], A, k)
                ax.plot(u_br, v_br, 'g+', label='Radial Distortion Calibration')

            ax.axis([0, self.w_pixels, 0, self.h_pixels])
            plt.gca().set_aspect('equal', adjustable='box')
            plt.title('Chessboard {0}'.format(p+1))
            ax.legend(loc='lower center', bbox_to_anchor=(0.5, -0.3), fontsize='medium', fancybox=True, shadow=True)

            plt.show(block=False)
            plt.waitforbuttonpress()

    def plotBoardLocations(self, X, Y, R, t, n_disp_img=1e5):
        # Expects X, U, R, t to be lists of arrays, just like u_meas, v_meas

        ind_corners = [0, self.n_corners_x-1, self.n_corners_x*self.n_corners_y-1, self.n_corners_x*(self.n_corners_y-1), ]
        s_cam = 0.02
        d_cam = 0.05
        xyz_cam = [[0, -s_cam, s_cam, s_cam, -s_cam],
                   [0, -s_cam, -s_cam, s_cam, s_cam],
                   [0, -d_cam, -d_cam, -d_cam, -d_cam]]
        ind_cam = [[0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 4, 1]]
        verts_cam = []
        for i in range(len(ind_cam)):
            verts_cam.append([zip([xyz_cam[0][j] for j in ind_cam[i]],
                                  [xyz_cam[1][j] for j in ind_cam[i]],
                                  [xyz_cam[2][j] for j in ind_cam[i]])])

        fig = plt.figure('Estimated Chessboard Locations', figsize=(12, 5))
        axim = fig.add_subplot(121)
        ax3d = fig.add_subplot(122, projection='3d')

        boards = []
        verts = []
        for p in range(self.n_chessboards):

            M = []
            W = np.column_stack((R[p], t[p]))
            for i in range(4):
                M_tld = W.dot(np.array([X[p][ind_corners[i]], Y[p][ind_corners[i]], 0, 1]))
                if np.sign(M_tld[2]) == 1:
                    Rz = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
                    M_tld = Rz.dot(M_tld)
                    M_tld[2] *= -1
                M.append(M_tld[0:3])

            M = (np.array(M).T).tolist()
            verts.append([zip(M[0], M[1], M[2])])
            boards.append(Poly3DCollection(verts[p]))

        for i, file in enumerate(sorted(os.listdir(self.cal_img_path))):
            if i < n_disp_img:
                img = cv2.imread(self.cal_img_path + '/' + file, 0)
                axim.imshow(img, cmap='gray')
                axim.axis('off')

                ax3d.clear()

                for j in range(len(ind_cam)):
                    cam = Poly3DCollection(verts_cam[j])
                    cam.set_alpha(0.2)
                    cam.set_color('green')
                    ax3d.add_collection3d(cam)

                for p in range(self.n_chessboards):
                    if p == i:
                        boards[p].set_alpha(1.0)
                        boards[p].set_color('blue')
                    else:
                        boards[p].set_alpha(0.1)
                        boards[p].set_color('red')

                    ax3d.add_collection3d(boards[p])
                    ax3d.text(verts[p][0][0][0], verts[p][0][0][1], verts[p][0][0][2], '{0}'.format(p+1))
                    plt.show(block=False)

                view_max = 0.2
                ax3d.set_xlim(-view_max, view_max)
                ax3d.set_ylim(-view_max, view_max)
                ax3d.set_zlim(-2*view_max, 0)
                ax3d.set_xlabel('X axis')
                ax3d.set_ylabel('Y axis')
                ax3d.set_zlabel('Z axis')

                if i == 0:
                    ax3d.view_init(azim=90, elev=120)

                plt.tight_layout()
                fig.canvas.set_window_title('Estimated Board Locations (Chessboard {0})'.format(i+1))

                plt.show(block=False)

                raw_input('<Hit Enter To Continue>')

    def writeCalibrationYaml(self, A, k):
        self.c.intrinsics = np.array(A)
        self.c.distortion = np.hstack(([k[0], k[1]], np.zeros(3))).reshape((1, 5))
        #self.c.distortion = np.zeros(5)
        self.c.name = self.name
        self.c.R = np.eye(3)
        self.c.P = np.column_stack((np.eye(3), np.zeros(3)))
        self.c.size = [self.w_pixels, self.h_pixels]

        filename = self.name + '_calibration.yaml'
        with open(filename, 'w') as f:
            f.write(self.c.yaml())

        print('Calibration exported successfully to ' + filename)

    def getMeasuredPixImageCoord(self):
        u_meas = []
        v_meas = []
        for chessboards in self.c.good_corners:
            u_meas.append(chessboards[0][:, 0][:, 0])
            v_meas.append(self.h_pixels - chessboards[0][:, 0][:, 1])   # Flip Y-axis to traditional direction

        return u_meas, v_meas   # Lists of arrays (one per chessboard)

