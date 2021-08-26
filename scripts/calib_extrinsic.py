import numpy as np
import cv2
import glob
from scipy.spatial.transform import Rotation as R

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 25, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
objp = objp*25

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

img = np.load('/home/ncslaber/109-2/210824_calibration/color_wall_2.npy')
cv2.imshow('img',img)
cv2.waitKey(0)

cv2.destroyAllWindows()

gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

# Find the chess board corners
ret, corners = cv2.findChessboardCorners(gray, (9,6),None)

    # If found, add object points, image points (after refining them)
if ret == True:
    print("here")

    objpoints.append(objp)

    corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
    imgpoints.append(corners2)

            # Draw and display the corners
    img = cv2.drawChessboardCorners(img, (9,6), corners2,ret)
    cv2.imshow('img',img)
    cv2.waitKey(0)

cv2.destroyAllWindows()


ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

rr, _ = cv2.Rodrigues(np.asarray(rvecs))
print(rr)
print(tvecs)

r = R.from_matrix(rr)
rpy = r.as_euler('zyx', degrees=True)
print(rpy)