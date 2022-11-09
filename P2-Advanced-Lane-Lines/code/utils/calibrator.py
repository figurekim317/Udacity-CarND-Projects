import numpy as np
import cv2
import glob

class Calibrator:
    def __init__(self, file_path):
        self.mtx, self.dist = self.get_undistort_coefficient(file_path)

    def get_undistort_coefficient(self, file_path):
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((6*9,3), np.float32)
        objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d points in real world space
        imgpoints = [] # 2d points in image plane.

        # Make a list of calibration images
        images = glob.glob(file_path)
        calibration_image = cv2.imread(images[0])
        calibration_imshape = calibration_image.shape

        # Step through the list and search for chessboard corners
        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            # Find the chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, (9,6),None)

            # If found, add object points, image points
            if ret == True:
                objpoints.append(objp)
                imgpoints.append(corners)

                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (9,6), corners, ret)

        # Get camera matrix adn distorion coefficient
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, calibration_imshape[1::-1], None, None)

        return mtx, dist

    def undistort(self, img):
        return cv2.undistort(img, self.mtx, self.dist, None)