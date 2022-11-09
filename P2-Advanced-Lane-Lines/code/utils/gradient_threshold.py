import numpy as np
import cv2

class GradientThreshold:
    def __init__(self, sobel_kernel=3, mag_thresh=(0, 255), dir_thresh=(0, np.pi/2)):
        self.sobel_kernel = sobel_kernel
        self.mag_thresh = mag_thresh
        self.dir_thresh = dir_thresh

    def set_sobel_kernel(self, sobel_kernel):
        self.sobel_kernel = sobel_kernel
    
    def set_mag_thresh(self, mag_thresh):
        self.mag_thresh = mag_thresh
    
    def set_dir_thresh(self, dir_thresh):
        self.dir_thresh = dir_thresh

    def grad_select(self, img):
        # Grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        # Calculate the x and y gradients
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=self.sobel_kernel)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=self.sobel_kernel)

        # Applying gradient magnitude threshold
        gradmag = np.sqrt(sobelx**2 + sobely**2)
        #    Rescale to 8 bit
        scale_factor = np.max(gradmag)/255 
        gradmag = (gradmag/scale_factor).astype(np.uint8)

        # Applying gradient direction threshold
        absgraddir = np.arctan2(np.absolute(sobely), np.absolute(sobelx))
        binary_output = np.zeros_like(absgraddir)

        # Create a binary image of ones where threshold is met, zeros otherwise
        binary_output = np.zeros_like(gray)
        binary_output[(gradmag >= self.mag_thresh[0]) & (gradmag <= self.mag_thresh[1]) & 
                      (absgraddir >= self.dir_thresh[0]) & (absgraddir <= self.dir_thresh[1])] = 1
        # Return the binary image
        return binary_output

