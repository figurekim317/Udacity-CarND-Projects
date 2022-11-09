import numpy as np
import cv2
from traitlets.traitlets import default


class LaneLineFinder:
    def __init__(self, warped_binary, my=30/720, mx=3.7/700):
        self.__my=my
        self.__mx=mx

        self.__sliwin_image, leftx, lefty, rightx, righty = self.__find_lane_pixels(warped_binary)
        self.ploty, left_fit, right_fit = self.__fit_polynomial_pix(warped_binary.shape, leftx, lefty, rightx, righty)
        self.left_fitx, self.right_fitx = self.__get_graph_pix(self.ploty, left_fit, right_fit)

        self.__left_curverad, self.__right_curverad = self.__measure_curvature_real(self.ploty, left_fit, right_fit)
        self.__cte = self.__measure_cte_real(self.left_fitx, self.right_fitx)
        self.lane_image = self.__visualize(warped_binary, self.ploty, self.left_fitx, self.right_fitx, lefty, leftx, righty, rightx)

    def get_sliwin_image(self):
        return self.__sliwin_image
    
    def get_curvature_real(self):
        return self.__left_curverad, self.__right_curverad

    def get_cte_real(self):
        return self.__cte

    def get_lane_image(self):
        return self.lane_image

    def plot_lane_graph(self, plt):
        plt.plot(self.left_fitx, self.ploty, color='yellow')
        plt.plot(self.right_fitx, self.ploty, color='yellow')

    def __find_lane_pixels(self, warped_binary):
        # Take a histogram of the bottom half of the image
        histogram = np.sum(warped_binary[warped_binary.shape[0]//2:,:], axis=0)
        # Create an output RGB image to draw on and visualize the result
        out_img = np.dstack((warped_binary, warped_binary, warped_binary))
        # Find the peak of the left and right halves of the histogram
        # These will be the starting point for the left and right lines
        midpoint = int(histogram.shape[0]//2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        # HYPERPARAMETERS
        # Choose the number of sliding windows
        nwindows = 9
        # Set the width of the windows +/- margin
        margin = 100
        # Set minimum number of pixels found to recenter window
        minpix = 50

        # Set height of windows - based on nwindows above and image shape
        window_height = int(warped_binary.shape[0]//nwindows)
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = warped_binary.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Current positions to be updated later for each window in nwindows
        leftx_current = leftx_base
        rightx_current = rightx_base

        # Create empty lists to receive left and right lane pixel indices
        left_lane_inds = []
        right_lane_inds = []

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = warped_binary.shape[0] - (window+1)*window_height
            win_y_high = warped_binary.shape[0] - window*window_height
            ### TO-DO: Find the four below boundaries of the window ###
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            
            # Draw the windows on the visualization image
            cv2.rectangle(out_img,(win_xleft_low,win_y_low),
            (win_xleft_high,win_y_high),(0,255,0), 2) 
            cv2.rectangle(out_img,(win_xright_low,win_y_low),
            (win_xright_high,win_y_high),(0,255,0), 2) 
            
            # Identify the x-position of nonzero pixels in x and y within the window #
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
            
            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:        
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

        # Concatenate the arrays of indices (previously was a list of lists of pixels)
        try:
            left_lane_inds = np.concatenate(left_lane_inds)
            right_lane_inds = np.concatenate(right_lane_inds)
        except ValueError:
            # Avoids an error if the above is not implemented fully
            pass

        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        ## Visualization ##
        # Colors in the left and right lane regions
        out_img[lefty, leftx] = [255, 0, 0]
        out_img[righty, rightx] = [0, 0, 255]

        return out_img, leftx, lefty, rightx, righty

    def __fit_polynomial_pix(self, imshape, leftx, lefty, rightx, righty):
        # Find our lane pixels first
        
        # Fit a second order polynomial to each using `np.polyfit`
        # Fit a polynomial in pixel
        left_fit_pix = np.polyfit(lefty, leftx, 2)
        right_fit_pix = np.polyfit(righty, rightx, 2)

        # Generate x and y pixel values for plotting
        ploty_pix = np.linspace(0, imshape[0]-1, imshape[0])
        
        return ploty_pix, left_fit_pix, right_fit_pix

    def __get_graph_pix(self, ploty, left_fit, right_fit):
        try:
            left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
            right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
        except TypeError:
            # Avoids an error if `left` and `right_fit` are still none or incorrect
            print('The function failed to fit a line!')
            left_fitx = 1*ploty**2 + 1*ploty
            right_fitx = 1*ploty**2 + 1*ploty

        # Plots the left and right polynomials on the lane lines
        # plt.plot(left_fitx, ploty, color='yellow')
        # plt.plot(right_fitx, ploty, color='yellow')

        return left_fitx, right_fitx

    def __measure_curvature_real(self, ploty_pix, left_fit_pix, right_fit_pix):

        '''
        Calculates the curvature of polynomial functions in meters.
        '''
        (my, mx) = (self.__my, self.__mx)
        # Converting pixel to meter
        left_fit_m = np.zeros_like(left_fit_pix)
        left_fit_m[0] = left_fit_pix[0]*mx/(my**2)
        left_fit_m[1] = left_fit_pix[1]*mx/my
        left_fit_m[2] = left_fit_pix[2]*mx
        right_fit_m = np.zeros_like(right_fit_pix)
        right_fit_m[0] = right_fit_pix[0]*mx/(my**2)
        right_fit_m[1] = right_fit_pix[1]*mx/my
        right_fit_m[2] = right_fit_pix[2]*mx
        # Define y-value where we want radius of curvature
        # We'll choose the maximum y-value, corresponding to the bottom of the image
        # ploty_pix is pixel value. Converting this to meter unit.
        y_eval = np.max(ploty_pix)*my
        
        # Calculation of R_curve (radius of curvature)
        left_curverad = ((1 + (2*left_fit_m[0]*y_eval + left_fit_m[1])**2)**1.5) / np.absolute(2*left_fit_m[0])
        right_curverad = ((1 + (2*right_fit_m[0]*y_eval + right_fit_m[1])**2)**1.5) / np.absolute(2*right_fit_m[0])

        return left_curverad, right_curverad

    def __measure_cte_real(self, left_fitx_pix, right_fitx_pix):
        '''
        Calculates CTE
        '''
        center_pos = (left_fitx_pix[-1]+right_fitx_pix[-1])/2
        cte = (center_pos - 1280/2) * self.__mx

        return cte

    def __visualize(self, warped_binary, ploty, left_fitx, right_fitx, lefty, leftx, righty, rightx):
        # Create blank image
        warped_blank = np.zeros_like(warped_binary).astype(np.uint8)
        warped_lane_image = np.dstack((warped_blank, warped_blank, warped_blank))

        # Fill the road between left and right lane
        left_lane_poly = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        right_lane_poly = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        lane_poly = np.hstack((left_lane_poly, right_lane_poly))
        cv2.fillPoly(warped_lane_image, np.int_(lane_poly), (0,255,0))

        # Draw the left and right lanes
        warped_lane_image[lefty, leftx] = [0, 0, 255]
        warped_lane_image[righty, rightx] = [255, 0, 0]

        return warped_lane_image
