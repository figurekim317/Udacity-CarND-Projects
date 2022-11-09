import numpy as np
import cv2

class ColorThreshold:
    def __init__(self, white_s_thresh=(0,255), white_b_thresh=(0,255),
                        yellow_s_thresh=(0,255), yellow_b_thresh=(0,255)):
        self.white_s_thresh=white_s_thresh
        self.white_b_thresh=white_b_thresh
        self.yellow_s_thresh=yellow_s_thresh
        self.yellow_b_thresh=yellow_b_thresh

    def set_white_thresh(self, s_thresh, b_thresh):
        self.white_s_thresh=s_thresh
        self.white_b_thresh=b_thresh

    def set_yellow_thresh(self, s_thresh, b_thresh):
        self.yellow_s_thresh=s_thresh
        self.yellow_b_thresh=b_thresh

        # Use exclusive lower bound (>) and inclusive upper (<=)

    def hls_select_white(self, img):
        return self.__hls_select(img, self.white_s_thresh, self.white_b_thresh)

    def hls_select_yellow(self, img):
        return self.__hls_select(img, self.yellow_s_thresh, self.yellow_b_thresh)
    
    def __hls_select(self, img, s_thresh, b_thresh):
        # 1) Convert to HLS color space
        hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        # 2) Apply a threshold to the S channel
        b_channel = hls[:,:,1]
        s_channel = hls[:,:,2]
        
        # 3) Return a binary image of threshold result
        binary_output = np.zeros_like(s_channel)
        binary_output[((s_channel > s_thresh[0]) & (s_channel <= s_thresh[1])) & 
                    ((b_channel > b_thresh[0]) & (b_channel <= b_thresh[1]))] = 1

        return binary_output

    def stack(self, binary1, binary2):
        # Stack each channel
        color_binary = np.zeros_like(binary1)
        color_binary[(binary1 == 1) | (binary2 == 1)] = 1
        return color_binary