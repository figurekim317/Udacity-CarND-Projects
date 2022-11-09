import numpy as np
import cv2

class PerspectiveTransformer:
    def __init__(self, imshape, corners, offset):
        self.imshape = imshape
        self.corners = corners
        self.offset = offset
        src = np.float32(self.corners)
        dst = np.float32([[self.offset,0],
                        [self.imshape[1]-self.offset,0],
                        [self.imshape[1]-self.offset,self.imshape[0]],
                        [self.offset,self.imshape[0]]])
    
        self.warp_perspective_M = cv2.getPerspectiveTransform(src, dst)
        self.unwarp_perspective_M = cv2.getPerspectiveTransform(dst, src)

    def warp_image(self, image):
        warped_image = cv2.warpPerspective(image, self.warp_perspective_M, (self.imshape[1], self.imshape[0]), flags=cv2.INTER_NEAREST)

        return warped_image

    def unwarp_image(self, image):
        unwarped_image = cv2.warpPerspective(image, self.unwarp_perspective_M, (self.imshape[1], self.imshape[0]), flags=cv2.INTER_NEAREST)

        return unwarped_image
