import cv2
from realsense_camera import *
from mask_rcnn import *

# Load Realsense camera and Mask R-CNN
rs = RealsenseCamera()
mrcnn = MaskRCNN()