## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2

def rgb_to_gray(rgb, show):
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    if show:
        cv2.imshow("Gray", gray)
    return gray

def to_hsv(source):
    return cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

def green_mask(upper,lower,source):
    return cv2.inRange(hsv, greenLower, greenUpper)

def get_contours(binary):
    contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours


def draw_contours(image, contours, image_name):
    index = -1  # means all contours [0....n]
    thickness = 2
    color = (255, 0, 255)
    cv2.drawContours(image, contours, index, color, thickness)
    cv2.imshow(image_name, image)


def process_contours(binary, rgb, contours):
    black_image = np.zeros([binary.shape[0], binary.shape[1], 3], dtype=np.uint8)
    # for c in contours:
    #     area = cv2.contourArea(c)
    #     perimeter = cv2.arcLength(c, True)
    #     ((x, y), radius) = cv2.minEnclosingCircle(c)
    #     cv2.drawContours(black_image, [c], -1, (150, 250, 150), 1)
    #     cx, cy = get_contour_center(c)
    #     cv2.circle(rgb, (cx, cy), int(radius), (0, 0, 255), 1)
    #     cv2.circle(black_image, (cx, cy), int(radius), (0, 0, 255), 1)
    
    if len(contours) > 1:

        c1 = max(contours, key = cv2.contourArea)
        
        c2 = min(contours, key = cv2.contourArea)

        n = len(contours)
        for i in range(2,n):
            current = cv2.contourArea(contours[i])
            c1_area = cv2.contourArea(c1)
            c2_area = cv2.contourArea(c2)
            if current>c1_area:
                c2=c1
                c1=contours[i]
            elif current>c2_area and c1_area != current:
                c2=contours[i]

        x1,y1,w1,h1 = cv2.boundingRect(c1)
        x2,y2,w2,h2 = cv2.boundingRect(c2)
        # draw the biggest contour (c) in green
        cv2.rectangle(black_image,(x1,y1),(x1+w1,y1+h1),(0,255,0),2)
        cv2.rectangle(black_image,(x2,y2),(x2+w2,y2+h2),(0,255,0),2)

        cx1, cy1 = get_contour_center(c1)
        cv2.circle(black_image, (cx1, cy1), 1, (0, 0, 255), 1)
        cx2, cy2 = get_contour_center(c2)
        cv2.circle(black_image, (cx2, cy2), 1, (0, 0, 255), 1)

        cv2.circle(black_image, ((cx1+cx2)//2, (cy2+cy2)//2), 2, (0, 255, 0), 1)

        cv2.imshow("Black Image Contours", black_image)


def get_contour_center(contour):
    m = cv2.moments(contour)
    cx = -1
    cy = -1
    if m['m00'] != 0:
        cx = int(m['m10'] / m['m00'])
        cy = int(m['m01'] / m['m00'])
    return cx, cy

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

#greenLower = (60, 100, 0)
#greenUpper = (90, 255, 255)


greenLower = np.array([65,60,60])
greenUpper = np.array([80,255,255])

kernel = np.ones((5,5),np.uint8)
try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        hsv = to_hsv(color_image)
        blurred = cv2.GaussianBlur(hsv, (7, 7), 0)
        mask = cv2.inRange(blurred, greenLower, greenUpper)
        erodedimage = cv2.erode(mask,kernel,iterations = 5)
        contours = get_contours(erodedimage)
        process_contours(mask, color_image, contours)


        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', mask)
        cv2.waitKey(1)

finally:

    # Stop streaming
    pipeline.stop()