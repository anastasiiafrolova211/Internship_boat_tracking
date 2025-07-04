#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import cv2
from . import camera_parameters as cam
from cv_bridge import CvBridge  # Can be removed if not using ROS messages anymore

def on_trackbar_change(x):
    pass

cv2.namedWindow('Result')

cv2.createTrackbar('Hue_Lower', 'Result', 0, 179, on_trackbar_change)
cv2.createTrackbar('Hue_Upper', 'Result', 30, 179, on_trackbar_change)
cv2.createTrackbar('Saturation_Lower', 'Result', 100, 255, on_trackbar_change)
cv2.createTrackbar('Saturation_Upper', 'Result', 255, 255, on_trackbar_change)
cv2.createTrackbar('Value_Lower', 'Result', 100, 255, on_trackbar_change)
cv2.createTrackbar('Value_Upper', 'Result', 255, 255, on_trackbar_change)

set_desired_point = False
get_hsv = False
mouseX, mouseY = 0, 0
hsv_value = [0, 0, 0]

u0, v0, lx, ly = 320, 240, 455, 455

def convert2meter(pt, u0, v0, lx, ly):
    return (float(pt[0]) - u0) / lx, (float(pt[1]) - v0) / ly

def convertOnePoint2meter(pt):
    return (float(pt[0]) - u0) / lx, (float(pt[1]) - v0) / ly

def overlay_points(image, pt, r, g, b, text="", scale=1, offsetx=5, offsety=5):
    cv2.circle(image, (int(pt[0]), int(pt[1])), int(4*scale+1), (b, g, r), -1)
    position = (int(pt[0]) + offsetx, int(pt[1]) + offsety)
    cv2.putText(image, text, position, cv2.FONT_HERSHEY_SIMPLEX, scale, (b, g, r, 255), 1)

def click_detect(event, x, y, flags, param):
    global get_hsv, set_desired_point, mouseX, mouseY
    if event == cv2.EVENT_LBUTTONDOWN:
        get_hsv = True
        mouseX, mouseY = x, y
    if event == cv2.EVENT_RBUTTONDOWN:
        set_desired_point = True
        mouseX, mouseY = x, y

class ImageProcessingNode(Node):
    def __init__(self, video_path):
        super().__init__('image_processing_node')
        self.pub_tracked_point = self.create_publisher(Float64MultiArray, 'tracked_point', 10)
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video: {video_path}")
        cv2.namedWindow("image")
        cv2.setMouseCallback("image", click_detect)
        self.get_logger().info(f"Processing video: {video_path}")

    def run_video_loop(self):
        global get_hsv, set_desired_point, mouseX, mouseY, hsv_value

        while self.cap.isOpened():
            ret, image_np = self.cap.read()
            if not ret:
                self.get_logger().info("Video ended or cannot fetch frame.")
                break

            hsv = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)
            lower_black = np.array([0, 0, 0])
            upper_black = np.array([180, 255, 50])
            mask = cv2.inRange(hsv, lower_black, upper_black)
            cv2.imshow("Mask", mask)
            contoured_image = image_np.copy()
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            current_point = [0, 0]

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    current_point = [cx, cy]
                    area = cv2.contourArea(largest_contour)
                    perimeter = cv2.arcLength(largest_contour, True)
                    if area < 60000 and area >= 15000:
                        self.get_logger().info(f"Area is {area}, perimeter is {perimeter}")
                        cv2.drawContours(contoured_image, [largest_contour], -1, (0, 255, 0), 2)
                        cv2.circle(contoured_image, (cx, cy), 5, (255, 255, 0), -1)
                        overlay_points(contoured_image, current_point, 0, 255, 0, 'Tracked boat')

            current_point_meter = convertOnePoint2meter(current_point)
            current_point_msg = Float64MultiArray(data=current_point_meter)
            self.pub_tracked_point.publish(current_point_msg)

            cv2.imshow("image", contoured_image)
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    video_path = "/home/mir/ros2_ws/videos/NoLight.avi"  # <<== Replace with your video path
    node = ImageProcessingNode(video_path)
    
    try:
        node.run_video_loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
