#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import Image  # Update to sensor_msgs.msg.Image if using ROS2 standard image message
import numpy as np
import cv2
from . import camera_parameters as cam
from cv_bridge import CvBridge


def on_trackbar_change(x):
    pass

cv2.namedWindow('Result')

# Create trackbars for threshold values
cv2.createTrackbar('Hue_Lower', 'Result', 0, 179, on_trackbar_change)
cv2.createTrackbar('Hue_Upper', 'Result', 30, 179, on_trackbar_change)
cv2.createTrackbar('Saturation_Lower', 'Result', 100, 255, on_trackbar_change)
cv2.createTrackbar('Saturation_Upper', 'Result', 255, 255, on_trackbar_change)
cv2.createTrackbar('Value_Lower', 'Result', 100, 255, on_trackbar_change)
cv2.createTrackbar('Value_Upper', 'Result', 255, 255, on_trackbar_change)
# Hue: 170 (ranges from 0 to 179 in OpenCV)
# Saturation: 155 (ranges from 0 to 255)
# Value: 160 (ranges from 0 to 255)
set_desired_point = False #click right button to allow
get_hsv =False #click left button to allow
#desired_point = [300, 200]
mouseX, mouseY = 0, 0
hsv_value = [0, 0, 0]
# camera parameters
u0 = 320
v0 = 240
lx = 455
ly = 455
kud =0.00683 
kdu = -0.01424     
    
# convert a pixel coordinate to meters given linear calibration parameters
def convert2meter(pt,u0,v0,lx,ly):
    return (float(pt[0])-u0)/lx, (float(pt[1])-v0)/ly

# convert a pixel coordinate to meters using defaut calibration parameters
def convertOnePoint2meter(pt):
    global u0,v0,lx, ly
    return (float(pt[0])-u0)/lx, (float(pt[1])-v0)/ly

# convert a list of pixels coordinates to meters using defaut calibration parameters
def convertListPoint2meter (points):
    global u0,v0,lx, ly
    
    if(np.shape(points)[0] > 1):
        n = int(np.shape(points)[0]/2)
        point_reshaped = (np.array(points).reshape(n,2))
        point_meter = []
        for pt in point_reshaped:
            pt_meter = convert2meter(pt,u0,v0,lx,ly)
            point_meter.append(pt_meter)
        point_meter = np.array(point_meter).reshape(-1)
        return point_meter
#end of camera parameters 
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
    def __init__(self):
        super().__init__('image_processing_node')
        
        self.pub_tracked_point = self.create_publisher(Float64MultiArray, 'tracked_point', 10)
        # # self.pub_desired_point = self.create_publisher(Float64MultiArray, 'desired_point', 10)
        # self.pub_heading = self.create_publisher(String, 'heading', 10)  
        # self.pub_heading_angle = self.create_publisher(Float64MultiArray, 'heading_angle', 10)  
        
        self.subscription = self.create_subscription(
            Image,                    # Message type
            'camera/image',           # Topic (assumed topic name)
            self.cameracallback,      # Callback function
            1                         # Queue size (adjust if necessary)
        )
        self.subscription  # prevent unused variable warning

        self.bridge = CvBridge()  # CvBridge for converting ROS images to OpenCV format

        cv2.namedWindow("image")  # Create the window before setting the mouse callback
        cv2.setMouseCallback("image", click_detect)
        self.get_logger().info('Image processing node started')


    # def cameracallback(self, image_data):
    #     image_np = self.bridge.imgmsg_to_cv2(image_data, "bgr8")  # Convert ROS Image message to OpenCV image
    #     hsv = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)

    #     # Define HSV range (adjust as needed for your object)
    #     lower_black = np.array([0, 0, 0])   # For black or dark objects
    #     upper_black = np.array([179, 255, 50])  # Fixed hue upper bound to 179 for OpenCV

    #     mask = cv2.inRange(hsv, lower_black, upper_black)

    #     # Find contours
    #     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #     current_point = [0, 0]

    #     if contours:
    #         # Get the largest contour
    #         largest_contour = max(contours, key=cv2.contourArea)

    #         # Draw contour around the object
    #         cv2.drawContours(image_np, [largest_contour], -1, (0, 255, 255), 2)  # Yellow contour

    #         # Calculate centroid
    #         M = cv2.moments(largest_contour)
    #         if M["m00"] != 0:
    #             cx = int(M["m10"] / M["m00"])
    #             cy = int(M["m01"] / M["m00"])
    #             current_point = [cx, cy]

    #             # Draw the centroid and overlay text
    #             cv2.circle(image_np, (cx, cy), 5, (0, 255, 0), -1)
    #             overlay_points(image_np, current_point, 0, 255, 0, 'Tracked Buoy')

    #     # Convert and publish the tracked point
    #     current_point_meter = cam.convertOnePoint2meter(current_point)
    #     current_point_msg = Float64MultiArray(data=current_point_meter)
    #     self.pub_tracked_point.publish(current_point_msg)

    #     # Show result
    #     cv2.imshow("Buoy Tracking", image_np)
    #     cv2.waitKey(2)


    def cameracallback(self, image_data):
        # get image data
        image_np = self.bridge.imgmsg_to_cv2(image_data, "bgr8")  # Convert ROS Image message to OpenCV image

        # ------------------------starting here------------------------
        # Convert to HSV color space - Hue Saturation Value
        hsv = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)

        # hue spectrum for black color
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])

        mask = cv2.inRange(hsv, lower_black, upper_black)
        cv2.imshow("Mask", mask)
        contoured_image = image_np.copy()

        # Find contours of black objects
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # init default tracked point
        current_point = [0, 0]
        heading = 'initial'

        if contours:
            # Find the largest black object
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                current_point = [cx, cy]

                #  boat
                cv2.drawContours(contoured_image, [largest_contour], -1, (0, 255, 0), 2)  # green contour
                cv2.circle(contoured_image, (cx, cy), 5, (255, 255, 0), -1)  # tracked point itself
                overlay_points(contoured_image, current_point, 0, 255, 0, 'Tracked boat')
                # in the last 2 lines i changed from image_np to contoured_image 

                # bounding box to check the heading (assume?)
                rect = cv2.minAreaRect(largest_contour)
                (rect_cx, rect_cy), (w, h), angle = rect
                box = cv2.boxPoints(rect)
                box = np.intp(box)
                #  boundingRect

                # Determine heading based on shape and centroid
                if w > h:
                    # Horizontal shape: left or right
                    if rect_cx < contoured_image.shape[1] // 2:
                        heading = "left"
                    else:
                        heading = "right"
                else:
                    angle += 90 # for publishing angle of heading in degrees 
                    heading_deg = -angle
                    # Vertical shape: forward or backward
                    if rect_cy < contoured_image.shape[0] // 2:
                        heading = "forward"
                    else:
                        heading = "backward"
                # Draw bounding box
                # cv2.rectangle(contoured_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.drawContours(contoured_image, [box], 0, (255, 0, 0), 2)
                # overlay_points(contoured_image, [x, y], 255, 0, 0, f"Heading: {heading}")
                overlay_points(contoured_image, [int(rect_cx), int(rect_cy)], 255, 0, 0, f"Heading: {heading} in {heading_deg:.1f}")    


        # Convert point to meters and publish
        current_point_meter = cam.convertOnePoint2meter(current_point)
        current_point_msg = Float64MultiArray(data = current_point_meter)
        self.pub_tracked_point.publish(current_point_msg)

        # heading_msg = String()
        # heading_msg.data = heading
        # self.pub_heading.publish(heading_msg)

        # heading_angle_msg = Float64MultiArray()
        # heading_angle_msg.data = heading_deg
        # self.pub_heading_angle.publish(heading_angle_msg)


        # Display the image
        cv2.imshow("Boat tracking", contoured_image)
        

        cv2.waitKey(2)
        # leave either like this or delete everything below and uncomment the previous line
        # key = cv2.waitKey(2)
        # if key == 27:  # ESC key
        #     rclpy.shutdown() # to stop the video display


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
