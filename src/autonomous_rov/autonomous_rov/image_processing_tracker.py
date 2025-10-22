#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import Image  # Update to sensor_msgs.msg.Image if using ROS2 standard image message
import numpy as np
import cv2
from . import camera_parameters as cam
from cv_bridge import CvBridge

set_desired_point = False #click right button to allow
get_hsv =False #click left button to allow

mouseX, mouseY = 0, 0
hsv_value = [0, 0, 0]

# camera parameters
u0 = 480
v0 = 270
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
        
        self.pub_tracked_point = self.create_publisher(Float64MultiArray, '/tracked_point', 10)
        self.pub_tracked_area = self.create_publisher(Float64MultiArray, '/tracked_area', 10)

        self.subscription = self.create_subscription(
            Image,                    # Message type
            'camera/image',           # Topic (assumed topic name)
            self.cameracallback,      # Callback function
            1                         # Queue size (adjust if necessary)
        )
        self.subscription  # prevent unused variable warning

        self.bridge = CvBridge()  # CvBridge for converting ROS images to OpenCV format

        self.get_logger().info('Image processing node started')


    def cameracallback(self, image_data):
        # Initial try - geometrical detection
        # Get image data
        image_np = self.bridge.imgmsg_to_cv2(image_data, "bgr8")  # Convert ROS Image message to OpenCV image
        
        # Find countours with canny edges - might have gaps - maybe need to blur 
        edges = cv2.Canny(image_np, 50, 150)
        cv2.imshow("Edges", edges)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contoured_image = image_np.copy()

        # init default tracked point - will be the center of the object
        current_point = [0, 0]

        if contours:
            # sort contours by area (largest first)
            contours = sorted(contours, key=cv2.contourArea, reverse=True)

            for contour in contours:
                area = cv2.contourArea(contour)
                # if area < 8000:
                #     continue  # to ignore small noise need to define the value during tests

                perimeter = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
                sides_num = len(approx)

                # Rectangular shape needed -> 4 corners
                
                # and here if we detected an aruco inside the contour then it is definetely a box

                if sides_num == 4:
                    x, y, w, h = cv2.boundingRect(approx)
                    # (x,y) - the top-left coordinate of the rectangle and (w,h) - its width and height

                    if h != 0:
                        aspect_ratio = float(w) / h
                    else:
                        aspect_ratio = 0

                    # Filter roughly rectangular shapes (not too elongated or square)
                    if 0.5 < aspect_ratio < 3.0 and area > 100: # values were 0.5 and 3.0 and 15000
                        # Draw detected rectangle
                        cv2.drawContours(contoured_image, [approx], -1, (0, 255, 0), 2)
                        cx = x + w // 2
                        cy = y + h // 2
                        current_point = [cx, cy]

                        cv2.circle(contoured_image, (cx, cy), 5, (255, 255, 0), -1)
                        cv2.putText(contoured_image, f"A:{int(area)} AR:{aspect_ratio:.2f}", 
                                    (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                        self.get_logger().info(
                            f"Rectangle: Area={area:.1f}, AR={aspect_ratio:.2f}, Corners={sides_num}"
                        )

                        # break if only one main rectangle (object) is expected
                        # might leave it like that ? need to test
                        break

                # width calculation 
                # pts = largest_contour.reshape(-1, 2)
                # x_vals = pts[:, 0]
                # y_vals = pts[:, 1]

                # min_x, max_x = np.min(x_vals), np.max(x_vals)
                # min_y, max_y = np.min(y_vals), np.max(y_vals)

                # width = max_x - min_x
                # height = max_y - min_y

                # # Draw horizontal width line
                # # cv2.line(contoured_image, (min_x, cy), (max_x, cy), (255, 0, 0), 2)
                # mid_x = (min_x + max_x) // 2
                # cv2.putText(contoured_image, f"W: {width}px", (mid_x, cy - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                # # Draw vertical height line
                # # cv2.line(contoured_image, (cx, min_y), (cx, max_y), (0, 0, 255), 2)
                # mid_y = (min_y + max_y) // 2
                # cv2.putText(contoured_image, f"H: {height}px", (cx + 10, mid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                # object_area = width * height
                # self.get_logger().info(f"Detected area has {object_area}")

                # if object_area < 60000 and object_area >= 23000:

                #     self.get_logger().info(f"Area after the check has height {height} and width {width}")

                #     #  boat
                #     cv2.drawContours(contoured_image, [largest_contour], -1, (0, 255, 0), 2)  # green contour
                #     cv2.circle(contoured_image, (cx, cy), 5, (255, 255, 0), -1)  # tracked point itself
                #     overlay_points(contoured_image, current_point, 0, 255, 0, 'Tracked boat')


                
        # Convert point to meters and publish
        current_point_meter = cam.convertOnePoint2meter(current_point)
        current_point_msg = Float64MultiArray(data = current_point_meter)
        self.pub_tracked_point.publish(current_point_msg)


        # Display the image
        cv2.imshow("Black Box tracking (hopefully please)", contoured_image)
        

        cv2.waitKey(2)


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