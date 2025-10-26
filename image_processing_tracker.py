#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import Image  # Update to sensor_msgs.msg.Image if using ROS2 standard image message
import numpy as np
import cv2
from . import camera_parameters as cam
from cv_bridge import CvBridge
from ultralytics import YOLO

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
    
# load model once for data recognition
model = YOLO("/home/mir/ros2_ws/best_nano_underwater_3.pt")

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
        image_np = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
        annotated_frame = image_np.copy()

        # YOLO segmentation model - trained
        results = model.predict(source=image_np, imgsz=640, conf=0.4, classes=None, verbose=False)

        current_point = [0, 0]
        area = 0.0
        width = 0.0
        yaw_deg = None 

        all_contours = []
        all_masks = []

        # all contours from all detected masks
        for result in results:
            masks = result.masks
            names = result.names
            class_ids = result.boxes.cls.cpu().numpy().astype(int)  # class indices per mask

            if masks is not None:
                for i, (mask, class_id) in enumerate(zip(masks.data, class_ids)):
                    class_name = names[class_id]
                    
                    # to skip unnecessary classes and work with one only
                    if class_name != "boat_segmentation": 
                        continue

                    mask_np = mask.cpu().numpy().astype("uint8") * 255
                    mask_np = cv2.resize(mask_np, (image_np.shape[1], image_np.shape[0]), interpolation=cv2.INTER_NEAREST)

                    contours, _ = cv2.findContours(mask_np, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    if contours:
                        largest = max(contours, key=cv2.contourArea)
                        all_contours.append(largest)
                        all_masks.append(mask_np)

        # |largest contour| - done to avoid double detected objects in the frame
        if all_contours:
            largest_contour = max(all_contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            M = cv2.moments(largest_contour)

            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                current_point = [cx, cy]


                # width calculation
                pts = largest_contour.reshape(-1, 2)
                x_vals = pts[:, 0]
                y_vals = pts[:, 1]

                min_x, max_x = np.min(x_vals), np.max(x_vals)
                min_y, max_y = np.min(y_vals), np.max(y_vals)

                width = max_x - min_x
                height = max_y - min_y

                # boat
                cv2.circle(annotated_frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(annotated_frame, f"{names[0]}", (cx + 5, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                # self.get_logger().info(f"Object {names[0]} at the point {current_point} with area ")
                cv2.drawContours(annotated_frame, [largest_contour], -1, (0, 255, 0), 1)

                
                
                # red tape for yaw angle detection - not fully working. It requires more color tuning
                hsv = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)

                lower_red1 = np.array([10, 24, 50])
                upper_red1 = np.array([40, 80, 75])

                mask_red = cv2.inRange(hsv, lower_red1, upper_red1)

                # operate just inside the boat contour
                boat_mask = np.zeros_like(mask_red)
                cv2.drawContours(boat_mask, [largest_contour], -1, 255, thickness=cv2.FILLED)
                red_inside_boat = cv2.bitwise_and(mask_red, boat_mask)

                # contours for the red tape
                red_contours, _ = cv2.findContours(red_inside_boat, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


                # Compute centers of the tape area
                red_centers = []
                for cnt in red_contours:
                    M = cv2.moments(cnt)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        red_centers.append(np.array([cx, cy]))

                # Draw the red contours
                cv2.drawContours(annotated_frame, red_contours, -1, (0, 255, 255), 2)
                cv2.circle(annotated_frame, (cx, cy), 5, (0, 0, 255), -1)

                # process to distinguish motors/tip based on distances
                if len(red_centers) == 3:
                    red_centers = np.array(red_centers)

                    # distances between each pair of points
                    d01 = np.linalg.norm(red_centers[0] - red_centers[1])
                    d02 = np.linalg.norm(red_centers[0] - red_centers[2])
                    d12 = np.linalg.norm(red_centers[1] - red_centers[2])

                    self.get_logger().info(f"Distances: {d01}, {d02}, {d12}")

                    # closest pair to each other will be left/right - tbd later which one is which
                    if d01 <= d02 and d01 <= d12:
                        left_right = [red_centers[0], red_centers[1]]
                        tip = red_centers[2]
                    elif d02 <= d01 and d02 <= d12:
                        left_right = [red_centers[0], red_centers[2]]
                        tip = red_centers[1]
                    else:
                        left_right = [red_centers[1], red_centers[2]]
                        tip = red_centers[0]

                    # detect which one is left/right by assosiated x-coordinate
                    if left_right[0][0] < left_right[1][0]:
                        left, right = left_right[0], left_right[1]
                    else:
                        left, right = left_right[1], left_right[0]

                # if we detect only 2 points
                elif len(red_centers) == 2:
                    dd = np.linalg.norm(red_centers[0] - red_centers[1]) # distance between assumed motors (if we see the back part of the boat)
                    
                    if red_centers[0][0] < red_centers[1][0]:
                        left, right = red_centers[0], red_centers[1]
                    else:
                        left, right = red_centers[1], red_centers[0]

                    midpoint = (left + right) / 2.0

                    direction = current_point - midpoint # current_point is a boat center
                    norm = np.linalg.norm(direction) # roughly the length of tip -> midpoint
                    
                    direction = direction / norm # to get purely the directoin and no length

                    tip = (midpoint + direction * (1/4 * dd)).astype(int) # to 'extrapolate' the length as sides' ratio is ~1/4



                    # Draw points
                    for pt, label in zip([tip, right, left], ["Tip", "Right", "Left"]):
                        cv2.circle(annotated_frame, tuple(pt), 5, (0, 0, 255), -1)
                        cv2.putText(annotated_frame, label, tuple(pt + np.array([5, -5])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                        self.get_logger().info(f"Points: {pt} and states {label}")

                    # Pose estimation
                    obj_pts = np.array([
                        [0.0, 0.0, 0.0],       # Boat tip
                        [5.5, -40.0, 0.0],     # Right motor
                        [-5.5, -40.0, 0.0]     # Left motor
                    ], dtype=np.float32)
                    
                    img_pts = np.array([tip, right, left], dtype=np.float32)

                    camera_matrix = np.array( [
                        [513.23189881,   0.,         489.18060175],
                        [  0.,         515.63388219, 247.67259085],
                        [  0.,           0.,           1.        ]
                    ], dtype=np.float32)

                    # Distortion coefficients (assume zero if unknown)
                    dist_coeffs = np.array([ 0.01649641,  0.02585203,  0.00211138,  0.00230635, -0.02915967], dtype=np.float32)


                    success, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_SQPNP)
                    
                    if success:
                        R, _ = cv2.Rodrigues(rvec)
                        yaw_rad = np.arctan2(R[1, 0], R[0, 0])
                        yaw_deg = np.degrees(yaw_rad)
                        self.get_logger().info(f"Yaw (around vertical axis): {yaw_deg:.2f} degrees")

                    self.get_logger().info("---------")

        # pixel coordinates instead of meters
        current_point_msg = Float64MultiArray(data=current_point)
        self.pub_tracked_point.publish(current_point_msg)

        # contour area in pixels as well
        current_area_msg = Float64MultiArray(data=[area,width])
        self.pub_tracked_area.publish(current_area_msg)

        cv2.imshow("YOLO Segmentation", annotated_frame)
        cv2.waitKey(2)


        # VERSION 2
        # Below is geometrical version of boat detecting - failed as it kept detecting 'dark corners' as a boat

        # # get image data
        # image_np = self.bridge.imgmsg_to_cv2(image_data, "bgr8")  # Convert ROS Image message to OpenCV image

        # # ------------------------starting here------------------------
        # # Convert to HSV color space - Hue Saturation Value
        # hsv = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)

        # # hue spectrum for black color
        # lower_black = np.array([0, 0, 0])
        # upper_black = np.array([180, 255, 50])

        # mask = cv2.inRange(hsv, lower_black, upper_black)
        # cv2.imshow("Mask", mask)
        # contoured_image = image_np.copy()

        # # Find contours of black objects
        # contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # # init default tracked point
        # current_point = [0, 0]
        # heading = 'initial'

        # if contours:
        #     # Find the largest black object
        #     largest_contour = max(contours, key=cv2.contourArea)
        #     M = cv2.moments(largest_contour)

        #     if M["m00"] != 0:
        #         cx = int(M["m10"] / M["m00"])
        #         cy = int(M["m01"] / M["m00"])
        #         current_point = [cx, cy]

                # width calculation 
        #         # pts = largest_contour.reshape(-1, 2)
        #         # x_vals = pts[:, 0]
        #         # y_vals = pts[:, 1]

        #         # min_x, max_x = np.min(x_vals), np.max(x_vals)
        #         # min_y, max_y = np.min(y_vals), np.max(y_vals)

        #         # width = max_x - min_x
        #         # height = max_y - min_y

        #         # # Draw horizontal width line
        #         # # cv2.line(contoured_image, (min_x, cy), (max_x, cy), (255, 0, 0), 2)
        #         # mid_x = (min_x + max_x) // 2
        #         # cv2.putText(contoured_image, f"W: {width}px", (mid_x, cy - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        #         # # Draw vertical height line
        #         # # cv2.line(contoured_image, (cx, min_y), (cx, max_y), (0, 0, 255), 2)
        #         # mid_y = (min_y + max_y) // 2
        #         # cv2.putText(contoured_image, f"H: {height}px", (cx + 10, mid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        #         # object_area = width * height
        #         # self.get_logger().info(f"Detected area has {object_area}")

        #         # if object_area < 60000 and object_area >= 23000:

        #         #     self.get_logger().info(f"Area after the check has height {height} and width {width}")

        #         #     #  boat
        #         #     cv2.drawContours(contoured_image, [largest_contour], -1, (0, 255, 0), 2)  # green contour
        #         #     cv2.circle(contoured_image, (cx, cy), 5, (255, 255, 0), -1)  # tracked point itself
        #         #     overlay_points(contoured_image, current_point, 0, 255, 0, 'Tracked boat')


        #         area = cv2.contourArea(largest_contour)
        #         perimeter = cv2.arcLength(largest_contour, True) 
        #         approx = cv2.approxPolyDP(largest_contour, 0.07 * perimeter, True) # (contour, epsilon, boulean)
        #         sides_num = len(approx)

        #         # to check - delete later
        #         self.get_logger().info(f"Detected area is {area} and perimeters is {perimeter} and {sides_num}") 

        #         if area >= 15000 and sides_num > 2 and sides_num < 5:
        #         # perimeter <= 1700

        #             self.get_logger().info(f"Detected area is {area} and perimeters is {perimeter} and {sides_num}") 
                    
        #             #  boat
        #             cv2.drawContours(contoured_image, [largest_contour], -1, (0, 255, 0), 2)  # green contour
        #             cv2.drawContours(contoured_image, [approx], -1, (255, 0, 0), 2)  # contour for the approximated polygon
        #             # cv2.circle(contoured_image, (cx, cy), 5, (255, 255, 0), -1)  # tracked point itself
        #             # overlay_points(contoured_image, current_point, 0, 255, 0, 'Tracked boat')

                
        # # Convert point to meters and publish
        # current_point_meter = cam.convertOnePoint2meter(current_point)
        # current_point_msg = Float64MultiArray(data = current_point_meter)
        # self.pub_tracked_point.publish(current_point_msg)


        # # Display the image
        # cv2.imshow("Boat tracking", contoured_image)
        

        # cv2.waitKey(2)


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