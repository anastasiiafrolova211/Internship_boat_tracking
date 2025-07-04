#!/usr/bin/env python

import rclpy
import traceback
import numpy as np
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from struct import pack, unpack
from std_msgs.msg import Int16, Float64, Empty, Float64MultiArray, String
from sensor_msgs.msg import Joy, Imu, FluidPressure, LaserScan
from mavros_msgs.srv import CommandLong, SetMode, StreamRate
from mavros_msgs.msg import OverrideRCIn, Mavlink
from mavros_msgs.srv import EndpointAdd
from geometry_msgs.msg import Twist
from time import sleep
from collections import deque
from . import camera_parameters as cam

# from waterlinked_a50_ros_driver.msg import DVL
# from waterlinked_a50_ros_driver.msg import DVLBeam

class MyPythonNode(Node):
    def __init__(self):
        super().__init__("listenerMIR")
        
        self.declare_parameter("run_initialization_test", False)
        self.run_initialization_test = self.get_parameter("run_initialization_test").value
        
        self.get_logger().info("This node is named listenerMIR")

        self.ns = self.get_namespace()
        self.get_logger().info("namespace =" + self.ns)
        self.pub_msg_override = self.create_publisher(OverrideRCIn, "rc/override", 10)
        self.pub_angle_degre = self.create_publisher(Twist, 'angle_degree', 10)
        self.pub_depth = self.create_publisher(Float64, 'depth', 10)
        self.pub_angular_velocity = self.create_publisher(Twist, 'angular_velocity', 10)
        self.pub_linear_velocity = self.create_publisher(Twist, 'linear_velocity', 10)
        
        self.publisher_current_depth = self.create_publisher(Float64, 'current_depth', 10)
        self.publisher_current_yaw = self.create_publisher(Float64, 'current_yaw', 10)
        self.publisher_current_surge = self.create_publisher(Float64, 'current_surge', 10)
        self.get_logger().info("Publishers created.")

        self.get_logger().info("ask router to create endpoint to enable mavlink/from publication.")
        # self.addEndPoint()

        self.armDisarm(False)  # Not automatically disarmed at startup
        rate = 25  # 25 Hz
        self.setStreamRate(rate)
        # self.manageStabilize(False)

        self.subscriber()

        # set timer if needed
        timer_period = 0.05  # 50 msec - 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.timer_period_control = timer_period ###ADDED FOR THE PRACTIAL WORK for the INTEGRAL IN PI
        # variables
        self.set_mode = [0] * 3
        self.set_mode[0] = True  # Mode manual
        self.set_mode[1] = False  # Mode automatic without correction
        self.set_mode[2] = False  # Mode with correction

        # Conditions
        self.init_a0 = True
        self.init_p0 = True
        self.arming = False

        self.angle_roll_ajoyCallback0 = 0.0
        self.angle_pitch_a0 = 0.0
        self.angle_yaw_a0 = 0.0
        self.depth_wrt_startup = 0
        self.depth_p0 = 0

        self.pinger_confidence = 0
        self.pinger_distance = 0

        self.Vmax_mot = 1900
        self.Vmin_mot = 1100
        
        #light values 
        self.light_pin = 13.0  # float between 0 and 15
        self.light_max = 1900.0
        self.light_min = 1100.0
        self.light = 1100.0
        self.light_int = 1100.0
        
        # camera servo 
        self.camera_servo_pin = 15.0 # float between 0 and 15
        self.servo_max = 1850.0
        self.servo_min = 1100.0
        self.tilt_int = 1450.0
        self.tilt = 1450.0

        ## Intail test for the system
        if self.run_initialization_test:
            self.initialization_test()
        
        # corrections for control
        self.Correction_yaw = 1500
        self.Correction_depth = 1500
        self.Correction_surge = 1500
        
        ## TODO ##
        # Task 1 : Calculate the flotability of the ROV
        self.flotability = 2 # is reduced drastically from previous practicals as the rov has heavy configuration
        self.desired_depth = -0.4
        self.current_t = 0
        self.depth_flag = False
        

        #### visual survoing
        self.boat_detected = False
        self.object_state = None

        self.x_value = 0.0
        self.y_value = 0.0
        self.width_value = 0.5
        self.width = 0.0
        self.area = 0.0
        # self.Kd_flag = True
        # self.error_d = np.array([0.0,0.0])

        self.desired_yaw = 0.0
        # self.current_yaw = 0.0 # self.angle_yaw_a0 already

        # extra PI for the surge (+ yaw to keep the logic clear and consistent)
        self.integral_error = np.array([0.0, 0.0, 0.0])
        self.prev_time = self.get_clock().now().nanoseconds / 1e9


        # for 'search when lost' logic
        self.yaw_phase = 0 # 0: to +90, 1: to -90, 2: return to 0
        self.cur_search = 'yaw'
        self.search_dir = 1
        self.last_direction = None
        self.sweep_completed = False
        self.surge_start_time = None


        self.correction_mode = False 

        self.current_yaw = 0.0
        self.area_list = deque(maxlen=5)

    def initialization_test(self):
        """Tests the light by flashing it and tests the camera servo by moving it to max/min limits before starting the sytsem."""
        self.get_logger().info("Testing light and camera servo...")

        # Flash the light
        self.light = self.light_int
        self.send_servo_comand(self.light_pin, self.light)
        sleep(0.5)
        self.light = self.light_max
        self.send_servo_comand(self.light_pin, self.light)
        sleep(0.5)
        self.light = self.light_min
        self.send_servo_comand(self.light_pin, self.light)

        # Move the camera servo to max and min
        self.tilt = self.tilt_int
        self.send_servo_comand(self.camera_servo_pin, self.tilt)
        sleep(0.5)
        self.tilt = self.servo_max
        self.send_servo_comand(self.camera_servo_pin, self.tilt)
        sleep(0.5)
        self.tilt = self.servo_min
        self.send_servo_comand(self.camera_servo_pin, self.tilt)
        sleep(0.5)
        self.tilt = self.tilt_int  # Reset camera tilt to neutral
        self.send_servo_comand(self.camera_servo_pin, self.tilt)
        
        self.get_logger().info("Light and camera servo test completed.")  
        
    def send_servo_comand(self, pin_number, value):
        '''
        Sends a command to the navigator to adjust servo pins pwm using Mavros service
        pin_number (float) --> the servo number in the navigator board (13 for lights and 15 for camera servo)
        value (float) --> The pwm value sent to the servo between 1100 and 1900
        '''
        client = self.create_client(CommandLong, 'cmd/command')
        result = False
        while not result:
                result = client.wait_for_service(timeout_sec=4.0)
        # Create a request object for CommandLong service
        request = CommandLong.Request()

        # Set the parameters for the command (command 183: MAV_CMD_DO_SET_SERVO)
        request.command = 183       # Command 183: MAV_CMD_DO_SET_SERVO
        request.param1 = pin_number           # Servo number (param1)
        request.param2 = value         # Desired servo position (param2)
        request.param3 = 0.0             
        request.param4 = 0.0             

        # Send the service request and wait for the response
        future = client.call_async(request)

        # Check the result
        if future.result() is not None:
            self.get_logger().info('Change Completed')
        else:
            self.get_logger().error('Failed to preform the change ')
        
    
            
    def timer_callback(self):
        '''
        Time step at a fixed rate (1 / timer_period = 20 Hz) to execute control logic.
        '''
        if self.set_mode[0]:  # commands sent inside joyCallback()
            self.correction_mode = False
            return
        elif self.set_mode[
            1]:  # Arbitrary velocity command can be defined here to observe robot's velocity, zero by default
            self.correction_mode = False
            self.setOverrideRCIN(1500, 1500, 1500, 1500, 1500, 1500)
            return
        elif self.set_mode[2]:
            if  not self.correction_mode:
                self.Correction_yaw = 1500
                self.Correction_surge = 1500
                self.depth_flag = False
                self.correction_mode = True
            # send commands in correction mode
            self.setOverrideRCIN(1500, 1500, self.Correction_depth, self.Correction_yaw, self.Correction_surge, 1500)
            # self.setOverrideRCIN(1500, 1500, self.Correction_depth, 1500, 1500, 1500) # only depth control for making videos

        else:  # normally, never reached
            pass

    def armDisarm(self, armed):
        """Arms or disarms the vehicle motors using MAVROS command 400."""
        cli = self.create_client(CommandLong, 'cmd/command')  # Create MAVROS service client
        result = False
        while not result:
            result = cli.wait_for_service(timeout_sec=4.0)  # Wait for service to be available
            self.get_logger().info(f"{'Arming' if armed else 'Disarming'} requested, waiting for service: {result}")
        
        # Create request object for arming/disarming
        req = CommandLong.Request()
        req.broadcast = False  # Command is not broadcasted
        req.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
        req.confirmation = 0  # No confirmation required
        req.param1 = 1.0 if armed else 0.0  # 1.0 = Arm, 0.0 = Disarm
        req.param2 = 0.0  
        req.param3 = 0.0  
        req.param4 = 0.0  
        req.param5 = 0.0  
        req.param6 = 0.0  
        req.param7 = 0.0 
        
        self.get_logger().info("Sending command...")
        resp = cli.call_async(req)  # Send command asynchronously
        
        # Log the result
        self.get_logger().info(f"{'Arming' if armed else 'Disarming'} Succeeded")

    # def manageStabilize(self, stabilized):
    #     # This functions sends a SetMode command service to stabilize or reset
    #     if (stabilized):
    #         traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
    #         cli = self.create_client(SetMode, 'set_mode')
    #         result = False
    #         while not result:
    #             result = cli.wait_for_service(timeout_sec=4.0)
    #             self.get_logger().info(
    #                 "stabilized mode requested, wait_for_service, (False if timeout) result :" + str(result))
    #         req = SetMode.Request()
    #         req.base_mode = 0
    #         req.custom_mode = "0"
    #         resp = cli.call_async(req)
    #         # rclpy.spin_until_future_complete(self, resp)
    #         self.get_logger().info("set mode to STABILIZE Succeeded")

    #     else:
    #         traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
    #         result = False
    #         cli = self.create_client(SetMode, 'set_mode')
    #         while not result:
    #             result = cli.wait_for_service(timeout_sec=4.0)
    #             self.get_logger().info(
    #                 "manual mode requested, wait_for_service, (False if timeout) result :" + str(result))
    #         req = SetMode.Request()
    #         req.base_mode = 0
    #         req.custom_mode = "19"
    #         resp = cli.call_async(req)
    #         # rclpy.spin_until_future_complete(self, resp)
    #         self.get_logger().info("set mode to MANUAL Succeeded")

    def setStreamRate(self, rate):
        ''' Set the Mavros rate for reading the senosor data'''
        traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
        cli = self.create_client(StreamRate, 'set_stream_rate')
        result = False
        while not result:
            result = cli.wait_for_service(timeout_sec=4.0)
            self.get_logger().info("stream rate requested, wait_for_service, (False if timeout) result :" + str(result))

        req = StreamRate.Request()
        req.stream_id = 0
        req.message_rate = rate
        req.on_off = True
        resp = cli.call_async(req)
        rclpy.spin_until_future_complete(self, resp)
        self.get_logger().info("set stream rate Succeeded")

    # def addEndPoint(self):
    #     traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
    #     cli = self.create_client(EndpointAdd, 'mavros_router/add_endpoint')
    #     result = False
    #     while not result:
    #         result = cli.wait_for_service(timeout_sec=4.0)
    #         self.get_logger().info(
    #             "add endpoint requesRelAltCallbackted, wait_for_service, (False if timeout) result :" + str(result))

    #     req = EndpointAdd.Request()
    #     req.url = "udp://@localhost"
    #     req.type = 1  # TYPE_GCS
    #     resp = cli.call_async(req)
    #     rclpy.spin_until_future_complete(self, resp)
    #     self.get_logger().info("add endpoint rate Succeeded")

    def joyCallback(self, data):
        ''' Map the Joystick buttons according the bluerov configuration as descriped at
        https://bluerobotics.com/wp-content/uploads/2023/02/default-button-layout-xbox.jpg
        **Note: the lights are set to be in RT and LT button instead of the cross buttons'''
        btn_arm = data.buttons[7]  # Start button
        btn_disarm = data.buttons[6]  # Back button
        btn_manual_mode = data.buttons[3]  # Y button
        btn_automatic_mode = data.buttons[2]  # X button
        btn_corrected_mode = data.buttons[0]  # A button
        btn_camera_servo_up = data.buttons[4] # LB button 
        btn_camera_servo_down = data.buttons[5] # RB button 
        btn_camera_rest = data.buttons[9] # R3 button 
        btn_light_down = data.axes[2] # LT button
        btn_light_up = data.axes[5] # RT button
        

        # Disarming when Back button is pressed
        if (btn_disarm == 1 and self.arming == True):
            self.arming = False
            self.armDisarm(self.arming)

        # Arming when Start button is pressed
        if (btn_arm == 1 and self.arming == False):
            self.arming = True
            self.armDisarm(self.arming)

        # Switch manual, auto anset_moded correction mode
        if (btn_manual_mode and not self.set_mode[0]):
            self.set_mode[0] = True
            self.set_mode[1] = False
            self.set_mode[2] = False
            self.get_logger().info("Mode manual")
        if (btn_automatic_mode and not self.set_mode[1]):
            self.set_mode[0] = False
            self.set_mode[1] = True
            self.set_mode[2] = False
            self.get_logger().info("Mode automatic")
        if (btn_corrected_mode and not self.set_mode[2]):
            self.init_a0 = True
            self.init_p0 = True
            #Integral error reset 
            self.integral_error = 0
            self.set_mode[0] = False
            self.set_mode[1] = False
            self.set_mode[2] = True
            self.get_logger().info("Mode correction")
            
        
        #### Control light intensity####
        if (btn_light_up == -1 and self.light < self.light_max):
            self.light = min(self.light + 100.0, self.light_max)
            self.send_servo_comand(self.light_pin,self.light)
            self.get_logger().info(f"light PWM is: {self.light}")
            
        elif (btn_light_down == -1 and self.light > self.light_min):
            self.light = max(self.light_min,self.light - 100)
            self.send_servo_comand(self.light_pin,self.light)
            self.get_logger().info(f"light PWM is: {self.light}")

        ### Control Camera tilt angle ###
        if (btn_camera_servo_up and not btn_camera_servo_down and self.tilt < self.servo_max):
            self.tilt = min(self.servo_max, self.tilt + 100)
            self.send_servo_comand(self.camera_servo_pin, self.tilt)
            self.get_logger().info(f"tilt pwm: {self.tilt}")
            
        elif (btn_camera_servo_down and self.tilt > self. servo_min):
            self.tilt = max(self.servo_min, self.tilt - 100)
            self.send_servo_comand(self.camera_servo_pin, self.tilt)
            self.get_logger().info(f"tilt pwm: {self.tilt}")
            
        elif (btn_camera_rest):
            self.tilt = self.tilt_int
            self.send_servo_comand(self.camera_servo_pin,self.tilt)
            self.get_logger().info(f"Camera tilt has been reseted")
            
            
            

    def velCallback(self, cmd_vel):
        ''' Used in manual mode to read the values of the analog and map it pwm then send it the thrusters'''
        if (self.set_mode[1] or self.set_mode[2]):
            return
        else:
            self.get_logger().info("Sending...")

        # Extract cmd_vel message
        roll_left_right = self.mapValueScalSat(cmd_vel.angular.x)
        yaw_left_right = self.mapValueScalSat(-cmd_vel.angular.z)
        ascend_descend = self.mapValueScalSat(cmd_vel.linear.z)
        forward_reverse = self.mapValueScalSat(cmd_vel.linear.x)
        lateral_left_right = self.mapValueScalSat(-cmd_vel.linear.y)
        pitch_left_right = self.mapValueScalSat(cmd_vel.angular.y)
        
        # send the commands to the mthrusters 
        self.setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend, yaw_left_right, forward_reverse,
                             lateral_left_right)
        

    def setOverrideRCIN(self, channel_pitch, channel_roll, channel_throttle, channel_yaw, channel_forward,
                        channel_lateral):
        ''' This function replaces setservo for motor commands.
            It overrides Rc channels inputs and simulates motor controls.
            In this case, each channel manages a group of motors (DOF) not individually as servo set '''

        msg_override = OverrideRCIn()
        msg_override.channels[0] = np.uint(channel_pitch)  # pulseCmd[4]--> pitch
        msg_override.channels[1] = np.uint(channel_roll)  # pulseCmd[3]--> roll
        msg_override.channels[2] = np.uint(channel_throttle)  # pulseCmd[2]--> heave
        msg_override.channels[3] = np.uint(channel_yaw)  # pulseCmd[5]--> yaw
        msg_override.channels[4] = np.uint(channel_forward)  # pulseCmd[0]--> surge
        msg_override.channels[5] = np.uint(channel_lateral)  # pulseCmd[1]--> sway
        msg_override.channels[6] = 1500 # camera pan servo motor speed 
        msg_override.channels[7] = 1500 #camers tilt servo motro speed

        self.pub_msg_override.publish(msg_override)

    def mapValueScalSat(self, value):
        ''' Map the value of the joysteck analog form -1 to 1 to a pwm value form 1100 to 1900
            where 1500 is the stop value 1100 is maximum negative and 1900 is maximum positive'''
        pulse_width = value * 400 + 1500

        # Saturation
        if pulse_width > 1900:
            pulse_width = 1900
        if pulse_width < 1100:
            pulse_width = 1100

        return int(pulse_width)

    def OdoCallback(self, data):
        ''' Read the Imu data angular velocities and angles and convert the angles from quaternion angles 
            to roll, pitch and yaw then publish them in sperate new topics '''
        orientation = data.orientation
        angular_velocity = data.angular_velocity

        # extraction of roll, pitch, yaw angles
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        sinp = 2.0 * (w * y - z * x)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        angle_roll = np.arctan2(sinr_cosp, cosr_cosp)
        angle_pitch = np.arcsin(sinp)
        angle_yaw = np.arctan2(siny_cosp, cosy_cosp)

        if (self.init_a0):
            # at 1st execution, init
            self.angle_roll_a0 = angle_roll
            self.angle_pitch_a0 = angle_pitch
            self.angle_yaw_a0 = angle_yaw
            self.init_a0 = False

        angle_wrt_startup = [0] * 3
        angle_wrt_startup[0] = ((angle_roll - self.angle_roll_a0 + 3.0 * math.pi) % (
                    2.0 * math.pi) - math.pi) * 180 / math.pi
        angle_wrt_startup[1] = ((angle_pitch - self.angle_pitch_a0 + 3.0 * math.pi) % (
                    2.0 * math.pi) - math.pi) * 180 / math.pi
        angle_wrt_startup[2] = ((angle_yaw - self.angle_yaw_a0 + 3.0 * math.pi) % (
                    2.0 * math.pi) - math.pi) * 180 / math.pi

        angle = Twist()
        angle.angular.x = angle_wrt_startup[0]
        angle.angular.y = angle_wrt_startup[1]
        angle.angular.z = angle_wrt_startup[2]

        self.pub_angle_degre.publish(angle)

        # Extraction of angular velocity
        p = angular_velocity.x
        q = angular_velocity.y
        r = angular_velocity.z

        vel = Twist()
        vel.angular.x = p
        vel.angular.y = q
        vel.angular.z = r
        self.pub_angular_velocity.publish(vel)

        # Only continue if manual_mode is disabled
        if (self.set_mode[0]):
            return

        self.current_yaw = angle_yaw


    def RelAltCallback(self, data):
        if (self.init_p0):
            # 1st execution, init
            self.depth_p0 = data
            self.current_t = 0
            self.init_p0 = False
            
        Kp = 8.0 

        desired_depth = self.desired_depth # 0.4 meters
        depth_error = data.data - desired_depth
        fz = -(Kp * depth_error + self.flotability)
        # self.get_logger().info("this is the error: "+str(depth_error))

        if fz < 0:
            pwm_z = 1464 + 12.3 * fz / 4 # divide by 4 for each thruster on the rov
        else:
            pwm_z = 1536 + 9.6 * fz / 4


        # update Correction_depth
        Correction_depth = pwm_z
        # Correction_depth = 1500
        self.Correction_depth = int(Correction_depth)
        # Send PWM commands to motors in timer

        #### publish data for plots
        msg = Float64()
        msg.data = desired_depth
        self.publisher_current_depth.publish(msg)

        if abs(depth_error) < 0.07: 
            if not self.depth_flag:
                self.get_logger().info(f"depth stable")
                self.depth_flag = True



    def pingerCallback(self, data):
        self.pinger_distance = data.data[0]
        self.pinger_confidence = data.data[1]


    # to get area data from image_processing_tracker for Control
    def area_msg(self, area_msg):
        self.area = area_msg.data[0]
        self.width = area_msg.data[1]
        # self.get_logger().info(f"area_msg start")


    def visual_servoing_callback(self, msg):
        # self.get_logger().info(f"Boat is detected")

        if self.depth_flag and abs(self.area) > 0.0:
            self.boat_detected = True

            # desired_width = 40000 # desired area - desired distance is done with Interaction matrix now instead
            
            desired_x = 480
            desired_y = 270
            desired_z = 1.3

            self.x_value = msg.data[0]
            self.y_value = msg.data[1]
            # actual_area = self.area

            # the logic of the area_list when the tracking was lost - start the list over for proper calculations
            # average area is calculated to smooth the surge movement
            if self.object_state == "search":
                self.area_list = deque(maxlen=5)

            if len(self.area_list) != 0:
                if abs(self.area - (sum(self.area_list)/len(self.area_list))) > 3000:
                    self.area_list.append(self.area)

                self.width_value = sum(self.area_list)/len(self.area_list)
                # self.get_logger().info(f"area average = {sum(self.area_list)/len(self.area_list)}")
            
            else:
                self.width_value = self.area
                self.area_list.append(self.area) # was absent for the previous water tank check

            # self.get_logger().info(f"Detected area is {self.width_value} with len(area_list) {len(self.area_list)}")


            Z =  (0.4*75 * 455) / self.width_value 

            self.get_logger().info(f"Z  = {Z}")
            
            # sampling_time = 1/25 # for integral controller
            
            # to work with meters datat is converted before the error calculations
            x, y = cam.convertOnePoint2meter([self.x_value, self.y_value])
            xd,yd = cam.convertOnePoint2meter([desired_x, desired_y])
            
            error_x = xd - x
            error_y = yd - y
            error_z = desired_z - Z         
            error = [error_x, error_y, error_z]
            

            # error_d_plus = (error - self.error_d) / error

            # self.integral_error = self.integral_error + error * sampling_time
            # self.integral_error = np.clip(self.integral_error, -5000, 5000)
            # self.get_logger().info(f"xI error= {self.integral_error}")

            # function for kp as area dependency
            # k_p_Lx = np.diag([0.07, 0.003]) # tune parameters P

            k_p_Lx = np.diag([[0.15,0.2,0.2,0.005,0.005,0.015]])
            # surge sway heave roll pitch yaw


            # k_i_Lx = np.diag([0, 0, 0, 0, 0, 0]) # tune parameters I TODO
            

            # k_p_Lx = np.diag([0.05, 0.02]) # tune parameters P
            # k_i_Lx = np.diag([0, 0.02]) # tune parameters I
            
            

            # x error = yaw 
            # size error = surge
            # L = np.array([
            #     [1/Z, 0],  # прямая зависимость так как двигаемся в
            #     [0, -1/Z]  # обратная зависимость так как двигаемся вперед
            # ])

            # L_pinv = np.linalg.pinv(L)

            # v = -k_p_Lx * L_pinv @ error + -kd * L_pinv @ error_d_plus
            # v = -k_p_Lx * L_pinv @ error 
            # v = -k_p_Lx @ L_pinv @ error - k_i_Lx @ L_pinv @ self.integral_error

            # self.get_logger().info(f"velocity {v}")

            # v_yaw = v[0, 0] # shape of v - take the 1st value
            # v_surge = v[1, 0] # shape of v - take the 2nd value


            fx = 1
            fy = 1
            

            L = np.array([
                [fx * (-1/Z), 0, fx * (x/Z), fx * (x * y), -fx * (1 + x**2), fx * y],
                [0, fy * (-1/Z), fy * (y/Z), fy * (1 + y**2), -fy * (x * y), -fy * x],
                [0, 0, fx * (1/Z),0, 0, 0],
            ])

            L_inv = np.linalg.pinv(L)  # 6x2 pseudo-inverse
            
        
            v_cam_6d = -k_p_Lx * L_inv @ error  #- k_i_Lx @ L_inv @ self.integral_error
            
            R = np.array([[0, 0, 1],
                          [1, 0, 0],
                          [0, 1, 0]])
            # t = [0, 0, -0.2]
            t = [0,0,0]

            st = np.array([
                [0, -t[2], t[1]],
                [t[2], 0, -t[0]],
                [-t[1], t[0], 0]
            ])
            
            upper = np.hstack((R, st @ R))
            lower = np.hstack((np.zeros((3, 3)), R))
            
            H = np.vstack((upper, lower))
            
            v_rov_6d = H @ v_cam_6d

            for i in range(6):
                if v_rov_6d[i] > 0.5:
                    v_rov_6d[i] = 0.5
                elif v_rov_6d[i] < -0.5:
                    v_rov_6d[i] = -0.5
            # v_rov_6d = [0,0,0,0,0,0.1]

            self.Correction_surge = self.mapValueScalSat(v_rov_6d[0])
            # self.Correction_surge = 1500
            
            # седня нало поработать над этой частью 
            # self.Camera_cooriction_sway = self.mapValueScalSat(-v_rov_6d[1])
            # self.Camera_cooriction_heave = self.mapValueScalSat(v_rov_6d[2])
            # self.Camera_cooriction_roll = self.mapValueScalSat(v_rov_6d[3])
            # self.Camera_cooriction_pitch = self.mapValueScalSat(-v_rov_6d[4])
            self.Correction_yaw = self.mapValueScalSat(-v_rov_6d[5])

            # self.get_logger().info(f"velocities {v_rov_6d[0]} and {-v_rov_6d[5]}")

            # if v_yaw > 0: # rotate first
            #     pwm_yaw = 1464 + 12.3 * v_yaw / 4       
            # elif v_yaw < 0:
            #     pwm_yaw = 1536 + 9.6 * v_yaw / 4
            # else:
            #     pwm_yaw = 1500

            # if v_surge > 0:  # then translate 
            #     pwm_surge = 1464 + 12.3 * v_surge / 4 
            # elif v_surge < 0: 
            #     pwm_surge = 1536 + 9.6 * v_surge / 4
            # else:
            #     pwm_surge = 1500
            

            # clip so you dont accelarate accidentally
            # pwm_yaw = np.clip(pwm_yaw, 1450, 1540)
            # pwm_surge = np.clip(pwm_surge, 1430, 1600)

            pwm_yaw = self.Correction_yaw
            pwm_surge = self.Correction_surge

            yaw_msg = Float64()
            yaw_msg.data = float(pwm_yaw)
            self.publisher_current_yaw.publish(yaw_msg)

            surge_msg = Float64()
            surge_msg.data = float(pwm_surge)
            self.publisher_current_surge.publish(surge_msg)

            self.Correction_yaw = int(pwm_yaw)
            self.Correction_surge = int(pwm_surge)

            # self.get_logger().info(f"x coordinate (yaw) error = {error[0]}, y coordinate (heave kinda) error = {error[1]}, z (surge) error = {error[2]}")

            self.get_logger().info(f"x = {x}, y = {y}, desired x = {xd}, desired y = {yd}") # верни те +50 и проверь поменялось ли что в desired values

            self.object_state = "track"

        elif self.depth_flag and self.area == 0:
            # self.get_logger().info(f"Boat is not detected")
            self.boat_detected = False

            self.Correction_yaw = 1530
            self.Correction_surge = 1500

            self.object_state = "search"

            # # Mahmoud's code below - there was a problem with controlling rotations
            # # chage it to the destection flag
            # if self.boat_detected:
            #     self.object_state = "track"


            # yaw_max = np.pi / 2
            # yaw_min = -np.pi / 2
            # yaw_int = self.angle_yaw_a0  # Initial yaw
            # omega = 0.8  # Angular speed for turning
            # surge_rate = 0.5  # Forward speed
            # surge_duration = 5.0  # seconds


            # yaw_targets = [yaw_max + yaw_int, yaw_min + yaw_int, yaw_int]
            # yaw_tolerance = np.deg2rad(5)  # Tolerance for yaw target


            # if self.cur_search == 'yaw':
            #     self.Correction_surge = 1500  # Stop surge during yaw phase
                
            #     self.yaw_desired = yaw_targets[self.yaw_phase]
            #     yaw_error = self.yaw_desired - self.current_yaw
            #     # Normalize yaw_error to [-pi, pi]
            #     yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi
            #     # Check if we're close enough to target yaw
            #     if abs(yaw_error) < yaw_tolerance:
            #         self.get_logger().info(f'Yaw phase {self.yaw_phase} reached target {round(self.yaw_desired, 2)}')
            #         self.yaw_phase += 1

            #         if self.yaw_phase > 2:
            #             self.get_logger().info('Full sweep complete. Starting surge.')
            #             self.cur_search = 'surge'
            #             surge_start_time = self.get_clock().now().nanoseconds * 1e-9
            #             self.yaw_phase = 0  # Reset for next round


            # # Surge logic
            # elif self.cur_search == 'surge':
            #     self.Correction_surge = self.mapValueScalSat(surge_rate)
            #     current_time = self.get_clock().now().nanoseconds * 1e-9
            #     if current_time - surge_start_time > surge_duration:
            #         self.get_logger().info('Surge complete. Restarting yaw sweep.')
            #         self.cur_search = 'yaw'
            #         self.last_direction = None
            #         self.sweep_completed = False

            # self.object_state = "search"




    def subscriber(self):
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subjoy = self.create_subscription(Joy, "joy", self.joyCallback, qos_profile=qos_profile)
        self.subjoy  # prevent unused variable warning
        self.subcmdvel = self.create_subscription(Twist, "cmd_vel", self.velCallback, qos_profile=qos_profile)
        self.subcmdvel  # prevent unused variable warning
        self.subimu = self.create_subscription(Imu, "imu/data", self.OdoCallback, qos_profile=qos_profile)
        self.subimu  # prevent unused variable warning
        # subscriber for camera visual survoing ##############################
        self.camdata = self.create_subscription(Float64MultiArray,'/tracked_point',self.visual_servoing_callback, qos_profile=qos_profile)
        self.camdata
        self.camdata2 = self.create_subscription(Float64MultiArray,'/tracked_area',self.area_msg, qos_profile=qos_profile)
        self.camdata2
        ##############################################################
        self.subrel_alt = self.create_subscription(Float64, "global_position/rel_alt", self.RelAltCallback,
                                                   qos_profile=qos_profile)
        self.subrel_alt  # prevent unused variable warning
       
        # self.sub = self.create_subscription(DVL, "/dvl/data", DvlCallback)
        self.subping = self.create_subscription(Float64MultiArray, "ping1d/data", self.pingerCallback,
                                                qos_profile=qos_profile)
        self.subping  # prevent unused variable warning

        self.get_logger().info("Subscriptions done.")


def main(args=None):
    rclpy.init(args=args)
    node = MyPythonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()