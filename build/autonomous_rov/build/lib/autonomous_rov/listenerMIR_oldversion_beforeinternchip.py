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
import time


# from waterlinked_a50_ros_driver.msg import DVL
# from waterlinked_a50_ros_driver.msg import DVLBeam

class MyPythonNode(Node):
    def __init__(self):
        super().__init__("listenerMIR")
        self.get_logger().info("This node is named listenerMIR")

        self.ns = self.get_namespace()
        self.get_logger().info("namespace =" + self.ns)
        self.pub_msg_override = self.create_publisher(OverrideRCIn, "rc/override", 10)
        self.pub_angle_degre = self.create_publisher(Twist, 'angle_degree', 10)
        self.pub_depth = self.create_publisher(Float64, 'depth', 10)
        self.pub_angular_velocity = self.create_publisher(Twist, 'angular_velocity', 10)
        self.pub_linear_velocity = self.create_publisher(Twist, 'linear_velocity', 10)
        # Added to plot data:
        self.publisher_desired_depth = self.create_publisher(Float64, 'desired_depth', 10)
        self.publisher_desired_heave = self.create_publisher(Twist, 'desired_heave', 10)
        self.publisher_estimated_heave = self.create_publisher(Twist, 'estimated_heave', 10)
        self.publisher_desired_yaw = self.create_publisher(Float64, 'desired_yaw', 10)
        self.publisher_measured_yaw = self.create_publisher(Float64, 'measured_yaw', 10)
        #added for pinger
        self.publisher_detected_obstacle = self.create_publisher(Float64, 'detected_obstacle', 10)
        
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
        self.pinger_callback_counter = 0


        self.Vmax_mot = 1900
        self.Vmin_mot = 1100

        # corrections for control
        self.Correction_yaw = 1500
        self.Correction_depth = 1500
        self.Correction_surge = 1500

        # TODO Task4: Flotability of the ROV
        self.flotability = 11 # 12N, 1.2kg
        self.desired_depth = -0.5
        self.current_t = 0
        self.integral_error = 0

        # TODO Task9: alpha beta
        # Alfa and beta from instructions
        self.alfa = 0.1
        self.beta = 0.005
        self.dt_alpha_beta = 1 / 58  # 58 Hz

        # Initialize variables
        self.z_filter = 0
        self.w_filter = 0
        self.z_filter_1 = 0
        self.w_filter_1 = 0
        self.pred_error = 0
        
        # TODO Task_yaw:
        self.desired_yaw = self.angle_yaw_a0 #+ math.pi / 2 # current angle + pi/2
        # self.current_t_yaw = 0
        self.integral_error_yaw = 0
        self.prev_yaw_err = 0
        #FINAL TRY
        self.yaw_at_press = 0
        self.yaw_readings = 0
        self.prev_time = time.time()

    def timer_callback(self):
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.i += 1
        #TODO update t? this timer is called every 50msec
        self.current_t = self.current_t + 0.05

        if self.set_mode[0]:  # commands sent inside joyCallback()
            return
        elif self.set_mode[1]:  # Arbitrary velocity command can be defined here to observe robot's velocity, zero by default
            self.setOverrideRCIN(1500, 1500, 1500, 1500, 1500, 1500)
            return
        elif self.set_mode[2]:
            # send commands in correction mode
            self.setOverrideRCIN(1500, 1500, self.Correction_depth, self.Correction_yaw, self.Correction_surge, 1500)
        else:  # normally, never reached
            pass

    def armDisarm(self, armed):
        # This functions sends a long command service with 400 code to arm or disarm motors
        if (armed):
            traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
            cli = self.create_client(CommandLong, 'cmd/command')
            result = False
            while not result:
                result = cli.wait_for_service(timeout_sec=4.0)
                self.get_logger().info("arming requested, wait_for_service, timeout, result :" + str(result))
            req = CommandLong.Request()
            req.broadcast = False
            req.command = 400
            req.confirmation = 0
            req.param1 = 1.0
            req.param2 = 0.0
            req.param3 = 0.0
            req.param4 = 0.0
            req.param5 = 0.0
            req.param6 = 0.0
            req.param7 = 0.0
            self.get_logger().info("just before call_async")
            resp = cli.call_async(req)
            self.get_logger().info("just after call_async")
            # rclpy.spin_until_future_complete(self, resp)
            self.get_logger().info("Arming Succeeded")
        else:
            traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
            cli = self.create_client(CommandLong, 'cmd/command')
            result = False
            while not result:
                result = cli.wait_for_service(timeout_sec=4.0)
                self.get_logger().info(
                    "disarming requested, wait_for_service, (False if timeout) result :" + str(result))
            req = CommandLong.Request()
            req.broadcast = False
            req.command = 400
            req.confirmation = 0
            req.param1 = 0.0
            req.param2 = 0.0
            req.param3 = 0.0
            req.param4 = 0.0
            req.param5 = 0.0
            req.param6 = 0.0
            req.param7 = 0.0
            resp = cli.call_async(req)
            # rclpy.spin_until_future_complete(self, resp)
            self.get_logger().info("Disarming Succeeded")

    def manageStabilize(self, stabilized):
        # This functions sends a SetMode command service to stabilize or reset
        if (stabilized):
            traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
            cli = self.create_client(SetMode, 'set_mode')
            result = False
            while not result:
                result = cli.wait_for_service(timeout_sec=4.0)
                self.get_logger().info(
                    "stabilized mode requested, wait_for_service, (False if timeout) result :" + str(result))
            req = SetMode.Request()
            req.base_mode = 0
            req.custom_mode = "0"
            resp = cli.call_async(req)
            # rclpy.spin_until_future_complete(self, resp)
            self.get_logger().info("set mode to STABILIZE Succeeded")

        else:
            traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
            result = False
            cli = self.create_client(SetMode, 'set_mode')
            while not result:
                result = cli.wait_for_service(timeout_sec=4.0)
                self.get_logger().info(
                    "manual mode requested, wait_for_service, (False if timeout) result :" + str(result))
            req = SetMode.Request()
            req.base_mode = 0
            req.custom_mode = "19"
            resp = cli.call_async(req)
            # rclpy.spin_until_future_complete(self, resp)
            self.get_logger().info("set mode to MANUAL Succeeded")

    def setStreamRate(self, rate):
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

    def addEndPoint(self):
        traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
        cli = self.create_client(EndpointAdd, 'mavros_router/add_endpoint')
        result = False
        while not result:
            result = cli.wait_for_service(timeout_sec=4.0)
            self.get_logger().info(
                "add endpoint requesRelAltCallbackted, wait_for_service, (False if timeout) result :" + str(result))

        req = EndpointAdd.Request()
        req.url = "udp://@localhost"
        req.type = 1  # TYPE_GCS
        resp = cli.call_async(req)
        rclpy.spin_until_future_complete(self, resp)
        self.get_logger().info("add endpoint rate Succeeded")

    def joyCallback(self, data):
        # Joystick buttons
        btn_arm = data.buttons[7]  # Start button
        btn_disarm = data.buttons[6]  # Back button
        btn_manual_mode = data.buttons[3]  # Y button
        btn_automatic_mode = data.buttons[2]  # X button
        btn_corrected_mode = data.buttons[0]  # A button

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
            self.yaw_at_press = self.yaw_readings
            self.prev_time = time.time()

            # set sum errors to 0 here, ex: Sum_Errors_Vel = [0]*3
            self.set_mode[0] = False
            self.set_mode[1] = False
            self.set_mode[2] = True
            self.get_logger().info("Mode correction")

    def velCallback(self, cmd_vel):
        # Only continue if manual_mode is enabled
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

        self.setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend, yaw_left_right, forward_reverse,
                             lateral_left_right)

    def setOverrideRCIN(self, channel_pitch, channel_roll, channel_throttle, channel_yaw, channel_forward,
                        channel_lateral):
        # This function replaces setservo for motor commands.
        # It overrides Rc channels inputs and simulates motor controls.
        # In this case, each channel manages a group of motors not individually as servo set

        msg_override = OverrideRCIn()
        msg_override.channels[0] = np.uint(channel_pitch)  # pulseCmd[4]--> pitch
        msg_override.channels[1] = np.uint(channel_roll)  # pulseCmd[3]--> roll
        msg_override.channels[2] = np.uint(channel_throttle)  # pulseCmd[2]--> heave
        msg_override.channels[3] = np.uint(channel_yaw)  # pulseCmd[5]--> yaw
        msg_override.channels[4] = np.uint(channel_forward)  # pulseCmd[0]--> surge
        msg_override.channels[5] = np.uint(channel_lateral)  # pulseCmd[1]--> sway
        msg_override.channels[6] = 1500
        msg_override.channels[7] = 1500

        self.pub_msg_override.publish(msg_override)

    def mapValueScalSat(self, value):
        # Correction_Vel and joy between -1 et 1
        # scaling for publishing with setOverrideRCIN values between 1100 and 1900
        # neutral point is 1500
        pulse_width = value * 400 + 1500

        # Saturation
        if pulse_width > 1900:
            pulse_width = 1900
        if pulse_width < 1100:
            pulse_width = 1100

        return int(pulse_width)

    def OdoCallback(self, data):
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
            self.integral_error = 0
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

        #try final
        self.yaw_readings = angle_wrt_startup[2]

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

        # Send PWM commands to motors
        # yaw command to be adapted using sensor feedback
        # Proportional control
        Kp_yaw = 0.2
        Kd_yaw = 0.1
        desired_yaw = self.yaw_at_press  
        #desired_yaw, _ = self.cubic_trajectory(self.angle_yaw_a0, self.desired_yaw, self.current_t)
        #yaw_error = desired_yaw +self.angle_yaw_a0- angle_yaw

        #OLD VERSION FINAL TRY ------------------------------------------------------------------------------------------------------
        """yaw_error = desired_yaw - self.yaw_readings
        if (yaw_error>math.pi):
            yaw_error = yaw_error - 2*math.pi
        elif(yaw_error<-math.pi):
            yaw_error = yaw_error + 2*math.pi

        gamma_z = -( Kp_yaw * yaw_error)

        Kd_yaw = 0
        new_error = (yaw_error - self.prev_yaw_err)/(yaw_error+1e-4)
        
        self.prev_yaw_err = yaw_error
        if (new_error>math.pi):
            new_error = new_error - 2*math.pi
        elif(new_error<-math.pi):
            new_error = new_error + 2*math.pi
        
        gamma_z = gamma_z - Kd_yaw*new_error"""
        #End of old version final try --------------------------------------------------------------------------------------------------

        #NEW VERSION FINAL TRY ++++++++++++++++++++++++++++++++++++++++++++++++++++
        """yaw_error = desired_yaw - self.yaw_readings #error for P control
        new_error = (yaw_error - self.prev_yaw_err)/(yaw_error+1e-4) #error for D control
        self.prev_yaw_err = yaw_error #update needed for D error calc

        if (yaw_error>math.pi):
            yaw_error = yaw_error - 2*math.pi
        elif(yaw_error<-math.pi):
            yaw_error = yaw_error + 2*math.pi
        
        if (new_error>math.pi):
            new_error = new_error - 2*math.pi
        elif(new_error<-math.pi):
            new_error = new_error + 2*math.pi
        
        gamma_z = -( Kp_yaw * yaw_error) - Kd_yaw*new_error"""

        #new version final try +++++++++++++++++++++++++++++++++++++++++++++

        #SECOND NEW FINAL TRY ////////////////////////////////////////////////////////
        yaw_error = desired_yaw - self.yaw_readings
        if (yaw_error>math.pi):
            yaw_error = yaw_error - 2*math.pi
        elif(yaw_error<-math.pi):
            yaw_error = yaw_error + 2*math.pi

        gamma_z = -( Kp_yaw * yaw_error)

        current_time = time.time()
        dt = current_time - self.prev_time

        new_error = (yaw_error - self.prev_yaw_err)/(dt)
        
        self.prev_yaw_err = yaw_error
        self.prev_time = current_time

        gamma_z = gamma_z - Kd_yaw*new_error
        
        #SECOND NEW FINAL TRY ////////////////////////////////////////////////////////

        # Ki_yaw = 0.1
        # sampling_time = 1/25 # corresponding to 25Hz: depth publishing frequency
        # self.integral_error = self.integral_error + yaw_error*sampling_time
        # # self.get_logger().info("this is the integral error: "+str(self.integral_error))
        # gamma_z =  gamma_z - Ki_yaw*self.integral_error # it is a minus sign because we provide negative values
        # # self.get_logger().info("this is the yaw_error: "+str(yaw_error))


        if gamma_z < 0:
            pwm_yaw = 1464 + 12.3 * gamma_z / 4  
        else:
            pwm_yaw = 1536 + 9.6 * gamma_z / 4

        self.Correction_yaw = pwm_yaw
        #self.Correction_yaw = 1500


        #publish data
        msg = Float64()
        msg_2 = Float64()
        msg.data = self.desired_yaw
        msg_2.data = angle_yaw
        self.publisher_desired_yaw.publish(msg)
        self.publisher_measured_yaw.publish(msg_2)

    def cubic_trajectory(self, z_init, z_final, t):
        t_final = 5.0

        a2 = (3 * (z_final - z_init)) / t_final**2
        a3 = (-2 * (z_final - z_init)) / t_final**3

        if t < t_final:
            z_desired = z_init + a2 * t**2 + a3 * t**3
            z_dot_desired = z_init + 2 * a2 * t + 3 * a3 * t**2
        else:
            z_desired = z_final
            z_dot_desired = 0     
        # self.get_logger().info("cubic func: "+str(z_init)+" "+str(z_final)+" "+str(t))
        return z_desired, z_dot_desired
    
    def RelAltCallback(self, data):
        if (self.init_p0):
            # 1st execution, init
            self.depth_p0 = data
            self.current_t = 0
            self.integral_error = 0
            self.init_p0 = False
        # setup depth servo control here
        # TODO Task2:
        # fz = -(self.flotability)

        # TODO Task3:
        Kp = 20.0
        # desired_depth = self.desired_depth
        # TODO Task5: call cubic trajectory to get the desired depth 
        desired_depth, desired_dot_depth = self.cubic_trajectory(self.depth_p0.data, self.desired_depth, self.current_t)
        # self.get_logger().info("this is the desired depth: "+str(desired_depth))
        # self.get_logger().info("this is the desired heave: "+str(desired_dot_depth))
        # calculate the needed thrust from the depth error
        depth_error = data.data-desired_depth
        fz = -(Kp * depth_error + self.flotability)
        # self.get_logger().info("this is the error: "+str(depth_error))

        # TODO Task 7: Ki control
        Ki = 0.02
        sampling_time = 1/25 # corresponding to 25Hz: depth publishing frequency
        self.integral_error = self.integral_error + depth_error*sampling_time
        # self.get_logger().info("this is the integral error: "+str(self.integral_error))
        fz =  fz - Ki*self.integral_error # it is a minus sign because we provide negative values
        

        # TODO Task 9: alpha beta estimator
        w_estimate = self.AlfaBetaFilter(data.data)
        # self.get_logger().info("this is the velocity estimate: "+str(w_estimate))


        # TODO Task 10: full PID
        Kd = 5 # to tune
        fz = fz - Kd*(w_estimate - desired_dot_depth)

        if fz<0:
            pwm_z = 1464 + 12.3*fz/4 ## devide by 4 for each thruster
        else:
            pwm_z = 1536 + 9.6*fz/4


        # update Correction_depth
        Correction_depth = pwm_z
        Correction_depth = 1500
        self.Correction_depth = int(Correction_depth)
        # Send PWM commands to motors in timer

        # #### publish data for plots
        # msg = Float64()
        # msg_h = Twist()
        # msg_h_e = Twist()
        # msg.data = desired_depth
        # # msg_h.data = desired_dot_depth
        # # msg_h_e.data = w_estimate
        # msg_h.linear.z = float(desired_dot_depth)  # Use .linear.z or another component
        # msg_h_e.linear.z = float(w_estimate)  # Use .linear.z or another component
        # if desired_dot_depth is None:
        #     self.get_logger().info("desired_dot_depth type error")
        #     self.Correction_depth = 1500
        #     return
        # if w_estimate is None:
        #     self.get_logger().info("w_estimate type error")
        #     self.Correction_depth = 1500
        #     return
        # self.publisher_desired_depth.publish(msg)
        # self.publisher_desired_heave.publish(msg_h)
        # self.publisher_estimated_heave.publish(msg_h_e)

    def AlfaBetaFilter(self, zvalue):
        # Sensor measurement
        self.z_sensor = zvalue  # Sensor measurement

        # Prediction step
        self.z_filter = self.z_filter_1 + (self.w_filter_1 * self.dt_alpha_beta)
        self.w_filter = self.w_filter_1

        # Compute prediction error
        self.pred_error = self.z_sensor - self.z_filter

        # Correction step
        self.z_filter += self.alfa * self.pred_error
        self.w_filter += (self.beta * self.pred_error) / self.dt_alpha_beta

        # Update previous state
        self.z_filter_1 = self.z_filter
        self.w_filter_1 = self.w_filter

        return self.w_filter

    # works but at 4hz compared with 25 Hz for global_position/rel_alt !
    # /uas1/mavlink_source runs at more than 500 Hz !    
    # try with pyvmavlink using an udp connection, to see if gain in hz                 
    def pingerCallback(self, data):
        self.pinger_distance = data.data[0]
        self.pinger_confidence = data.data[1]

        # self.get_logger().info("pinger_distance = " + str(self.pinger_distance) + 
                            #    "with pinger_confidence = " + str(self.pinger_confidence))
        self.stop_distance = 1.3 # 1meter
        self.min_confidence = 0.7 # 70% enough? too less? 
        Kp = 0.1
        surge_error = self.pinger_distance - self.stop_distance
        fz = (Kp * surge_error)
        # self.get_logger().info("this is the error: "+str(depth_error))

      
        if fz<0:
            pwm_z = 1464 + 12.3*fz/4 ## devide by 4 for each thruster
        else:
            pwm_z = 1536 + 9.6*fz/4


        # update Correction_depth
        Correction_surge = pwm_z
        # Correction_surge = 1500
        
        self.Correction_surge = int(Correction_surge)


        if self.pinger_confidence > self.min_confidence:
            if self.pinger_distance <= self.stop_distance:
                self.Correction_surge = 1500

                self.yaw_at_press =+ 0.1
                if self.yaw_at_press >  math.pi :
                    self.yaw_at_press = - math.pi
                # self.get_logger().info("Obstacle detected in 1meter! Stop and turn right")

    # self.get_logger().info("pinger_distance =" + str(self.pinger_distance))

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

        self.subrel_alt = self.create_subscription(Float64, "global_position/rel_alt", self.RelAltCallback,
                                                   qos_profile=qos_profile)
        self.subrel_alt  # prevent unused variable warning
        # self.subwater_pressure = self.create_subscription(Mavlink,"mavlink/from", self.mavlink_callback, qos_profile = qos_profile)
        # self.subwater_pressure = self.create_subscription(Mavlink,"/uas1/mavlink_source", self.mavlink_callback, qos_profile = qos_profile)
        # self.subwater_pressure # prevent unused variable warning

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
