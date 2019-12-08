#!/usr/bin/env python

import socket
import threading
import sys
import time
import json
import rospy
import tf
from std_msgs.msg import Empty, Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2

from mavros_msgs.srv import *
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix
import numpy as np


# import ROSLink constants 
from enums.ROSLINK_VERSION import ROSLINK_VERSION
from enums.ROS_VERSION import ROS_VERSION
from enums.ROBOT_TYPE import ROBOT_TYPE
from enums.ROBOT_STATE import ROBOT_STATE
from enums.MESSAGE_TYPE import MESSAGE_TYPE
from enums.ROBOT_MODE import ROBOT_MODE

# import ROSLink messages
from messages.ROSLinkHeader import ROSLinkHeader 
from messages.HeartBeat import HeartBeat
from messages.RobotStatus import RobotStatus
from messages.GlobalMotion import GlobalMotion
from messages.GPSRawInfo import GPSRawInfo
from messages.RangeFinderData import RangeFinderData

# import configuration parameters
from configuration.CONFIG import *
from configuration.ROSLINK_MESSAGE_PERIOD import *

# import state variables 
from states.ROSLinkStateVariables import ROSLinkStateVariables

voltage = 0
current = 0
remaining = 0

class ROSLinkBridgeGapter:
        
    '''''''''''''''''''''''''''''''''''''''''''''
    Represents ROSLink Bridge for Gapter
    '''''''''''''''''''''''''''''''''''''''''''''
    def __init__(self):
        '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
        Constructor: read and initialize robot and roslink bridge parameters
        '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
        # initialize ROS node for this client
        rospy.init_node('roslink_bridge_gapter_node', anonymous=True) 
        
        # init the parameters from launch file
        ROSLinkBridgeGapter.init_params()
        
        #start ROS publishers
        ROSLinkBridgeGapter.start_ros_publishers()
        
        #start ROS publishers
        ROSLinkBridgeGapter.start_ros_subscribers()
        
        #start UDP server socket
        ROSLinkBridgeGapter.init_servers() 
        
        #create threads for sending roslink messages
        ROSLinkBridgeGapter.create_roslink_message_threads()
        
        #create threads for processing received roslink command messages
        ROSLinkBridgeGapter.create_roslink_command_processing_thread()
    
        #sping ROS forever
        rospy.spin()
    
    @staticmethod
    def init_servers():
        ROSLinkBridgeGapter.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #create server socket
        ROSLinkBridgeGapter.client_socket.settimeout(2) #set a timeout on a blocking socket operation
        ROSLinkBridgeGapter.gcs_server_ip = rospy.get_param("/ground_station_ip", "192.168.100.17")
	#ROSLinkBridgeGapter.gcs_server_ip = rospy.get_param("/ground_station_ip", "208.113.133.197")
        ROSLinkBridgeGapter.gcs_server_port =rospy.get_param("/ground_station_port", 25500)
        ROSLinkBridgeGapter.server_address  = ( ROSLinkBridgeGapter.gcs_server_ip, ROSLinkBridgeGapter.gcs_server_port)
        print ROSLinkBridgeGapter.gcs_server_ip
        print ROSLinkBridgeGapter.gcs_server_port 
        #while True:
        #    print 'sending ...'
        #    self.client_socket.sendto(json.dumps(self.build_heartbeat_message()), self.server_address)
        #    time.sleep(1)   
           
    @staticmethod
    def init_params():   
        rospy.loginfo('[ROSLink Bridge] reading initialization parameters')
        # get roslink version
        ROSLinkStateVariables.roslink_version = rospy.get_param("/roslink_version", ROSLINK_VERSION.ABUBAKR)  
        # get ROS version  
        ROSLinkStateVariables.ros_version = rospy.get_param("/ros_version", ROS_VERSION.INDIGO)    
        # get system id
        ROSLinkStateVariables.system_id = rospy.get_param("/system_id", 15)
        # get robot name
        ROSLinkStateVariables.robot_name = rospy.get_param("/robot_name", "Gapter")
        # get robot type
        ROSLinkStateVariables.type = rospy.get_param("/type", ROBOT_TYPE.ROBOT_TYPE_GENERIC)
        # get owner id
        ROSLinkStateVariables.owner_id = rospy.get_param("/owner_id", 3)
        # get key
        ROSLinkStateVariables.key = rospy.get_param("/key", "1243-0000-0000-FGFG")
        
        # define periods of updates
        ROSLinkBridgeGapter.heartbeat_msg_rate = rospy.get_param("/heartbeat_msg_period", ROSLINK_MESSAGE_PERIOD.ROSLINK_HEARTBEAT_MESSAGE_RATE)
        ROSLinkBridgeGapter.robot_status_msg_rate = rospy.get_param("/robot_status_msg_period", ROSLINK_MESSAGE_PERIOD.ROSLINK_ROBOT_STATUS_MESSAGE_RATE)
        ROSLinkBridgeGapter.global_motion_msg_rate = rospy.get_param("/global_motion_msg_period", ROSLINK_MESSAGE_PERIOD.ROSLINK_GLOBAL_MOTION_MESSAGE_RATE)
        ROSLinkBridgeGapter.gps_raw_info_msg_rate = rospy.get_param("/gps_raw_info_msg_period", ROSLINK_MESSAGE_PERIOD.ROSLINK_GPS_RAW_INFO_MESSAGE_RATE)
        ROSLinkBridgeGapter.range_finder_data_msg_rate = rospy.get_param("/range_finder_data_msg_period", ROSLINK_MESSAGE_PERIOD.ROSLINK_RANGE_FINDER_DATA_MESSAGE_RATE)
        
        ROSLinkBridgeGapter.bridge = CvBridge()
        
    #start ROS publishers 
    @staticmethod
    def start_ros_publishers():
        # ROS publishers: for executing ROSLink commands
        #ROSLinkBridgeGapter.takeoff_publisher = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=10)
        #ROSLinkBridgeGapter.land_publisher    = rospy.Publisher('/ardrone/land', Empty, queue_size=10)
        #ROSLinkBridgeGapter.reset_publisher   = rospy.Publisher('/ardrone/reset',Empty, queue_size=10)
        #ROSLinkBridgeGapter.move_publisher    = rospy.Publisher('/cmd_vel',Twist, queue_size=10)    
     
        ROSLinkBridgeGapter.move_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

    #start ROS subscribers
    @staticmethod
    def start_ros_subscribers():
        #rospy.Subscriber("/ground_truth/state", Odometry, ROSLinkBridgeGapter.odometryCallback)
        #rospy.Subscriber("/ardrone/navdata", Navdata, ROSLinkBridgeGapter.navdataCallback)
        #rospy.Subscriber("/ardrone/front/image_raw/compressed", CompressedImage, ROSLinkBridgeGapter.frontCompressedImageCallback)
        #rospy.Subscriber("/ardrone/front/image_raw", Image, ROSLinkBridgeGapter.frontImageCallback)
        rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, ROSLinkBridgeGapter.globalPositionCallback)
	rospy.Subscriber("/mavros/global_position/rel_alt", Float64, ROSLinkBridgeGapter.altitudeCallback)
	rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, ROSLinkBridgeGapter.headingCallback)    
	rospy.Subscriber("/mavros/BatteryStatus", BatteryState, ROSLinkBridgeGapter.batteryStateCallback)    




    @staticmethod 
    def setGuidedMode():
        rospy.wait_for_service('/mavros/set_mode')
        try:
        	flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        	#http://wiki.ros.org/mavros/CustomModes for custom modes
        	isModeChanged = flightModeService(custom_mode='GUIDED') #return true or false
        except rospy.ServiceException, e:
        	print "service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled"%e
    
    @staticmethod     
    def setStabilizeMode():
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
            #http://wiki.ros.org/mavros/CustomModes for custom modes
            isModeChanged = flightModeService(custom_mode='STABILIZE') #return true or false
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. v Mode could not be set"%e

    @staticmethod 
    def setLandMode():
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
            #http://wiki.ros.org/mavros/CustomModes for custom modes
            isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
        except rospy.ServiceException, e:
            print "service land call failed: %s. The vehicle cannot land "%e
    
    @staticmethod           
    def setArm():
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arm call failed: %s"%e
      
    @staticmethod     
    def setDisarm():
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service Disarm call failed: %s"%e
    
    @staticmethod 
    def setTakeoffMode(user_altitude):
        print "Test take off"
        rospy.wait_for_service('/mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL) 
            takeoffService(altitude = user_altitude, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
        except rospy.ServiceException, e:
            print "Service takeoff call failed: %s"%e

    @staticmethod
    def globalPositionCallback(globalPositionCallback):
        ROSLinkStateVariables.lat = globalPositionCallback.latitude
        ROSLinkStateVariables.lon = globalPositionCallback.longitude

    @staticmethod
    def altitudeCallback(relativeAltMsg):
	ROSLinkStateVariables.alt = relativeAltMsg.data

    @staticmethod
    def headingCallback(headingMsg):
	ROSLinkStateVariables.yaw = headingMsg.data

    @staticmethod
    def batteryStateCallback(batteryStateMsg):
	global voltage
	global current
	global remaining
	ROSLinkStateVariables.battery = batteryStateMsg.percentage
	voltage = batteryStateMsg.voltage
	current = batteryStateMsg.current
	remaining = batteryStateMsg.remaining



    @staticmethod   
    def odometryCallback(msg):
        #position 
        ROSLinkStateVariables.time_boot_ms=time.time()
        ROSLinkStateVariables.x= msg.pose.pose.position.x
        ROSLinkStateVariables.y= msg.pose.pose.position.y
        ROSLinkStateVariables.z= msg.pose.pose.position.z
        #orientation
        quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        ROSLinkStateVariables.roll = euler[0]
        ROSLinkStateVariables.pitch = euler[1]
        ROSLinkStateVariables.yaw = euler[2]
        #twist: linear
        ROSLinkStateVariables.vx_truth = msg.twist.twist.linear.x
        ROSLinkStateVariables.vy_truth = msg.twist.twist.linear.y
        ROSLinkStateVariables.vz_truth = msg.twist.twist.linear.z
        #twist: angular
        ROSLinkStateVariables.wx_truth = msg.twist.twist.angular.x
        ROSLinkStateVariables.wy_truth = msg.twist.twist.angular.y
        ROSLinkStateVariables.wz_truth = msg.twist.twist.angular.z
        
        #print self.x
        #print self.y
     
    @staticmethod   
    def navdataCallback(msg):
        ROSLinkStateVariables.time_boot_ms=time.time()
        ROSLinkStateVariables.seq_number = ROSLinkStateVariables.seq_number +1;
        ROSLinkStateVariables.vx = msg.vx
        ROSLinkStateVariables.vy = msg.vy
        ROSLinkStateVariables.vz = msg.vz
        ROSLinkStateVariables.wx = msg.ax
        ROSLinkStateVariables.wy = msg.ay
        ROSLinkStateVariables.wz = msg.az
        ROSLinkStateVariables.battery = msg.batteryPercent
        ROSLinkStateVariables.state = msg.state
        ROSLinkStateVariables.magX = msg.magX
        ROSLinkStateVariables.magY = msg.magY
        ROSLinkStateVariables.magZ = msg.magZ
        ROSLinkStateVariables.pressure = msg.pressure
        ROSLinkStateVariables.temp = msg.temp
        ROSLinkStateVariables.wind_speed = msg.wind_speed
        ROSLinkStateVariables.wind_angle = msg.wind_angle
        ROSLinkStateVariables.rotX = msg.rotX
        ROSLinkStateVariables.rotY = msg.rotY
        ROSLinkStateVariables.rotZ = msg.rotZ
        ROSLinkStateVariables.altitude = msg.altd
        ROSLinkStateVariables.motor1 = msg.motor1
        ROSLinkStateVariables.motor2 = msg.motor2
        ROSLinkStateVariables.motor3 = msg.motor3
        ROSLinkStateVariables.motor4 = msg.motor4
        ROSLinkStateVariables.tags_count = msg.tags_count
        ROSLinkStateVariables.tags_type = msg.tags_type
        
        
        
    
    #this method create threads for sending roslink message at specific rate
    @staticmethod
    def create_roslink_message_threads(): 
        ROSLinkBridgeGapter.heatrbeat_message_thread = ROSLinkMessageThread(ROSLinkBridgeGapter.client_socket, ROSLinkBridgeGapter.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_HEARTBEAT, "heatrbeat_message_thread", ROSLinkBridgeGapter.heartbeat_msg_rate)
        ROSLinkBridgeGapter.robot_status_thread = ROSLinkMessageThread(ROSLinkBridgeGapter.client_socket, ROSLinkBridgeGapter.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_ROBOT_STATUS, "robot_status_thread", ROSLinkBridgeGapter.robot_status_msg_rate)
        ROSLinkBridgeGapter.global_motion_thread = ROSLinkMessageThread(ROSLinkBridgeGapter.client_socket, ROSLinkBridgeGapter.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_GLOBAL_MOTION, "global_motion_thread", ROSLinkBridgeGapter.global_motion_msg_rate)
        ROSLinkBridgeGapter.gps_raw_info_thread = ROSLinkMessageThread(ROSLinkBridgeGapter.client_socket,  ROSLinkBridgeGapter.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_GPS_RAW_INFO, "gps_raw_info_thread", ROSLinkBridgeGapter.gps_raw_info_msg_rate)
        #ROSLinkBridgeGapter.range_finder_data_thread = ROSLinkMessageThread(ROSLinkBridgeGapter.client_socket, ROSLinkBridgeGapter.server_address, "range_finder_data_thread", 0.333)
        
    @staticmethod
    def create_roslink_command_processing_thread(): 
        ROSLinkBridgeGapter.command_processing_thread =  ROSLinkCommandProcessingThread(ROSLinkBridgeGapter.client_socket, 'ROSLink Command Processing Thread')
    
    @staticmethod
    def static_build_roslink_header_message(message_id):
        header = ROSLinkHeader(ROSLinkStateVariables.roslink_version, ROSLinkStateVariables.ros_version, ROSLinkStateVariables.system_id, message_id, ROSLinkStateVariables.sequence_number,ROSLinkStateVariables.key)
        ROSLinkStateVariables.sequence_number = ROSLinkStateVariables.sequence_number + 1
        return header.__dict__
    
    @staticmethod 
    def static_build_heartbeat_message():
        message_header = ROSLinkBridgeGapter.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_HEARTBEAT)
        heartbeat_message = HeartBeat(message_header, ROBOT_TYPE.ROBOT_TYPE_GENERIC, ROSLinkStateVariables.robot_name, ROBOT_STATE.ROBOT_STATE_ACTIVE, ROSLinkStateVariables.owner_id ,ROBOT_MODE.ROBOT_STATE_UNKNOWN)
        return heartbeat_message.__dict__
    
    @staticmethod 
    def static_build_robot_status_message():
	global voltage
	global current
	global remaining
        message_header = ROSLinkBridgeGapter.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_ROBOT_STATUS)
        robot_status_message = RobotStatus(message_header, [], [], voltage, current , remaining)
        return robot_status_message.__dict__

    @staticmethod
    def static_build_global_motion_message():
        message_header = ROSLinkBridgeGapter.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_GLOBAL_MOTION) 
        global_motion_message = GlobalMotion(message_header, ROSLinkStateVariables.time_boot_ms , ROSLinkStateVariables.x, ROSLinkStateVariables.y, ROSLinkStateVariables.yaw, ROSLinkStateVariables.vx, ROSLinkStateVariables.vy, ROSLinkStateVariables.vz, ROSLinkStateVariables.wx, ROSLinkStateVariables.wy, ROSLinkStateVariables.wz, ROSLinkStateVariables.pitch, ROSLinkStateVariables.roll, ROSLinkStateVariables.yaw)
        return global_motion_message.__dict__  
    
    @staticmethod
    def static_build_gps_raw_info_message():
        message_header = ROSLinkBridgeGapter.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_GPS_RAW_INFO) 
        global_motion_message = GPSRawInfo(message_header, ROSLinkStateVariables.time_boot_ms , ROSLinkStateVariables.fix_type, ROSLinkStateVariables.lat, ROSLinkStateVariables.lon, ROSLinkStateVariables.alt, ROSLinkStateVariables.eph, ROSLinkStateVariables.epv, ROSLinkStateVariables.vel, ROSLinkStateVariables.cog, ROSLinkStateVariables.satellites_visible)
        return global_motion_message.__dict__  
    
    
    @staticmethod
    def process_roslink_command_message(msg):
        #print 'msg is ', msg 
        command = json.loads(msg)
        print 'ROSLink command received ..'
        print msg
        
        if command['header']['message_id'] == MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_TAKEOFF:  
            print 'I received Takeoff command' 
            print '\n\nThe robot is Taking off with altitude',command['altitude']  , '\n\n'  
               
            ROSLinkBridgeGapter.setTakeoffMode(command['altitude'])
   
        elif command['header']['message_id'] == MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_LAND:
            print 'I received Land command' 
            print '\n\nThe robot is landing\n\n'
            
            ROSLinkBridgeGapter.setLandMode() 
            
        elif command['header']['message_id'] == 104:
            print 'I received Arm command' 
            print 'APM: ARMING MOTORS'
            ROSLinkBridgeGapter.setArm() 
            
        elif command['header']['message_id'] == 105:
            print 'I received Disarm command' 
            print 'DISARMED: DISARMING MOTORS'
            ROSLinkBridgeGapter.setDisarm() 
            
        elif command['header']['message_id'] == 106:
            print 'I received change mode command' 
            print 'GUIDED> Mode GUIDED'
            ROSLinkBridgeGapter.setGuidedMode() 
            
        elif command['header']['message_id'] == 107:
            print 'I received change mode  command' 
            print 'STABILIZE> Mode STABILIZE'
            ROSLinkBridgeGapter.setStabilizeMode() 
            
        if command['header']['message_id'] == MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_TWIST:  
            print 'I received Twist command successfully'
            TwistCommand = TwistStamped() 
            TwistCommand.twist.linear.x = command['vx']
            TwistCommand.twist.linear.y = command['vy'] 
            TwistCommand.twist.linear.z = command['vz'] 
            TwistCommand.twist.angular.x = command['wx']
            TwistCommand.twist.angular.y = command['wy'] 
            TwistCommand.twist.angular.z = command['wz']             
            print TwistCommand
            ROSLinkBridgeGapter.move_publisher.publish (TwistCommand)
  
        # finds what kinds of message in that data  
        '''
        if command['header']['message_id'] == 100:  
            print 'I received Move command successfully'
            TwistCommand = Twist() 
            TwistCommand.linear.x = command['vx']
            TwistCommand.linear.y = command['vy'] 
            TwistCommand.linear.z = command['vz'] 
            TwistCommand.angular.x = command['wx']
            TwistCommand.angular.y = command['wy'] 
            TwistCommand.angular.z = command['wz']             
            #print TwistCommand
            #pubMove.publish (TwistCommand)
              
        if command['header']['message_id'] == 101:  
            print 'I received GO tO Waypoint command successfully'
            print 'But no topic published!!' 
   
        elif command['header']['message_id'] == 103:
            print 'I received Land command' 
            print '\n\nThe robot is landing\n\n'
            #pub_land.publish(Empty())    
        '''

class ROSLinkMessageThread ():
    count = 0
    def __init__(self, sock, server_address,message_type ,thread_name='noname', data_rate=1.0):
        self.count = self.count +1
        self.name = thread_name
        self.socket = sock
        self.server_address = server_address
        self.data_rate = data_rate
        self.roslink_message_type = message_type
        t = threading.Thread(target=self.run)
        t.setName(thread_name)
        t.start()
    
    def run ( self ):
        while True:
            self.count=self.count+1
            time.sleep(1.0/self.data_rate)
            print 'thread %s %d\n'%(self.name, self.count)
            #self.send(self.socket, json.dumps(self.roslink_message.__dict__))
            if (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_HEARTBEAT):
		#print ROSLinkBridgeGapter.static_build_heartbeat_message(), '\n' 
                self.send(self.socket, json.dumps(ROSLinkBridgeGapter.static_build_heartbeat_message()))
            elif (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_ROBOT_STATUS):
		#print ROSLinkBridgeGapter.static_build_robot_status_message(), '\n' 
                self.send(self.socket, json.dumps(ROSLinkBridgeGapter.static_build_robot_status_message()))
            elif (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_GLOBAL_MOTION):
		#print ROSLinkBridgeGapter.static_build_global_motion_message(), '\n' 
                self.send(self.socket, json.dumps(ROSLinkBridgeGapter.static_build_global_motion_message()))
            elif (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_GPS_RAW_INFO):
		#print ROSLinkBridgeGapter.static_build_gps_raw_info_message(), '\n' 
                self.send(self.socket, json.dumps(ROSLinkBridgeGapter.static_build_gps_raw_info_message()))

    '''
        Sending method
    '''
    def send (self, sock , msg): 
        self.socket.sendto(msg, self.server_address)   




class ROSLinkCommandProcessingThread ( ):
    def __init__(self, sock,thread_name='noname'):
        self.name = thread_name
        self.socket = sock
        t = threading.Thread(target=self.run)
        t.setName(thread_name)
        t.start()
    def run ( self):
        print "Start ROSLINK Command Processing Thread"
        while True:
            try:
                msg, address = self.socket.recvfrom(MESSAGE_MAX_LENGTH)
                ROSLinkBridgeGapter.process_roslink_command_message(msg)
            except socket.timeout:
                continue   


if __name__ == '__main__':
    print '\n************** Starting ROSLink Bridge for Gapter **************\n' 
    # initialize ROS node for this client
    #rospy.init_node('udp_client_drone_node', anonymous=True) 
    myDroneBridge = ROSLinkBridgeGapter() 
