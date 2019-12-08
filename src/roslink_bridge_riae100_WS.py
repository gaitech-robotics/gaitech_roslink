#!/usr/bin/env python

import websocket
from websocket import create_connection , WebSocket # pip install websocket-client
import thread

import socket
import threading
import sys
import time
import json
import rospy
import tf
# from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

from sensor_msgs.msg import Image, CameraInfo, CompressedImage

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped

from std_msgs.msg import String
import os
import cv2
import numpy as np
import base64
from cv_bridge import CvBridge, CvBridgeError


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
from messages.RoslinkImage import RoslinkImage
from messages.RoslinkMap import RoslinkMap


# import configuration parameters
from configuration.CONFIG import *
from configuration.ROSLINK_MESSAGE_PERIOD import *

# import state variables 
from states.ROSLinkStateVariables import ROSLinkStateVariables

debug=False


    
class ROSLinkBridgeRIA_E100:
        
    '''''''''''''''''''''''''''''''''''''''''''''
    Represents ROSLink Bridge for AR Drone
    '''''''''''''''''''''''''''''''''''''''''''''
    def __init__(self):
        '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
        Constructor: read and initialize robot and roslink bridge parameters
        '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
        
        # init the parameters from launch file
        ROSLinkBridgeRIA_E100.init_params()
        
        if (ROSLinkStateVariables.key == "NOKEY"):
            print("ROBOT KEY HAS NOT BEEN SET. PROGRAM WILL EXIT. SET YOUR ROBOT KEY BEFORE STARTING THE ROSLINK BRIDGE")
            sys.exit()
        if (ROSLinkStateVariables.owner_id == -1):
            print("OWNER ID NOT BEEN SET. PROGRAM WILL EXIT. SET YOUR OWNER ID BEFORE STARTING THE ROSLINK BRIDGE")
            sys.exit()
        # initialize ROS node for this client
        rospy.init_node('roslink_bridge_ardrone_node'+ROSLinkStateVariables.key, anonymous=True) 
        
        #start ROS publishers
        ROSLinkBridgeRIA_E100.start_ros_publishers()
        
        #start ROS publishers
        ROSLinkBridgeRIA_E100.start_ros_subscribers()
        
        #start UDP server socket
        ROSLinkBridgeRIA_E100.init_servers() 
        
        #create threads for sending roslink messages
        ROSLinkBridgeRIA_E100.create_roslink_message_threads()
        
        #create threads for processing received roslink command messages
        ROSLinkBridgeRIA_E100.create_roslink_command_processing_thread()
    
        #sping ROS forever
        rospy.spin()
    
    @staticmethod
    def init_servers():
        ROSLinkBridgeRIA_E100.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #create server socket
        ROSLinkBridgeRIA_E100.client_socket.settimeout(2) #set a timeout on a blocking socket operation
        #ROSLinkBridgeRIA_E100.gcs_server_ip = rospy.get_param("/ground_station_ip", "127.0.0.1")
        #ROSLinkBridgeRIA_E100.gcs_server_ip = rospy.get_param("/ground_station_ip", "192.168.100.17")
        #ROSLinkBridgeRIA_E100.gcs_server_port =rospy.get_param("/ground_station_port", 10000)
        ROSLinkBridgeRIA_E100.gcs_server_ip = rospy.get_param("/ground_station_ip", "208.113.133.197")
        ROSLinkBridgeRIA_E100.gcs_server_port =rospy.get_param("/ground_station_port", 25500)
        ROSLinkBridgeRIA_E100.server_address  = ( ROSLinkBridgeRIA_E100.gcs_server_ip, ROSLinkBridgeRIA_E100.gcs_server_port)
        print ROSLinkBridgeRIA_E100.gcs_server_ip
        print ROSLinkBridgeRIA_E100.gcs_server_port 
        
        ROSLinkStateVariables.ws_timer = 0
        ROSLinkBridgeRIA_E100.WSconnected=False
        ROSLinkBridgeRIA_E100.ws = None;

        
        t = threading.Thread(target=ROSLinkBridgeRIA_E100.websocketThread)
        t.setName("websocket thread")
        t.start()
       
    
    


    @staticmethod
    def websocketThread():
        if(not ROSLinkBridgeRIA_E100.WSconnected):
            # this for public network
            WS_url = "ws://"+ ROSLinkBridgeRIA_E100.gcs_server_ip+":9090/websockets/roslink/user/"+str(ROSLinkStateVariables.owner_id) +"/robot/"+ROSLinkStateVariables.key+"/robot" 
            # WS_url = "ws://"+ ROSLinkBridgeRIA_E100.gcs_server_ip+":9090/";
            ROSLinkBridgeRIA_E100.ws = websocket.WebSocketApp(WS_url,
                                on_message = ROSLinkBridgeRIA_E100.on_message,
                                on_error = ROSLinkBridgeRIA_E100.on_error,
                                on_close = ROSLinkBridgeRIA_E100.on_close)
            ROSLinkBridgeRIA_E100.ws.on_open = ROSLinkBridgeRIA_E100.on_open
            
            # ROSLinkBridgeRIA_E100.WSconnected=True;
            ROSLinkBridgeRIA_E100.ws.run_forever()
        


    @staticmethod
    def handel_websocket_closing():
        print("***************reconnecting to WS server*****************")
        t = threading.Thread(target=ROSLinkBridgeRIA_E100.websocketThread)
        t.setName("websocket thread")
        t.start()



    

    @staticmethod
    def on_close(ws):   
        ROSLinkStateVariables.WSconnected =False;
        print("websocket connection closed");
        ROSLinkStateVariables.ws_timer+=1;
        print("try to re connect after",ROSLinkStateVariables.ws_timer)
        time.sleep(ROSLinkStateVariables.ws_timer)
        ROSLinkBridgeRIA_E100.handel_websocket_closing();


    @staticmethod
    def on_open(ws):
        ROSLinkStateVariables.ws_timer=0;
        print("websocket connection opened");
        time.sleep(1) # to make sure no thread is sending now
        ROSLinkStateVariables.WSconnected =True;


    @staticmethod
    def on_message(ws, message):
        ROSLinkBridgeRIA_E100.process_roslink_command_message(message)



    @staticmethod
    def on_error(ws, error):   
        print("websocket error ", error)



    @staticmethod
    def init_params():   
        rospy.loginfo('[ROSLink Bridge] reading initialization parameters')
        # get roslink version
        ROSLinkStateVariables.roslink_version = rospy.get_param("/roslink_version", ROSLINK_VERSION.ABUBAKR)  
        # get ROS version  
        ROSLinkStateVariables.ros_version = rospy.get_param("/ros_version", ROS_VERSION.INDIGO)    
        # get system id
        ROSLinkStateVariables.system_id = rospy.get_param("/system_id", 12)
        # get robot name
        ROSLinkStateVariables.robot_name = rospy.get_param("/robot_name", "RIA_E100")
        # get robot type
        ROSLinkStateVariables.type = rospy.get_param("/type", ROBOT_TYPE.ROBOT_TYPE_RIA_E100)
        # get owner id
        ROSLinkStateVariables.owner_id = rospy.get_param("/owner_id", "26")
        # get key
        ROSLinkStateVariables.key = rospy.get_param("/key", "ABCD")
        #get map data
        ROSLinkStateVariables.map_location = rospy.get_param("/map_location", "ABCD")
        ROSLinkStateVariables.map_xoffset = rospy.get_param("/map_xoffset", 0)
        ROSLinkStateVariables.map_yoffset = rospy.get_param("/map_yoffset", 0)



        ROSLinkStateVariables.WSconnected= False;
        
        # define periods of updates
        ROSLinkBridgeRIA_E100.heartbeat_msg_rate = rospy.get_param("/heartbeat_msg_rate", ROSLINK_MESSAGE_PERIOD.ROSLINK_HEARTBEAT_MESSAGE_RATE)
        ROSLinkBridgeRIA_E100.robot_status_msg_rate = rospy.get_param("/robot_status_msg_rate", ROSLINK_MESSAGE_PERIOD.ROSLINK_ROBOT_STATUS_MESSAGE_RATE)
        ROSLinkBridgeRIA_E100.global_motion_msg_rate = rospy.get_param("/global_motion_msg_rate", ROSLINK_MESSAGE_PERIOD.ROSLINK_GLOBAL_MOTION_MESSAGE_RATE)
        ROSLinkBridgeRIA_E100.gps_raw_info_msg_rate = rospy.get_param("/gps_raw_info_msg_rate", ROSLINK_MESSAGE_PERIOD.ROSLINK_GPS_RAW_INFO_MESSAGE_RATE)
        ROSLinkBridgeRIA_E100.range_finder_data_msg_rate = rospy.get_param("/range_finder_data_msg_rate", ROSLINK_MESSAGE_PERIOD.ROSLINK_RANGE_FINDER_DATA_MESSAGE_RATE)
        ROSLinkBridgeRIA_E100.ROSLing_Image_msg_rate = rospy.get_param("/ROSLing_Image_msg_rate", 1)
        ROSLinkBridgeRIA_E100.TwistCommand = Twist()
        ROSLinkBridgeRIA_E100.sendingTwistFlag=True
        ROSLinkBridgeRIA_E100.bridge = CvBridge()
        ROSLinkBridgeRIA_E100.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)


    #start ROS publishers 
    @staticmethod
    def start_ros_publishers():
        # ROS publishers: for executing ROSLink commands
        ROSLinkBridgeRIA_E100.move_publisher = rospy.Publisher('/teleop/cmd_vel',Twist, queue_size=10)    

    #start ROS subscribers
    @staticmethod
    def start_ros_subscribers():
        # rospy.Subscriber("/odom", Odometry, ROSLinkBridgeRIA_E100.odometryCallback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, ROSLinkBridgeRIA_E100.amclCallback)
        # rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, ROSLinkBridgeRIA_E100.compressedImageCallback)
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, ROSLinkBridgeRIA_E100.compressedImageCallback)

    
    @staticmethod
    def frontImageCallback(data):
        data = str(data)
        print(data[0:5])
        print(len(data))
        if(len(data)<65000):
            ROSLinkStateVariables.ROSLinkImageData=data[7:len(data)-1]
            return;
        else:
            ROSLinkStateVariables.ROSLinkImageData=data[:65000]
            print('\n\n\n image too long it will be cut \n\n\n')

        print(len(ROSLinkStateVariables.ROSLinkImageData))
        # cv2.imshow("ARDrone Front Image Viewer", cv_image)
        # cv2.waitKey(3)

    @staticmethod   
    def compressedImageCallback(data):
        try:
          #cv_image = ROSLinkBridgeARDrone.bridge.imgmsg_to_cv2(data, "bgr8")
          np_arr = np.fromstring(data.data, np.uint8)
          cv_image = cv2.imdecode(np_arr, cv2.CV_16U)
        except CvBridgeError as e:
          print(e)
        data =  u''+base64.encodestring(np_arr)
        ROSLinkStateVariables.ROSLinkImageData=data
       
        
    @staticmethod   
    def amclCallback(msg):
        #position 
        ROSLinkStateVariables.x= msg.pose.pose.position.x
        ROSLinkStateVariables.y= msg.pose.pose.position.y
        ROSLinkStateVariables.z= msg.pose.pose.position.z

        ROSLinkStateVariables.time_boot_ms=time.time()
        
        # print(ROSLinkStateVariables.x,ROSLinkStateVariables.y,"x,y")
   

    @staticmethod   
    def odometryCallback(msg):
        #position 
        ROSLinkStateVariables.time_boot_ms=time.time()
        ROSLinkStateVariables.x= msg.pose.pose.position.x
        ROSLinkStateVariables.y= msg.pose.pose.position.y
        ROSLinkStateVariables.z= msg.pose.pose.position.z
        # print(ROSLinkStateVariables.x,ROSLinkStateVariables.y,"x,y")
        #orientation
        quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        ROSLinkStateVariables.roll = euler[0]
        ROSLinkStateVariables.pitch = euler[1]
        ROSLinkStateVariables.yaw = euler[2]
        #twist: linear
        ROSLinkStateVariables.vx = msg.twist.twist.linear.x
        ROSLinkStateVariables.vy = msg.twist.twist.linear.y
        ROSLinkStateVariables.vz = msg.twist.twist.linear.z
        #twist: angular
        ROSLinkBridgeRIA_E100.wx = msg.twist.twist.angular.x
        ROSLinkStateVariables.wy = msg.twist.twist.angular.y
        ROSLinkStateVariables.wz = msg.twist.twist.angular.z
             
    
    #this method create threads for sending roslink message at specific rate
    @staticmethod
    def create_roslink_message_threads(): 
        ROSLinkBridgeRIA_E100.heatrbeat_message_thread = ROSLinkMessageThread(ROSLinkBridgeRIA_E100.client_socket, ROSLinkBridgeRIA_E100.ws, ROSLinkBridgeRIA_E100.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_HEARTBEAT, "heatrbeat_message_thread", ROSLinkBridgeRIA_E100.heartbeat_msg_rate)
        ROSLinkBridgeRIA_E100.robot_status_thread = ROSLinkMessageThread(ROSLinkBridgeRIA_E100.client_socket, ROSLinkBridgeRIA_E100.ws, ROSLinkBridgeRIA_E100.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_ROBOT_STATUS, "robot_status_thread", ROSLinkBridgeRIA_E100.robot_status_msg_rate)
        ROSLinkBridgeRIA_E100.global_motion_thread = ROSLinkMessageThread(ROSLinkBridgeRIA_E100.client_socket, ROSLinkBridgeRIA_E100.ws, ROSLinkBridgeRIA_E100.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_GLOBAL_MOTION, "global_motion_thread", ROSLinkBridgeRIA_E100.global_motion_msg_rate)
        ROSLinkBridgeRIA_E100.ROSLinkImage_message_thread = ROSLinkMessageThread(ROSLinkBridgeRIA_E100.client_socket, ROSLinkBridgeRIA_E100.ws,  ROSLinkBridgeRIA_E100.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_ROSLINK_IMAGE, "ROSLink_image_thread", ROSLinkBridgeRIA_E100.ROSLing_Image_msg_rate)
        ROSLinkBridgeRIA_E100.TwistCommandThread = TwistCommandThread()



# ROSLinkBridgeRIA_E100.gps_raw_info_thread = ROSLinkMessageThread(ROSLinkBridgeRIA_E100.client_socket,  ROSLinkBridgeRIA_E100.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_GPS_RAW_INFO, "gps_raw_info_thread", ROSLinkBridgeRIA_E100.gps_raw_info_msg_rate)
        #ROSLinkBridgeRIA_E100.range_finder_data_thread = ROSLinkMessageThread(ROSLinkBridgeRIA_E100.client_socket, ROSLinkBridgeRIA_E100.server_address, "range_finder_data_thread", 0.333)
        
    @staticmethod
    def create_roslink_command_processing_thread(): 
        ROSLinkBridgeRIA_E100.command_processing_thread =  ROSLinkCommandProcessingThread(ROSLinkBridgeRIA_E100.client_socket, 'ROSLink Command Processing Thread')
    
    @staticmethod
    def static_build_roslink_header_message(message_id):
        header = ROSLinkHeader(ROSLinkStateVariables.roslink_version, ROSLinkStateVariables.ros_version, ROSLinkStateVariables.system_id,ROSLinkStateVariables.owner_id, message_id, ROSLinkStateVariables.sequence_number,ROSLinkStateVariables.key)
        ROSLinkStateVariables.sequence_number = ROSLinkStateVariables.sequence_number + 1
        return header.__dict__
    
    @staticmethod 
    def static_build_heartbeat_message():
        
        message_header = ROSLinkBridgeRIA_E100.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_HEARTBEAT)
        heartbeat_message = HeartBeat(message_header, ROBOT_TYPE.ROBOT_TYPE_RIA_E100, ROSLinkStateVariables.robot_name, ROBOT_STATE.ROBOT_STATE_ACTIVE, ROSLinkStateVariables.owner_id ,ROBOT_MODE.ROBOT_STATE_UNKNOWN)
        return heartbeat_message.__dict__
    
    @staticmethod 
    def static_build_robot_status_message():
        message_header = ROSLinkBridgeRIA_E100.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_ROBOT_STATUS)
        robot_status_message = HeartBeat(message_header, 0, ROSLinkStateVariables.robot_name, 0, 0 ,0)
        return robot_status_message.__dict__
    
    @staticmethod
    def static_build_global_motion_message():
        message_header = ROSLinkBridgeRIA_E100.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_GLOBAL_MOTION) 
        global_motion_message = GlobalMotion(message_header, ROSLinkStateVariables.time_boot_ms , ROSLinkStateVariables.x, ROSLinkStateVariables.y, ROSLinkStateVariables.yaw, ROSLinkStateVariables.vx, ROSLinkStateVariables.vy, ROSLinkStateVariables.vz, ROSLinkStateVariables.wx, ROSLinkStateVariables.wy, ROSLinkStateVariables.wz, ROSLinkStateVariables.pitch, ROSLinkStateVariables.roll, ROSLinkStateVariables.yaw)
        return global_motion_message.__dict__  
    
    @staticmethod
    def static_build_gps_raw_info_message():
        message_header = ROSLinkBridgeRIA_E100.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_GPS_RAW_INFO) 
        global_motion_message = GPSRawInfo(message_header, ROSLinkStateVariables.time_boot_ms , ROSLinkStateVariables.fix_type, ROSLinkStateVariables.lat, ROSLinkStateVariables.lon, ROSLinkStateVariables.alt, ROSLinkStateVariables.eph, ROSLinkStateVariables.epv, ROSLinkStateVariables.vel, ROSLinkStateVariables.cog, ROSLinkStateVariables.satellites_visible)
        return global_motion_message.__dict__  

    @staticmethod
    def static_build_ROSLink_image_message():
        message_header = ROSLinkBridgeRIA_E100.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_ROSLINK_IMAGE) 
        ROSLink_image_message = RoslinkImage(message_header, ROSLinkStateVariables.owner_id ,ROSLinkStateVariables.key,ROSLinkStateVariables.ROSLinkImageData,ROSLinkStateVariables.ROSLinkImageHeight,ROSLinkStateVariables.ROSLinkImageWidth)
        return ROSLink_image_message.__dict__ 
    
    @staticmethod
    def move_to_goal(xGoal,yGoal):
        ROSLinkBridgeRIA_E100.sendingTwistFlag=False;
        #define a client for to send goal requests to the move_base server through a SimpleActionClient
        # ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        #wait for the action server to come up
        while(not ROSLinkBridgeRIA_E100.ac.wait_for_server(rospy.Duration.from_sec(5.0))):
               rospy.loginfo("Waiting for the move_base action server to come up")

        goal = MoveBaseGoal()


        #set up the frame parameters
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # moving towards the goal*/

        goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Sending goal location ...")
        ROSLinkBridgeRIA_E100.ac.send_goal(goal)

        ROSLinkBridgeRIA_E100.ac.wait_for_result(rospy.Duration(60))

        if(ROSLinkBridgeRIA_E100.ac.get_state() ==  GoalStatus.SUCCEEDED):
               rospy.loginfo("You have reached the destination")
               ROSLinkBridgeRIA_E100.sendingTwistFlag = True

        else:
               rospy.loginfo("The robot failed to reach the destination")
               ROSLinkBridgeRIA_E100.sendingTwistFlag = True
        ROSLinkBridgeRIA_E100.sendingTwistFlag = True
        


    @staticmethod
    def process_roslink_command_message(msg):
        # print 'msg is ', msg 
        command = json.loads(msg)
        if(debug):
            print 'ROSLink command received ..'
        # print msg

        if command['header']['message_id'] == MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_TWIST:  
            # print 'Twist command received'
            # print(msg)

            ROSLinkBridgeRIA_E100.TwistCommand.linear.x = command['vx']
            ROSLinkBridgeRIA_E100.TwistCommand.linear.y = command['vy'] 
            ROSLinkBridgeRIA_E100.TwistCommand.linear.z = command['vz'] 
            ROSLinkBridgeRIA_E100.TwistCommand.angular.x = command['wx']
            ROSLinkBridgeRIA_E100.TwistCommand.angular.y = command['wy'] 
            ROSLinkBridgeRIA_E100.TwistCommand.angular.z = command['wz']             
            # print TwistCommand
            # ROSLinkBridgeRIA_E100.move_publisher.publish (ROSLinkBridgeRIA_E100.TwistCommand)
            
        elif command['header']['message_id'] == MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_GET_MAP:
            print 'get map command received --------------------------'
            print ROSLinkBridgeRIA_E100.sendingTwistFlag
            # TwistCommandThread()
            try:
                Map = cv2.imread(ROSLinkStateVariables.map_location,1);
                encoded = cv2.imencode('.png',Map)[1]
                
                # file = open(ROSLinkStateVariables.map_location, 'rb')
                # data = base64.b64encode(file.read())
                

                data = u''+base64.encodestring(encoded);
                
                
                message_header = ROSLinkBridgeRIA_E100.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_ROSLINK_MAP)
                map_message = RoslinkMap(message_header, ROSLinkStateVariables.owner_id ,  ROSLinkStateVariables.key, data,200,200,ROSLinkStateVariables.map_xoffset,ROSLinkStateVariables.map_yoffset,None,None,None)
                roslinkmap = json.dumps(map_message.__dict__)

                # ROSLinkBridgeRIA_E100.client_socket.sendto(roslinkmap,ROSLinkBridgeRIA_E100.server_address)
                ROSLinkBridgeRIA_E100.ws.send(roslinkmap)
                return;
            except:
                pass


        elif command['header']['message_id'] == MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_GO_TO_WAYPOINT:
            print 'GO TO WAYPOINT command received --------------------------'
            print(msg)
            print(command)
            print(command['x'])
            print(command["y"])
            
            # os.system('rosrun roslink-ba gotogaolnode.py')
            # ROSLinkBridgeRIA_E100.move_to_goal(command['x'],command['y'])
            move_to_goalThread(command['x'],command['y'])
            print("goal sent to thread")
            return

        elif command['header']['message_id'] == MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_ESTOP:
            ROSLinkBridgeRIA_E100.sendingTwistFlag=True
            print("Estop")
            ROSLinkBridgeRIA_E100.TwistCommand.linear.x = 0
            ROSLinkBridgeRIA_E100.TwistCommand.linear.y = 0 
            ROSLinkBridgeRIA_E100.TwistCommand.linear.z = 0 
            ROSLinkBridgeRIA_E100.TwistCommand.angular.x = 0
            ROSLinkBridgeRIA_E100.TwistCommand.angular.y = 0 
            ROSLinkBridgeRIA_E100.TwistCommand.angular.z = 0
            # print(ROSLinkBridgeRIA_E100.sendingTwistFlag)   
            ROSLinkBridgeRIA_E100.ac.cancelAllGoals()


class move_to_goalThread():
    """docstring for command"""
    def __init__(self,xGoal,yGoal):
        self.xGoal = xGoal
        self.yGoal = yGoal
        t = threading.Thread(target=self.run)
        t.setName("move_to_goalThread")
        t.start()

    def run ( self ):
        ROSLinkBridgeRIA_E100.sendingTwistFlag=False;
        #define a client for to send goal requests to the move_base server through a SimpleActionClient
        # ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        #wait for the action server to come up
        while(not ROSLinkBridgeRIA_E100.ac.wait_for_server(rospy.Duration.from_sec(5.0))):
               rospy.loginfo("Waiting for the move_base action server to come up")

        goal = MoveBaseGoal()


        #set up the frame parameters
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # moving towards the goal*/

        goal.target_pose.pose.position =  Point(self.xGoal,self.yGoal,0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Sending goal location ...")
        ROSLinkBridgeRIA_E100.ac.send_goal(goal)

        ROSLinkBridgeRIA_E100.ac.wait_for_result(rospy.Duration(60))

        if(ROSLinkBridgeRIA_E100.ac.get_state() ==  GoalStatus.SUCCEEDED):
               rospy.loginfo("You have reached the destination")
               ROSLinkBridgeRIA_E100.sendingTwistFlag = True

        else:
               rospy.loginfo("The robot failed to reach the destination")
               ROSLinkBridgeRIA_E100.sendingTwistFlag = True
        ROSLinkBridgeRIA_E100.sendingTwistFlag = True


class TwistCommandThread():
    """docstring for command"""
    def __init__(self):
        t = threading.Thread(target=self.run)
        t.setName("TwistCommandThread")
        t.start()

    def run ( self ):
        print(ROSLinkBridgeRIA_E100.sendingTwistFlag,"threading")
        while True:#ROSLinkBridgeRIA_E100.sendingTwistFlag:
            time.sleep(0.051)
            # print "ROSLinkBridgeRIA_E100.TwistCommand"
            if(ROSLinkBridgeRIA_E100.sendingTwistFlag):
                ROSLinkBridgeRIA_E100.move_publisher.publish (ROSLinkBridgeRIA_E100.TwistCommand)


class ROSLinkMessageThread ():
    count = 0
    def __init__(self,  sock, WS,server_address,message_type ,thread_name='noname', data_rate=1.0):
        self.count = self.count +1
        self.name = thread_name
        self.socket = sock
        self.server_address = server_address
        self.data_rate = data_rate
        self.roslink_message_type = message_type
        self.WS = WS

        t = threading.Thread(target=self.run)
        t.setName(thread_name)
        t.start()
    
    def run ( self ):
        print(self.name, " started")
        while True:
            self.count=self.count+1
            time.sleep(1.0/self.data_rate)
            if(debug):
                print(self.roslink_message_type,"sent")
            # print 'thread %s %d\n'%(self.name, self.count)
            #self.send(self.socket, json.dumps(self.roslink_message.__dict__))
            try:
                if (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_HEARTBEAT):
                    if (ROSLinkStateVariables.WSconnected):
                        self.send(self.socket, json.dumps(ROSLinkBridgeRIA_E100.static_build_heartbeat_message()))
                        self.WS.send(json.dumps(ROSLinkBridgeRIA_E100.static_build_heartbeat_message()))
                        # print('heartbeat_message')
                    else:
                        self.send(self.socket, json.dumps(ROSLinkBridgeRIA_E100.static_build_heartbeat_message()))
                        self.WS = ROSLinkBridgeRIA_E100.ws;

                elif (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_ROBOT_STATUS):
                    if (ROSLinkStateVariables.WSconnected):
                        self.WS.send(json.dumps(ROSLinkBridgeRIA_E100.static_build_robot_status_message()))
                    else:
                        self.send(self.socket, json.dumps(ROSLinkBridgeRIA_E100.static_build_robot_status_message()))
                        self.WS = ROSLinkBridgeRIA_E100.ws;
                
                elif (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_GLOBAL_MOTION):
                    if (ROSLinkStateVariables.WSconnected):
                        self.WS.send(json.dumps(ROSLinkBridgeRIA_E100.static_build_global_motion_message()))
                    else:
                        self.send(self.socket, json.dumps(ROSLinkBridgeRIA_E100.static_build_global_motion_message()))
                        self.WS = ROSLinkBridgeRIA_E100.ws;
                
                elif (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_GPS_RAW_INFO):
                    if (ROSLinkStateVariables.WSconnected):
                        self.WS.send(json.dumps(ROSLinkBridgeRIA_E100.static_build_gps_raw_info_message()))
                    else:
                        self.send(self.socket, json.dumps(ROSLinkBridgeRIA_E100.static_build_gps_raw_info_message()))
                        self.WS = ROSLinkBridgeRIA_E100.ws;

                elif (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_ROSLINK_IMAGE):
                    if (ROSLinkStateVariables.WSconnected):
                        self.WS.send(json.dumps(ROSLinkBridgeRIA_E100.static_build_ROSLink_image_message()))
                        self.WS.send(json.dumps(ROSLinkBridgeRIA_E100.static_build_ROSLink_image_message()))
                        # print ('ROSLink_image sent ws')
                    else:
                        # self.send(self.socket, json.dumps(ROSLinkBridgeRIA_E100.static_build_ROSLink_image_message()))
                        # print ('ROSLink_image sent udp')
                        self.WS = ROSLinkBridgeRIA_E100.ws;
            except Exception as e:
                print(e)
                print self.name
            

                
                #ROSLinkStateVariables.ROSLinkImageSendingCounter+=1;
                

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
                ROSLinkBridgeRIA_E100.process_roslink_command_message(msg)
            except socket.timeout:
                continue   


if __name__ == '__main__':
    print '\n************** Starting ROSLink Bridge for Turtlebot Burger **************\n' 
    # initialize ROS node for this client
    myDroneBridge = ROSLinkBridgeRIA_E100() 