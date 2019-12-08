#!/usr/bin/env python

from SimpleWebSocketServer import SimpleWebSocketServer, WebSocket # pip install git+https://github.com/dpallot/simple-websocket-server.git
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import  CompressedImage, LaserScan  #, Image, CameraInfo,
import cv2
import numpy as np
import base64
from geometry_msgs.msg import Twist
import threading
import time


# Load an color image in grayscale

clients = []

lidar= []
d=False
def compressedImageCallback(data):
    # print("compressedImageCallback")

    global clients,d
    if d:
        d=False
        return;
    else:
        d=True
    try:
      #cv_image = ROSLinkBridgeARDrone.bridge.imgmsg_to_cv2(data, "bgr8")
      np_arr = np.fromstring(data.data, np.uint8)
      cv_image = cv2.imdecode(np_arr, cv2.CV_16U)
    except CvBridgeError as e:
      print(e)
    data =  u''+base64.encodestring(np_arr)
    for client in clients:
        # print("message sent")
        client.sendMessage(data)

def LidarCallback(msg):
    global lidar
    lidar = msg.ranges[330:360]+msg.ranges[0:40]
    lidar = lidar[::-1]


rospy.init_node('roslink_tensorflow_bridge_node', anonymous=True)
rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, compressedImageCallback)
rospy.Subscriber("/scan", LaserScan, LidarCallback)
pub = rospy.Publisher('/tensorflow_bridge',String, queue_size=10)
tpub = rospy.Publisher('/teleop/cmd_vel',Twist, queue_size=10) 
twist= Twist()
print("roslink tensorflow bridge node started!")


class TwistCommandThread():
    """docstring for command"""
    def __init__(self):
        t = threading.Thread(target=self.run)
        t.setName("TwistCommandThread")
        t.start()

    def run ( self ):
        global tpub,twist,rospy
        while not rospy.is_shutdown():
            time.sleep(0.0551)
            # print ROSLinkBridgeRIA_E100.TwistCommand
            tpub.publish (twist)



TwistCommandThread = TwistCommandThread()
websocket_prot = 9091

clients = []


#PID controller 
linear_p=0.25
linear_i_sum=0
linear_i_factor=0.02

angular_p= - 0.0145
angular_i_sum = 0
angular_i_factor= - 0.0005


class SimpleWebSocketHub(WebSocket):

    def handleMessage(self):
        global pub, lidar,twist, angular_p,angular_i_factor,angular_i_sum,linear_p,linear_i_sum,linear_i_factor
        print("self.data")
        legs = self.data.split('-')
        chosed_point=0;
        chosed_point_distacne= 2.50
        for leg in legs:
            if(len(leg)>10):
                x = leg.split(",")
                xmin= int(float(x[1]) * 70)
                xmax= int(float(x[3]) * 70)
                print(xmin,xmax,"xmin,xmax", min(lidar[xmin:xmax]))
                atemp=chosed_point
                dtemp=chosed_point_distacne
                i=xmin
                while i<xmax:
                    if lidar[i]<chosed_point_distacne:
                        chosed_point_distacne = lidar[i]
                        chosed_point = i -30
                        # chosed_point = int((xmin+xmax)/2)
                    i+=1

        print(chosed_point, chosed_point_distacne)
        if (chosed_point_distacne<2.5):
            if (chosed_point_distacne>0.65):
                linear_i_sum+=chosed_point_distacne
                twist.linear.x= chosed_point_distacne * linear_p + linear_i_factor * linear_i_sum
                angular_i_sum+=chosed_point
                twist.angular.z= chosed_point * angular_p * 0.4  + angular_i_factor * angular_i_sum
            else:
                print("just angular")
                twist.linear.x=0
                if chosed_point_distacne>0.35:
                    angular_i_sum+=chosed_point
                    twist.angular.z= chosed_point * angular_p + angular_i_factor * angular_i_sum
                else:
                    twist.angular.z=0
        else:
            print("out of range")
            twist.angular.z=0
            twist.linear.x=0
            linear_i_sum=0
            angular_i_sum=0
            
                


        
        pub.publish(self.data)
        

        
    def handleConnected(self):
        global clients
        print(self.address, 'connected')
        clients.append(self)
        print("connected clients", len(clients))
         
        

    def handleError(self, error):
        print (error)
    
    def handleClose(self):
        clients.remove(self)
        print(self.address, 'closed')
        print("connected clients", len(clients))

server = SimpleWebSocketServer('', websocket_prot, SimpleWebSocketHub)
server.serveforever()
