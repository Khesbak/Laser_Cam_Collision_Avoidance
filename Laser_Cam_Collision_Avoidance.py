






#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from matplotlib import pyplot as plt
from time import sleep
import numpy as np
import sys
import cv2
import multiprocessing
import cv_bridge
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

#print cv2.__version__
range=([0])
def callback(msg):
    print("Hiiiii")
    global range
    range[0]=msg.ranges[540]

def Laser():
    # Starts a new node
    rospy.init_node('robot_cleaner', anonymous=True)
    sub1=rospy.Subscriber('/scan', LaserScan, callback)
    velocity_publisher = rospy.Publisher('/base/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    #Receiveing the user's input
    print("Let's move your robot")
    # speed = input("Input your speed:")
    # distance = input("Type your distance:")
    #isForward = input("Foward?: ")#True or False
    speed = 0.1
    distance =2
    isForward ="True"
    #Checking if the movement is forward or backwards
    # if(isForward):
    #     vel_msg.linear.x = abs(speed)
    # else:
    #     vel_msg.linear.x = -abs(speed)
    #Since we are moving just in x-axis
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    
    while not rospy.is_shutdown():
        
        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        #Loop to move the turtle in an specified distance
        
        while(current_distance < distance):
       
            print(range[0])
            #Publish the velocity
            if (range[0]<1 and range[0]>0):
                a=0
                vel_msg.linear.x = 0
            else:
                    if(isForward):
                        vel_msg.linear.x = abs(speed)
                    else:
                        vel_msg.linear.x = -abs(speed)
            velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
            current_distance= speed*(t1-t0)
        #After the loop, stops the robot
        vel_msg.linear.x = 0
        #Force the robot to stop
        velocity_publisher.publish(vel_msg)


#!/usr/bin/env python
class Nodo(object):
    #  Initialization subscription and publishing to Topics
    def __init__(self):
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(60)
        # Subscribers
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image,callback=self.convert_depth_image, queue_size=1)
        #rospy.Subscriber("/camera/color/image_raw", Image,callback=self.convert_depth_image, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.callback)
        # Publishers
        self.velocity_publisher = rospy.Publisher('/base/cmd_vel', Twist, queue_size=10)
        self.Cam_Laser_Error_publisher = rospy.Publisher('/Cam_Lasr_Error', Float32, queue_size=10)
        #self.Laser()
    def convert_depth_image(self,ros_image):
        self.bridge = CvBridge()
        # Use cv_bridge() to convert the ROS image to OpenCV format
        #try:
        #Convert the depth image using the default passthrough encoding
        self.depth_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
        self.depth_array = np.array(self.depth_image, dtype=np.float32)
        self.center_idx = np.array(self.depth_array.shape) / 2
        self.Cam_Dist=(self.depth_array[self.center_idx[0], self.center_idx[1]])/1000
        self.Laser_Dist=range[0]
        self.Error = Float32()
        self.Error.data=abs(self.Cam_Dist-self.Laser_Dist) 
        print(self.Error)
        self.Cam_Laser_Error_publisher.publish(self.Error)
        #print ('D435-Camera - Center Depth (m): ', self.Cam_Dist,'Laser- Center Beam (m) (after position translation): ',self.Laser_Dist,"  Error  = ",abs(self.Cam_Dist-self.Laser_Dist))
        #print(range[0])
        #print(self.depth_array)
        #except CvBridgeError, e:
            #print e
        #Convert the depth image to a Numpy array
    def callback(self,msg):
        #print("Hiiiii")
        range[0]=msg.ranges[540]

    def Laser(self):
        # Starts a new node
        self.vel_msg = Twist()

        # speed = input("Input your speed:")
        # distance = input("Type your distance:")
        #isForward = input("Foward?: ")#True or False
        self.speed = 0.1
        self.distance =2
        self.isForward ="True"
        #Checking if the movement is forward or backwards
        # if(isForward):
        #     vel_msg.linear.x = abs(speed)
        # else:
        #     vel_msg.linear.x = -abs(speed)
        #Since we are moving just in x-axis
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0
        
        #while not rospy.is_shutdown():
            
        #Setting the current time for distance calculus
        self.t0 = rospy.Time.now().to_sec()
        self.current_distance = 0

        #Loop to move the turtle in an specified distance
        
        while(self.current_distance < self.distance):
    
            #print(range[0])
            #Publish the velocity
            if (range[0]<1 and range[0]>0):
                
                self.vel_msg.linear.x = 0
            else:
                    if(self.isForward):
                        self.vel_msg.linear.x = abs(self.speed)
                    else:
                        self.vel_msg.linear.x = -abs(self.speed)
            #self.velocity_publisher.publish(self.vel_msg)
            #Takes actual time to velocity calculus
            self.t1=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
            self.current_distance= self.speed*(self.t1-self.t0)
        #After the loop, stops the robot
        self.vel_msg.linear.x = 0
        #Force the robot to stop
        #self.velocity_publisher.publish(self.vel_msg)
 

if __name__ == '__main__':
    rospy.init_node("nodo1", anonymous=True)
    my_node = Nodo()
    rospy.spin()
    
    
    