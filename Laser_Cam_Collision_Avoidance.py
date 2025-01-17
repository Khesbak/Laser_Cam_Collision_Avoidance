

#!/usr/bin/env python
import rospy
import time
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
import scipy.io as sio
import os

#print cv2.__version__
range=([0])
Laser_center_point=[]
Depth_center_point=[]
Laser_Depth_Error=[]
Time_difference=[]

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
        
        print("******      Code Started     ********")
        self.Initial_time=time.time()
        self.Execution_time= 2  # Execution time in Mins.
        self.Run_time=self.Execution_time*60
        self.loop_rate = rospy.Rate(60)
        # Subscribers
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image,callback=self.convert_depth_image, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.callback)
        # Publishers
        #self.velocity_publisher = rospy.Publisher('/base/cmd_vel', Twist, queue_size=10)
        self.Cam_Laser_Error_publisher = rospy.Publisher('/Cam_Lasr_Error', Float32, queue_size=10)
        #self.Laser()
    def convert_depth_image(self,ros_image):
        self.bridge = CvBridge()
        self.depth_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
        self.depth_array = np.array(self.depth_image, dtype=np.float32)
        self.center_idx = np.array(self.depth_array.shape) / 2
        # The center point is (540,960)
        #self.Cam_Dist=(self.depth_array[self.center_idx[0], self.center_idx[1]])/1000
        
        self.size=2
        #self.Cam_Dist_acc=(self.depth_array[(540-self.size):(540+self.size), (960-self.size):(960+self.size)])/1000
        self.Cam_Dist=((self.depth_array[540-self.size:540+self.size+1, 960-self.size:960+self.size+1])/1000).sum()/( (2*self.size+1) * (2*self.size+1) )
        # print(self.Cam_Dist)
        # print("")
        self.Depth_Time=time.time()
        self.Laser_Dist=range[0]
        self.Error = Float32()
        self.Error=abs(self.Cam_Dist-self.Laser_Dist) *100
        self.Cam_Laser_Error_publisher.publish(self.Error)
        Time_difference.append(abs(self.Laser_Time-self.Depth_Time))
        Laser_center_point.append(self.Laser_Dist)
        Depth_center_point.append(self.Cam_Dist)
        Laser_Depth_Error.append(self.Error)
        self.Time_Out=time.time()-self.Initial_time
        
        #print(Depth_center_point,"   ",Laser_center_point)
        #print("")
        
        if (self.Time_Out > self.Run_time):
            print("code was executed for ",self.Time_Out/60,"  Minutes.......check your .mat file please")
            sio.savemat('/home/mohammed/Laser_Cam_Collision_Avoidance/motion_4_point_test1.mat',{'Laser_center_point':Laser_center_point,'Depth_center_point':Depth_center_point,'Laser_Depth_Error':Laser_Depth_Error,'Time_difference':Time_difference})
            os._exit(0)
            


        #print ('D435-Camera - Center Depth (m): ', self.Cam_Dist,'Laser- Center Beam (m) (after position translation): ',self.Laser_Dist,"  Error (cm) = ",self.Error)
        # print("********************************************")
        # print(Laser_center_point)
        # print("")
        # print(Depth_center_point)
        # print("")
        # print(Laser_Depth_Error)
        # print("********************************************")
        #print(Time_difference)


    def callback(self,msg):
        range[0]=msg.ranges[540]
        self.Laser_Time=time.time()
        

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
    
    
    
