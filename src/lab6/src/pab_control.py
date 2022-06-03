#!/usr/bin/env python
from cmath import pi
from math import atan2
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
# from geometry_msgs.msg import PointStamped



global GP,NP,v,arriveflag
NP = Twist()
GP = Twist()#This is from A*
v = Twist()
arriveflag=Bool()


global rate,pub,flag
rate = 0
pub = 0
flag = 0


def Pub():

    global pub,rate
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz

def Pub_check_arrive():

    global pub_arrive,ratee
    pub_arrive = rospy.Publisher('/check_arrive_topic', Bool, queue_size=10)
    ratee = rospy.Rate(10) # 10hz


def callback_now_pose(now_pose):

    global NP
    NP = now_pose

    if NP.angular.z > pi :
            NP.angular.z = NP.angular.z - 2*pi
    else:
        pass
    # print 'I heard /robot_pose'
    

def callback_goal(goal_pose):

    global GP,flag
    GP = goal_pose
    while GP.angular.z > pi:
        GP.angular.z-=2*pi
    while GP.angular.z < -pi:
        GP.angular.z+=2*pi
    
    print 'I heard next point'

def now_pose():

    rospy.Subscriber('/robot_pose', Twist, callback_now_pose)

def goal_pose():
    # rospy.init_node('goal_pose', anonymous=True)
    rospy.Subscriber('/Aatar_nav_topic', Twist, callback_goal)
    # rospy.spin()
    # spin() simply keeps python from exiting until this node is stopped


def pab_control():

    global NP,GP,pub,v,arriveflag
    global rate


    dx= GP.linear.x - NP.linear.x
    dy= GP.linear.y - NP.linear.y
    # dtheta = GP_theta - NP.angular.z


    if abs(dx) < 0.1 and abs(dy) < 0.1 :#and abs(dtheta) < 0.07  :
        
        v.linear.x = 0.0
        v.linear.y = 0.0
        v.angular.z = 0.0
        ##Stop car
        # GP.linear.x = NP.linear.x##check
        # GP.linear.y = NP.linear.y
        # GP_theta = NP.angular.z
        
        arriveflag = True
        pub_arrive.publish(arriveflag)
        print 'Stop'
        

    else :
        arriveflag=False
        pub_arrive.publish(arriveflag)
        ##we need to map 0~2pi into +-pi
        

        p = (dx*dx+dy*dy)**0.5
        #9**0.5=3
        #Note:sqrt() will return x+jy

        a = -NP.angular.z + atan2(dy,dx)#-2pi~+2pi
        

        while a > pi:
            a-=2*pi
        while a < -pi:
            a+=2*pi

        b = -NP.angular.z - a + GP.angular.z##-3pi~+3pi
        # print a,b
        while b > pi:
            b-=2*pi
        while b < -pi:
            b+=2*pi

        # print a,b

        Kp=3/3.0
        Ka=8/3.0
        Kb=-1.5/3.0

        v.linear.y = 0.0
        v.linear.x = Kp*p
        v.angular.z = Ka*a + Kb*b
        

        if v.linear.x > 0.5:
            v.linear.x = 0.5
        elif v.linear.x  < -0.5:
            v.linear.x = -0.5
        
        if v.angular.z > 1:
            v.angular.z = 1
        elif v.angular.z  < -1:
            v.angular.z = -1
    print 'v:',v.linear.x,'w:',v.angular.z
    pub.publish(v)
    print 'N',NP.linear.x,NP.linear.y,NP.angular.z
    print 'G',GP.linear.x,GP.linear.y,GP.angular.z
    # print 'V',v.linear.x , v.linear.y ,  v.angular.z
        
    rate.sleep()


if __name__ == '__main__':
  try: 
        rospy.init_node('pab_controller', anonymous=True)
        Pub()
        Pub_check_arrive()
        now_pose()
        goal_pose()
        
        while not rospy.is_shutdown():
            # print 'okok88'
                pab_control()
                
      
  except rospy.ROSInterruptException:
      pass#The pass command actually do noting
    