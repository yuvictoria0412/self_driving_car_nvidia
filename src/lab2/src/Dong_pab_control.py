#!/usr/bin/env python
from cmath import pi, sqrt
from math import atan2
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

#Due to numerical problem,the coordinate of car will float 
#initail offset in /robot_pose:
#linear x0 = -0.0006 and keep increasing
#linear y0 = 0.0006 and keep increasing
#angular z0 will switch btw 6.2807 or 0 


global GP,NP,v,GP_theta
NP = Twist()
GP = PoseStamped()#This is from rviz
GP_theta = 0
v = Twist()

global rate,pub,flag
rate = 0
pub = 0
flag = 0

def angle4bit_trans(x,y,z,w):

    angle = atan2( 2*(w*z+x*y) , 1-2*(z*z+y*y) )
    return angle

def Pub():

    global pub,rate
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz


def callback_now_pose(now_pose):

    global NP
    NP = now_pose

    if NP.angular.z > pi :
            NP.angular.z = NP.angular.z - 2*pi
    else:
        pass
    # print 'I heard /robot_pose'
    

def callback_goal(goal_pose):

    global GP,GP_theta,flag
    GP = goal_pose
    GP_theta = angle4bit_trans(GP.pose.orientation.x,GP.pose.orientation.y,GP.pose.orientation.z,GP.pose.orientation.w)
    
    flag = 1
    print 'I heard /move_base_simple/goal'

    
    # print GP_theta

def now_pose():

    # rospy.init_node('now_posise', anonymous=True)
    rospy.Subscriber('/robot_pose', Twist, callback_now_pose)
    # rospy.spin()
    # spin() simply keeps python from exiting until this node is stopped

def goal_pose():
    # rospy.init_node('goal_pose', anonymous=True)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_goal)
    # rospy.spin()
    # spin() simply keeps python from exiting until this node is stopped


# while not rospy.is_shutdown():

    # k=Twist()
def pab_control():

    global NP,GP,GP_theta,pub,v,flag
    global rate


    dx= GP.pose.position.x - NP.linear.x
    dy= GP.pose.position.y - NP.linear.y
    # dtheta = GP_theta - NP.angular.z


    
        

        ##we need to map 0~2pi into +-pi
        

    p = (dx*dx+dy*dy)**0.5
    #9**0.5=3
    #Note:sqrt() will return x+jy

    a = -NP.angular.z + atan2(dy,dx)#-2pi~+2pi

    while a > pi:
        a-=2*pi
    while a < -pi:
        a+=2*pi

    b = -NP.angular.z - a + GP_theta##-3pi~+3pi
    # print a,b
    while b > pi:
        b-=2*pi
    while b < -pi:
        b+=2*pi

    # print a,b
    if p < 0.03 :#and abs(dtheta) < 0.07  :
        
        v.linear.x = 0.0
        v.linear.y = 0.0
        # v.angular.z = 0.0
        ##Stop car
        
        
        if a < 0.3 and b < 0.3:
            v.angular.z = 0.0
            GP.pose.position.x = NP.linear.x##check
            GP.pose.position.y = NP.linear.y
            GP_theta = NP.angular.z

            flag = 0
            print 'Stop'
        else:
            pass
    else :

        Kp=3/2.0#
        Ka=8/2.0#0.8
        Kb=-1.5/2.0#-0.15

        # v=Twist()
        v.linear.x = Kp*p
        v.linear.y = 0.0
        v.angular.z = Ka*a + Kb*b

	if v.linear.x > 0.5:
		v.linear.x = 0.5
	elif v.linear.x < -0.5:
		v.linear.x = -0.5
	if v.angular.z > 1:
		v.angular.z = 1
	elif v.angular.z < -1:
		v.angular.z = -1





    pub.publish(v)
    print 'N',NP.linear.x,NP.linear.y,NP.angular.z
    print 'G',GP.pose.position.x,GP.pose.position.y,GP_theta
    print 'V',v.linear.x , v.linear.y ,  v.angular.z
        
    rate.sleep()


if __name__ == '__main__':
  try: 
        rospy.init_node('pab_controller', anonymous=True)
        Pub()
        now_pose()
        goal_pose()
        
        while not rospy.is_shutdown():
            # print 'okok88'
                pab_control()
      
  except rospy.ROSInterruptException:
      pass#The pass command actually do noting
