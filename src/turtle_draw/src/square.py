#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

length = 3 ##side length

def talker():
  pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
  rospy.init_node('square_pub', anonymous=True)
  rate = rospy.Rate(1) # 1hz

  
  while not rospy.is_shutdown():

    k=Twist()
    k.linear.x=length
    k.linear.y=0
    k.linear.z=0
    k.angular.x=0
    k.angular.y=0
    k.angular.z=0
    pub.publish(k)
    rate.sleep()

    k.linear.x=0
    k.linear.y=length
    pub.publish(k)
    rate.sleep()

    k.linear.x=-length
    k.linear.y=0
    pub.publish(k)
    rate.sleep()

    k.linear.x=0
    k.linear.y=-length
    pub.publish(k)
    rate.sleep()
    

    


   
if __name__ == '__main__':
  try:
      talker()
  except rospy.ROSInterruptException:
      pass
