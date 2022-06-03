#!/usr/bin/env python
from math import atan2
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray

# from cmath import pi
# from math import atan2
global yx_map
yx_map=[]

#pub coor to pab_control
global sub_GP,count,arriveflag,GP,NP,GP_theta,get_arrow_flag,Velocity
# arriveflag = Bool()
GP = PoseStamped()#This is from rviz
GP_theta = 0
NP = Twist()
NP.linear.x = 0
NP.linear.y = 0
NP.angular.z = 0
sub_GP = Twist()
count=0
get_arrow_flag = False
Velocity = 0.3

def angle4bit_trans(x,y,z,w):

    angle = atan2( 2*(w*z+x*y) , 1-2*(z*z+y*y) )
    return angle

def goal_pose():
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_goal)
    
def callback_goal(goal_pose):

    global GP,GP_theta,car_path,get_arrow_flag,NP,count,pub
    GP = goal_pose
    GP_theta = angle4bit_trans(GP.pose.orientation.x,GP.pose.orientation.y,GP.pose.orientation.z,GP.pose.orientation.w)
    
    print 'I heard /move_base_simple/goal'

    count = 0
    car_path=Float64MultiArray()
    car_path.layout.data_offset=0
    car_path.layout.dim



    car_path.data=Astar_path([NP.linear.x,NP.linear.y],[GP.pose.position.x,GP.pose.position.y],NP.angular.z)
    
    print "Astar OK!"

    pub.publish(car_path)

    get_arrow_flag = True

    NP.linear.x = GP.pose.position.x
    NP.linear.y = GP.pose.position.y
    NP.angular.z = GP_theta

def Pub():
    global pub,rate
    pub = rospy.Publisher('/way_point', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(10) #       
    
# def if_arrive():
#     rospy.Subscriber('/check_arrive_topic', Bool, callback_if_arrive)
# def callback_if_arrive(arriveflag):
#     global arrived_flag,get_arrow_flag,pub,count,car_path,GP,NP
#     arrived_flag=arriveflag.data
    
#     if get_arrow_flag == True:
#         pass
#     else:
#         get_arrow_flag = False


class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0
    def __eq__(self, other):
        return self.position == other.position#(a==b) will return True if a.position==b.position


def read_map():
    rospy.Subscriber('/map',OccupancyGrid,read_map_callback)

def read_map_callback(map_data):
    global yx_map
    expansion_pixels=5#nums of pixels of expansion
    width   = 640#pixels
    height  = 608#pixels
    j=0
    i=0
    wall_list=[]
    for i in range(height):
        row=map_data.data[j:(j+width)]
        #row in tuple
        #[num]for : should plus one(including \n,i guess)
        row=list(row)
        yx_map.append(row)
        j+=width

    for y in range(height):
        for x in range(width):
            if yx_map[y][x]==100:
                wall_list.append((x,y))##[(2,3),(2,4),....]
            else:
                pass
    # print yx_map
    for wall in wall_list:
        '''map expasion'''
        for  ex_pix in range(0,expansion_pixels):

            try:##consider out of boundary                
                yx_map[wall[1]+ex_pix][wall[0]+ex_pix]=100
                yx_map[wall[1]+ex_pix][wall[0]+0]=100
                yx_map[wall[1]+ex_pix][wall[0]-ex_pix]=100
                yx_map[wall[1]+0][wall[0]-ex_pix]=100
                yx_map[wall[1]-ex_pix][wall[0]-ex_pix]=100
                yx_map[wall[1]-ex_pix][wall[0]+0]=100
                yx_map[wall[1]-ex_pix][wall[0]+ex_pix]=100
                yx_map[wall[1]+0][wall[0]+ex_pix]=100
            except:
                continue
    print "map expansion OK"
            

def take_element(ele):
    return ele.f

def near8_square(node):
    
    near8_node_list=[Node(None,[node.position[0]+1,node.position[1]+1]),\
                    Node(None,[node.position[0]+1,node.position[1]+0]),\
                    Node(None,[node.position[0]+1,node.position[1]-1]),\
                    Node(None,[node.position[0]+0,node.position[1]+1]),\
                    Node(None,[node.position[0]+0,node.position[1]-1]),\
                    Node(None,[node.position[0]-1,node.position[1]+1]),\
                    Node(None,[node.position[0]-1,node.position[1]+0]),\
                    Node(None,[node.position[0]-1,node.position[1]-1])]
    return near8_node_list

def integerize_coordinate(coor):#turn coordinate(3.0,4.0)m into (30,40) to avoid float == issue
    coor[0]=int(coor[0]*20)#reslution of map is 0.05m*0.05m
    coor[1]=int(coor[1]*20)

def turn_back(xy_list):
    '''turn integer coordinate back to float'''
    for coor in xy_list:
        coor[0]=coor[0]/20.0#reslution of map is 0.05m*0.05m
        coor[1]=coor[1]/20.0

def Astar(start,goal):#input start = [x,y] in meter

    global yx_map

    integerize_coordinate(start)#float using == will have some issues
    integerize_coordinate(goal)

    start_node  =  Node(None,start)
    start_node.g=start_node.h=start_node.f=0
    goal_node   =  Node(None,goal)
    goal_node.g = goal_node.h = goal_node.f = 0
    close_list  =  []
    open_list   =  []
    parent_dict = {}
    parent_dict[tuple(start)]=start    

    open_list.append(start_node)
    # near8_square = [[0,1],[1,1],[1,0],[1,-1],[0,-1],[-1,-1],[-1,0],[-1,1]]

    t=0
    while len(open_list) > 0:

        t+=1#how many times it search
        
        open_list.sort(key=take_element,reverse=True)#arrange f from max to min

        x_node = open_list.pop()#[x,y,gc,hc,fc,parent_x,parent_y]
        
        if x_node == goal_node:
           
            path = []
            trace_coor = x_node.position
            jj=0
            while trace_coor != start_node.position:
                jj+=1
                # print 'jj',jj
                # print parent_dict[tuple(trace_coor)]
                # print start_node.position
                path.append(trace_coor)
                trace_coor = parent_dict[tuple(trace_coor)]

            
            print '# of search:',t
            path.reverse()
            return path
        
        close_list.append(x_node.position)
        
        for y_node in near8_square(x_node):
            
            #ignore considered node
            if y_node.position in close_list:
                continue

            #ignore wall on map
            if yx_map[y_node.position[1]+int(21.2*20)][y_node.position[0]+10*20]==100:
                continue
            #origin shift (x,y)=(   10*20  ,   int(21.2*20) )
            # if yx_map[y_node.position[1]+int(21.2*20)][y_node.position[0]+10*20]==-1:
            #     continue

            if (y_node.position[0]!=x_node.position[0])   and   (y_node.position[1]!=x_node.position[1]):##check float issue
                dis_xy = 14
            else :
                dis_xy = 10    

            gt = y_node.g + dis_xy
            k=False

            # if not y_node in open_list:
            # if not y_node.position in open_coor_list:
            if not y_node in open_list:#need to check
                open_list.append(y_node)
                k=True
            elif gt < y_node.g:
                k=True

            if k == True:
                parent_dict[tuple(y_node.position)] = x_node.position#[x,y,gc,hc,fc,parent_x,parent_y]
                
                y_node.g= gt

                y_node.h=int(10*abs(goal[0]-y_node.position[0]))  + int(10*abs(goal[1]-y_node.position[1]))
                y_node.f=y_node.g+y_node.h
            # print 'check_y',y_node
    
    return 'failure' 

def xy_to_XYThetaV(xy_list,initial_theta):
    global Velocity
    # XYThetaV_list=[]
    XYThetaV_list=xy_list[0]+[initial_theta]+[0.0]
    #start form [x0,y0,theta0,v0,.......

    for i in range(1,len(xy_list)-1):
        
        XYThetaV_list+=xy_list[i]
        XYThetaV_list+=[atan2(xy_list[i][1]-xy_list[i-1][1],xy_list[i][0]-xy_list[i-1][0])]
        XYThetaV_list+=[Velocity]

    XYThetaV_list[len(XYThetaV_list)-1] = 0.0
    #end [......xf,yf,thetaf,0.0]

    return XYThetaV_list

def Astar_path(start_xy,goal_xy,initial_angle):
    '''input [x0,y0],[x_goal,y_goal],initial_theta 
        x,y unit in m;theta unit in rad 
    '''
    path_found = Astar(start_xy,goal_xy)
    path_skip=[]
    skip_pixels = 2
    for i in range(0,len(path_found)):
        if i %(skip_pixels+1) ==0:#0th 3th 6th 9th ....points
            path_skip.append(path_found[i])
        elif i == len(path_found)-1:
            path_skip.append(path_found[i])
        else:
            pass

    turn_back(path_skip)

    return  xy_to_XYThetaV(path_skip,initial_angle)


if __name__ == '__main__':
    
    rospy.init_node('Astar_nav_node', anonymous=True)
    read_map()

    goal_pose()
    Pub()
    # if_arrive()
    rospy.spin()



    


    
