#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib;
import rospy
import actionlib  
import roslaunch
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample 
from math import pow, sqrt
import sys, select, termios, tty
from robot_vision.msg import ballposition 
from robot_vision.msg import color_choice

msg="""
请选择你需要追什么颜色的球
如果你选择红色 请输入：red
如果你选择绿色 请输入：green
如果你选择蓝色 请输入：blue
如果你选择金色 请输入：gold
如果你选择黑色 请输入：black
"""

ball_color = {
        'red':0,
        'green':1,
        'blue':2,
        'gold':3,
        'white':4,
        'black':5
           }

x_max = 640
y_max = 480
speed = 3
Pturn = 0.005
Dturn = 0.005
det_screen = 0
det_x =0
kict_flag = 0

det = {'det_lastx':0 ,'det_nowx': 0}
#注意策略的框架

class NavTest():  
    #uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    #uuid1 = roslaunch.rlutil.get_or_generate_uuid(None, False)
    #uuid2 = roslaunch.rlutil.get_or_generate_uuid(None, False)
    #roslaunch.configure_logging(uuid)
    #roslaunch.configure_logging(uuid1)
    #roslaunch.configure_logging(uuid2)
    #launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/wengyuzhe/catkin_v520/src/mbot_navigation/launch/gmapping.launch"])
    #launch1 = roslaunch.parent.ROSLaunchParent(uuid1, ["/home/wengyuzhe/catkin_v520/src/mbot_navigation/launch/move_base.launch"])
    #launch2 = roslaunch.parent.ROSLaunchParent(uuid2, ["/home/wengyuzhe/catkin_v520/src/mbot_navigation/launch/amcl.launch"])
    # launch = roslaunch.parent.ROSLaunchParent(uuid, ["/opt/ros/kinetic/share/rosbridge_server/launch/rosbridge_websocket.launch"])
    #launch.start()
    #launch1.start()
    #launch2.start()
    #rospy.loginfo("started")

    #rospy.sleep(3)
    # 3 seconds later
    #launch.shutdown()
    def __init__(self):  
        rospy.on_shutdown(self.shutdown)  

        # 在每个目标位置暂停的时间 (单位：s)
        self.rest_time = rospy.get_param("~rest_time", 2)  

        # 是否仿真？  
        self.fake_test = rospy.get_param("~fake_test", True)  

        # 到达目标的状态  
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',   
                       'SUCCEEDED', 'ABORTED', 'REJECTED',  
                       'PREEMPTING', 'RECALLING', 'RECALLED',  
                       'LOST']  
 
        # 设置目标点的位置  
        # 在rviz中点击 2D Nav Goal 按键，然后单击地图中一点  
        # 在终端中就会看到该点的坐标信息  
        locations = dict()  

        locations['1'] = Pose(Point(15, 45, 0.000),  Quaternion(0.000, 0.000, -0.447, 0.894))  
        locations['2'] = Pose(Point(15, 45, 0.000),  Quaternion(0.000, 0.000, -0.847, 0.532))  
        locations['3'] = Pose(Point(15, 45, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))  
        locations['4'] = Pose(Point(15, 45, 0.000), Quaternion(0.000, 0.000, 0.645, 0.764))  
        locations['5'] = Pose(Point(15, 45, 0.000), Quaternion(0.000, 0.000, 0.340, 0.940))  
        locations['6'] = Pose(Point(15, 45, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000))  

        # 发布控制机器人的消息  
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)  

        # 订阅move_base服务器的消息  
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  

        #rospy.loginfo("Waiting for move_base action server...")  

        # 60s等待时间限制  
        self.move_base.wait_for_server(rospy.Duration(60))  
        #rospy.loginfo("Connected to move base server")  
  
        # 保存机器人的在rviz中的初始位置  
        initial_pose = PoseWithCovarianceStamped()  

        # 保存成功率、运行时间、和距离的变量  
        n_locations = len(locations)  
        n_goals = 0  
        n_successes = 0  
        i = n_locations  
        distance_traveled = 0  
        start_time = rospy.Time.now()  
        running_time = 0  
        location = ""  
        last_location = ""    
 
        # 确保有初始位置  
        while initial_pose.header.stamp == "":  
            rospy.sleep(1)  

        #rospy.loginfo("Starting navigation test")  

        # 开始主循环，随机导航  
        while not rospy.is_shutdown():  
            # 如果已经走完了所有点，再重新开始排序  
            if i == n_locations:  
                i = 0  
                sequence = sample(locations, n_locations)  
 
                # 如果最后一个点和第一个点相同，则跳过  
                if sequence[0] == last_location:  
                    i = 1  

            # 在当前的排序中获取下一个目标点  
            location = sequence[i]  

            # 跟踪行驶距离  
            # 使用更新的初始位置  
            if initial_pose.header.stamp == "":  
                distance = sqrt(pow(locations[location].position.x -   
                                    locations[last_location].position.x, 2) +  
                                pow(locations[location].position.y -   
                                    locations[last_location].position.y, 2))  
            else:  
                #rospy.loginfo("Updating current pose.")  
                distance = sqrt(pow(locations[location].position.x -   
                                    initial_pose.pose.pose.position.x, 2) +  
                                pow(locations[location].position.y -   
                                    initial_pose.pose.pose.position.y, 2))  
                initial_pose.header.stamp = ""  

            # 存储上一次的位置，计算距离  
            last_location = location  

            # 计数器加1  
            i += 1  
            n_goals += 1  

            # 设定下一个目标点  
            self.goal = MoveBaseGoal()  
            self.goal.target_pose.pose = locations[location]  
            self.goal.target_pose.header.frame_id = 'map'  
            self.goal.target_pose.header.stamp = rospy.Time.now()  

            # 让用户知道下一个位置  
            #rospy.loginfo("Going to: " + str(location))  

            # 向下一个位置进发  
            self.move_base.send_goal(self.goal)  

            # 十秒钟时间限制  
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(10))   

            rospy.sleep(self.rest_time)  

    def update_initial_pose(self, initial_pose):  
        self.initial_pose = initial_pose  

    def shutdown(self):  
        rospy.loginfo("Stopping the robot...")  
        self.move_base.cancel_goal()  
        rospy.sleep(2)  
        self.cmd_vel_pub.publish(Twist())  
        rospy.sleep(1)  


def chase(pos):
    #global Pturn 
    #global Dturn
    #if pos.x+(pos.w/2)<40 or pos.x+(pos.w/2)>600 :
        #Pturn=0.008
        #Dturn=0.006
    det_x = x_max/2 - (pos.x +(pos.w/2))
    det['det_lastx'] = det['det_nowx']
    det['det_nowx'] = det_x
    turn = Pturn * det_x+Dturn*(det['det_nowx']-det['det_lastx'])
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
    #print(pos.x)
    #print(pos.w)	
    global kict_flag
    #if pos.x<=1 or (pos.x+pos.w)>=1277:
    #    kict_now = 1
    #else:
    #    kict_now = 0
    pubcolor = rospy.Publisher('color',color_choice, queue_size=1)
    Color_choice = color_choice()
    Color_choice.c = chase_color
    pubcolor.publish(Color_choice)

    if pos.kict==1 :
        kict_flag = 1

    twist = Twist()
    if pos.flag == 1 and kict_flag == 0:
        twist.linear.x = speed
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = turn
        pub.publish(twist)
    elif pos.flag == 0 and kict_flag ==0:
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 1
        pub.publish(twist)
    else :
        print("10")
        NavTest()
        print("20")
	rospy.spin()
    

def listener():
    #rospy.init_node('move_mrobot')
    rospy.Subscriber("ball_position", ballposition, chase)
    rospy.spin()

if __name__=="__main__":
    try:
        rospy.init_node('move_mrobot')
        print msg
        #key = getKey()
        global chase_color
        key = raw_input("请输入球的颜色:")
        #print("input >> " + key)
        while key not in ball_color.keys():
            print("illegal input")
            key = raw_input("请重新输入球的颜色:")
        if key in ball_color.keys():
            chase_color = ball_color[key]
            if chase_color == 3:
		Pturn = 0.006
            print("chasing the "+key+" ball")
            pubcolor = rospy.Publisher('color',color_choice, queue_size=1)
            Color_choice = color_choice()
            Color_choice.c = chase_color
            pubcolor.publish(Color_choice)
        listener()
    except rospy.ROSInterruptException:
        pass
        
