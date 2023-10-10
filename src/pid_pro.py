import rospy
import math
import numpy as np
import keyboard
import tf
from geometry_msgs.msg import Twist, PoseStamped,Quaternion
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from scipy.interpolate import CubicSpline

import matplotlib.pyplot as plt




from pid_controller.path_modules import path_spline, waypoint_generator

old_pose_x=0
old_pose_y=0
stop_signal=0



class PIDControllerNode:
    def __init__(self):
        rospy.init_node('pid')
        
        circle_traj=np.load('/home/src/pid_controller/src/irl/sine.npy')
        self.circle_traj_x=circle_traj[0]+3.6
        self.circle_traj_y=circle_traj[1]-2.0

        

        self.cs_x_path, self.cs_y_path, self.cs_phi_path,self.cs_x_path_der,self.cs_y_path_der, self.arc_length, self.arc_vec = path_spline(self.circle_traj_x, self.circle_traj_y)
        stop_signal=0
        
        self.kp = 5.0
        self.kd = 2*np.sqrt(self.kp)
        self.robot_x_positions = []
        self.robot_y_positions = []
        self.old_time=rospy.Time.now()
        self.v = 0.5
        self.dt = 0.1
        # ROS publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/bebop/velocity', Twist, queue_size=1)
        self.angular_velocity_sub = rospy.Subscriber('/vrpn_client_node/bebop/pose', PoseStamped, self.angular_velocity_callback)

    def angular_velocity_callback(self, data):

        x_global_init = data.pose.position.x
        y_global_init = data.pose.position.y
        new_time=rospy.Time.now()
        global old_pose_x, old_pose_y
        current_vel_x = (x_global_init-old_pose_x)*240 
        current_vel_y = (y_global_init-old_pose_y)*240 

        


        x_waypoints, y_waypoints,x_waypoints_der,y_waypoints_der, phi_waypoints = waypoint_generator(x_global_init, y_global_init, self.circle_traj_x, self.circle_traj_y, self.arc_vec, self.cs_x_path, self.cs_y_path,self.cs_x_path_der,self.cs_y_path_der, self.cs_phi_path, self.arc_length,self.dt,self.v)

        self.robot_x_positions.append(x_global_init)
        self.robot_y_positions.append(y_global_init)
        twist_msg = Twist()
        
        vx = self.kp * (x_waypoints - x_global_init) + self.kd * (x_waypoints_der-current_vel_x)
        vy = self.kp * (y_waypoints - y_global_init) + self.kd * (y_waypoints_der-current_vel_y)


        another_v=np.sqrt(vx**2+vy**2)
        theta = np.arctan2(vy, vx)

        another_v=np.clip(another_v,-1*self.v,self.v)
        
        v_x=another_v*np.cos(theta)
        v_y=another_v*np.sin(theta)

        twist_msg.linear.x=v_x
        twist_msg.linear.y=v_y

        # twist_msg.angular.z=theta_error
        self.cmd_vel_pub.publish(twist_msg)

        
        old_pose_x=x_global_init
        old_pose_y=y_global_init
        self.old_time=rospy.Time.now()




    
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        global stop_signal
        while not rospy.is_shutdown():
            if stop_signal==0:
              rate.sleep()
            if keyboard.is_pressed('q'): 
              stop_signal=1
              break




try:
    controller = PIDControllerNode()
    controller.run()



    plt.figure(figsize=(8, 6))
    plt.plot(controller.robot_x_positions, controller.robot_y_positions, label='Robot Trajectory', linewidth=4)
    plt.scatter(controller.circle_traj_x, controller.circle_traj_y, c='red', label='Reference Circle', s=30)
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Robot Trajectory')
    plt.legend()
    plt.grid(True)
    plt.show()
except rospy.ROSInterruptException:
    pass
