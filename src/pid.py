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

old_pose_x=0
old_pose_y=0

stop_signal=0

class PIDControllerNode:
    def __init__(self):
        rospy.init_node('pid')
        
        circle_traj=np.load('/home/src/pid_controller/src/sim/ref_circle.npy')
        self.circle_traj_x=circle_traj[0]
        self.circle_traj_y=circle_traj[1]

        self.cs_x_path, self.cs_y_path, self.cs_phi_path,self.cs_x_path_der,self.cs_y_path_der, self.arc_length, self.arc_vec = path_spline(self.circle_traj_x, self.circle_traj_y)

        self.kp = 1.5
        self.kd = 1.0
        self.robot_x_positions = []
        self.robot_y_positions = []

        # ROS publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
        self.angular_velocity_sub = rospy.Subscriber('/bebop/odom', Odometry, self.angular_velocity_callback)

    def angular_velocity_callback(self, data):

        x_global_init = data.pose.pose.position.x
        y_global_init = data.pose.pose.position.y

        global old_pose_x, old_pose_y

        current_vel_x = (x_global_init-old_pose_x)*100
        current_vel_y = (y_global_init-old_pose_y)*100
        


        x_waypoints, y_waypoints,x_waypoints_der,y_waypoints_der, phi_waypoints = waypoint_generator(x_global_init, y_global_init, self.circle_traj_x, self.circle_traj_y, self.arc_vec, self.cs_x_path, self.cs_y_path,self.cs_x_path_der,self.cs_y_path_der, self.cs_phi_path, self.arc_length)

        self.robot_x_positions.append(x_global_init)
        self.robot_y_positions.append(y_global_init)
        twist_msg = Twist()
        
        vx = self.kp * (x_waypoints - x_global_init) + self.kd * (x_waypoints_der-current_vel_x)
        vy = self.kp * (y_waypoints - y_global_init) + self.kd * (y_waypoints_der-current_vel_y)


        global v
        v=np.sqrt(vx**2+vy**2)
        tetha=np.arctan2(vy,vx)

        v=np.clip(v,-0.3,0.3)

        desired_angle=np.arctan2((y_waypoints - y_global_init),(x_waypoints - x_global_init))

        v_x=v*np.cos(tetha)
        v_y=v*np.sin(tetha)

        twist_msg.linear.x=v_x
        twist_msg.linear.y=v_y

        twist_msg.angular.z=tetha
        self.cmd_vel_pub.publish(twist_msg)

        
        old_pose_x=x_global_init
        old_pose_y=y_global_init




    
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        global stop_signal
        while not rospy.is_shutdown():
            if stop_signal==0:  
              rate.sleep()
            if keyboard.is_pressed('q'):  
              stop_signal=1
              break



def path_spline(x_path, y_path):
    x_diff = np.diff(x_path)
    y_diff = np.diff(y_path)
    phi = np.unwrap(np.arctan2(y_diff, x_diff))
    phi_init = phi[0]
    phi = np.hstack(( phi_init, phi  ))
    arc = np.cumsum( np.sqrt( x_diff**2+y_diff**2 )   )
    arc_length = arc[-1]
    arc_vec = np.linspace(0, arc_length, np.shape(x_path)[0])
    cs_x_path = CubicSpline(arc_vec, x_path)
    cs_y_path = CubicSpline(arc_vec, y_path)
    
    cs_x_path_der=cs_x_path.derivative()
    cs_y_path_der=cs_y_path.derivative()
    cs_phi_path = CubicSpline(arc_vec, phi)
    return cs_x_path, cs_y_path, cs_phi_path, cs_x_path_der,cs_y_path_der,arc_length, arc_vec

v = 0.3
dt = 0.2

def waypoint_generator(x_global_init, y_global_init, x_path_data, y_path_data, arc_vec, cs_x_path, cs_y_path,cs_x_path_der,cs_y_path_der, cs_phi_path, arc_length):
    idx = np.argmin( np.sqrt((x_global_init-x_path_data)**2+(y_global_init-y_path_data)**2))
    arc_curr = arc_vec[idx]
    arc_pred = arc_curr + v*dt
    x_waypoints = cs_x_path(arc_pred)
    y_waypoints =  cs_y_path(arc_pred)
    x_waypoints_der = cs_x_path_der(arc_pred)
    y_waypoints_der =  cs_y_path_der(arc_pred)
    phi_Waypoints = cs_phi_path(arc_pred)
    return x_waypoints, y_waypoints,x_waypoints_der,y_waypoints_der, phi_Waypoints


if __name__ == '__main__':
    try:
        controller = PIDControllerNode()
        controller.run()


        # Plot the trajectory when the node is shutdown
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
