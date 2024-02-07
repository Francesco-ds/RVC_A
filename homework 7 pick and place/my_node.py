from typing import List
import rclpy
import pandas as pd
from rclpy.node import Node
import csv
from rclpy.parameter import Parameter
from tf_transformations import *
from math import pi as PI
from std_msgs.msg import Bool
from geometry_msgs.msg import *
from csv_utils import *
import os
import time

# Get the current working directory
current_directory = os.getcwd()
files = os.listdir(current_directory)
for file in files:
    if file.endswith(".csv"):
        file_path = os.path.join(current_directory, file)
        os.remove(file_path)
        print(f"Deleted: {file_path}")
initial_pose = np.array([0.484,0.128,0.61,0.5,0.497,0.501,0.5])
bottom_left_pose = np.array([0.440,0.0,0.4292,0.924,0.38,0.015,0.0334])
bottom_right_pose = np.array([0.440,0.5,0.4292,0.480,0.876,0.033,0.016])
up_right_pose = np.array([0.6,0.5,0.4292,0.480,0.876,0.033,0.016])
up_left_pose = np.array([0.6,-0.05,0.4292,0.480,0.876,0.033,0.016])
# start_yellow_red = [0.4948, -0.1233, 0.4290, 0.8605, 0.5080, 0.0197, 0.0302]
# start_blu_green = [0.4934, 0.1288, 0.4290, 0.7082, 0.7050, 0.0265, 0.0243]
# end_red = [0.5926, 0.2154, 0.3332, 0.6461, 0.7422, 0.1352, 0.1155]
# end_blu = [0.3353, 0.5177, 0.3086, 0.3837, 0.9206, 0.0667, 0.0262]
# end_green = [0.4954, 0.2400, 0.2600, 0.6096, 0.7532, -0.1915, -0.1559]
# end_yellow = [0.5079, 0.3736, 0.3332, 0.5339, 0.8266, 0.1503, 0.0951]

init_pose = [0,0,0,0,0,0,0]
cube_data = {
    "red":{"pose_stamped_start":init_pose, "pose_stamped_dest":init_pose},
    "green":{"pose_stamped_start":init_pose, "pose_stamped_dest:":init_pose},
    "blu":{"pose_stamped_start":init_pose, "pose_stamped_dest:":init_pose},
    "yellow":{"pose_stamped_start":init_pose, "pose_stamped_dest:":init_pose},
}

class Moving_ur5(Node):
    def __init__(self):
        super().__init__('moving_ur5_subscriber')
        #self.csv_creation()
        #save_trajectory(initial, final, initial_quat, final_quat, time_range,time_step,filename)
        save_trajectory(initial_pose[:3],bottom_left_pose[:3],bottom_left_pose[-4:],bottom_left_pose[-4:],1, 0.001, 'bottom_right.csv')
        save_trajectory(bottom_left_pose[:3],bottom_right_pose[:3],bottom_right_pose[-4:],bottom_right_pose[-4:], 1, 0.001, 'bottom_left.csv')
        save_trajectory(bottom_right_pose[:3],up_right_pose[:3],up_right_pose[-4:], up_right_pose[-4:], 1,0.001, 'up_right.csv')
        save_trajectory(up_right_pose[:3], up_left_pose[:3], up_left_pose[-4:], up_left_pose[-4:], 1, 0.001, 'up_left.csv')
        
        
        # Load the CSV files into numpy arrays
        bottom_right_data = np.genfromtxt('bottom_right.csv', delimiter=',', skip_header=0, filling_values=np.nan)
        bottom_left_data = np.genfromtxt('bottom_left.csv', delimiter=',', skip_header=0, filling_values=np.nan)
        up_right_data = np.genfromtxt('up_right.csv', delimiter=',', skip_header=0, filling_values=np.nan)
        up_left_data = np.genfromtxt('up_left.csv', delimiter=',', skip_header=0, filling_values=np.nan)

        # Delete the first row from each array
        bottom_left_data = np.delete(bottom_left_data, 0, axis=0)
        up_right_data = np.delete(up_right_data, 0, axis=0)
        up_left_data = np.delete(up_left_data, 0, axis=0)
        
        # Concatenate the two arrays along axis 0
        output_data = np.concatenate([bottom_right_data, bottom_left_data,up_right_data, up_left_data])

        # Save the combined numpy array to a new CSV file
        np.savetxt('output_trajectory.csv', output_data, delimiter=',', comments='')

        
        with open("output_trajectory.csv") as f:
            self.pose_list = list(csv.reader(f, delimiter=","))


        #red start
        self.pose_red_start = []
        self.i_r_s = 0
        self.calculated_red_trajectory = False
        
        #red end
        self.pose_red_dest = []
        self.i_r_e = 0
        self.calculated_red_trajectory_end = False
        
        #blue start
        self.calculated_red_to_blue_trajectory = False   
        self.pose_red_to_blue=[]
        self.i_r_b = 0
        
        #blue end
        self.calculated_blue_trajectory = False
        self.i_b = 0
        self.pose_blu = []
        
        #yellow start
        self.calculated_blue_to_yellow_trajectory = False
        self.pose_blue_to_yellow = []
        self.i_b_y = 0
        
        #yellow end
        self.calculated_yellow_trajectory = False
        self.pose_yellow = []
        self.i_y = 0
        
        #green start
        self.pose_yellow_to_green = []
        self.i_y_g = 0
        self.calculated_yellow_to_green_trajectory = False
        
        #green dest 
        self.pose_green = []
        self.i_g = 0
        self.calculated_green_trajectory = False
        
        self.done = False
        self.fsm_state = 0
        self.cube_order = ["yellow","green","blu","red"]
        self.cube_idx = 0
        self.ee_rotations = {
            "green":quaternion_from_euler(0,0,0),
            "yellow":quaternion_from_euler(0,0,0),
            "blu":quaternion_from_euler(0,0,0),
            "red":quaternion_from_euler(0,0,0)
        }
        self.current_trajectory =  {"path": [], "time": [], "time_idx": 0}
        self.ee_actual_pose = PoseStamped()
        self.timer_period_publisher = 0.01 # seconds
        self.trajectory_st = 0.001 # seconds
        self.prev_pose_target = PoseStamped()
        
        #Pose publisher
        self.publisher_ = self.create_publisher(PoseStamped, '/ur5/ee_target/pose', 1)
        self.timer = self.create_timer(self.timer_period_publisher, self.timer_callback)
        self.i = 0
        
        
        #Gripper publisher
        self.Gripper_close = True
        self.Gripper_open = False
        self.Gripper_status = -1
        self.gripper_pub = self.create_publisher(Bool, '/wsg_50/controller/command', 1)
        
        #Pose subscriber
        self.subscription = self.create_subscription(PoseStamped, '/ur5/ee_actual/pose', self.callback_ee_actual_pose,10)
        self.subscription
        
        #Cube blu pos subscriber
        self.subscription_blu = self.create_subscription(PoseStamped, '/cube_blu/pose', self.callback_blu_cube_pose,10)
        self.subscription_blu
        
        # Cube red pos subscriber
        self.subscrition_red = self.create_subscription(PoseStamped, '/cube_red/pose', self.callback_red_cube_pose,10)
        self.subscrition_red
        
        # Cube yellow pos subscriber
        self.subscription_yellow = self.create_subscription(PoseStamped, '/cube_yellow/pose',self.callback_yellow_cube_pose,10)
        self.subscription_yellow
        
        # Cube green pos subscriber
        self.subscription_green = self.create_subscription(PoseStamped, '/cube_green/pose',self.callback_green_cube_pose,10)
        self.subscription_green
        
        # Cube blu destination subscriber
        self.subscription_blu_dest = self.create_subscription(PoseStamped, '/dest_cube_blu/pose', self.callback_blu_cube_pose_dest,10)
        self.subscription_blu_dest
        
        # Cube red destination subscriber
        self.subscription_red_dest = self.create_subscription(PoseStamped, '/dest_cube_red/pose', self.callback_red_cube_pose_dest,10)
        self.subscription_red_dest
        
        # Cube yellow destination subscriber
        self.subscription_yellow_dest = self.create_subscription(PoseStamped, '/dest_cube_yellow/pose',self.callback_yellow_cube_pose_dest,10)
        self.subscription_yellow_dest
        
        # Cube green destination subscriber
        self.subscription_green_dest = self.create_subscription(PoseStamped, '/dest_cube_green/pose',self.callback_green_cube_pose_dest,10)
        self.subscription_green_dest

    # def csv_creation(self):
    #     starting = 
    def timer_callback(self):
        # State 0: initial trajectory to detect all the cubes
        
        if (self.fsm_state == 0): #and 
        #     (cube_data["red"]["pose_stamped_start"] == init_pose or cube_data["red"]["pose_stamped_dest"] == init_pose or
        #     cube_data["yellow"]["pose_stamped_start"] == init_pose or cube_data["yellow"]["pose_stamped_dest"] == init_pose or
        #     cube_data["green"]["pose_stamped_start"] == init_pose or cube_data["green"]["pose_stamped_dest"] == init_pose or
        #     cube_data["blu"]["pose_stamped_start"] == init_pose or cube_data["blu"]["pose_stamped_dest"] == init_pose)):
            
            #print(self.done)
            if self.i > len(self.pose_list)-1:
                self.i -= 1
                
            pose_pub = PoseStamped()
            pose_pub.pose.position.x = float(self.pose_list[self.i][0])
            pose_pub.pose.position.y = float(self.pose_list[self.i][1])
            pose_pub.pose.position.z = float(self.pose_list[self.i][2])
            pose_pub.pose.orientation.x = float(np.cos((np.pi/2)/2)) #float(self.pose_list[self.i][3])
            pose_pub.pose.orientation.y =  float(np.sin((np.pi/2)/2))#float(self.pose_list[self.i][4])
            pose_pub.pose.orientation.z = 0.0#float(self.pose_list[self.i][5])
            pose_pub.pose.orientation.w = 0.0#float(self.pose_list[self.i][6])
            # print(pose_pub.pose.orientation.x)
            print(self.i)
            print(not (cube_data["red"]["pose_stamped_start"] == init_pose or cube_data["red"]["pose_stamped_dest"] == init_pose or
            cube_data["yellow"]["pose_stamped_start"] == init_pose or cube_data["yellow"]["pose_stamped_dest"] == init_pose or
            cube_data["green"]["pose_stamped_start"] == init_pose or cube_data["green"]["pose_stamped_dest"] == init_pose or
            cube_data["blu"]["pose_stamped_start"] == init_pose or cube_data["blu"]["pose_stamped_dest"] == init_pose))
            # while(not self.check_arrived_destination(pose_pub, 0.01) and not self.check_arrived_orientation(pose_pub)):
            #     print(not self.check_arrived_destination(pose_pub, 0.01))
            self.publisher_.publish(pose_pub)            
            self.i += 1
            #print(len(self.pose_list))
            
            if not (cube_data["red"]["pose_stamped_start"] == init_pose or cube_data["red"]["pose_stamped_dest"] == init_pose or
            cube_data["yellow"]["pose_stamped_start"] == init_pose or cube_data["yellow"]["pose_stamped_dest"] == init_pose or
            cube_data["green"]["pose_stamped_start"] == init_pose or cube_data["green"]["pose_stamped_dest"] == init_pose or
            cube_data["blu"]["pose_stamped_start"] == init_pose or cube_data["blu"]["pose_stamped_dest"] == init_pose):
                self.fsm_state = 2
            
        #I got all the poses now I open the gripper        
        if self.fsm_state == 1:
            print('opening grip')
            self.open_gripper()
            self.fsm_state = 2
        
        # move to the red block
        if self.fsm_state == 2:
            if not (self.calculated_red_trajectory):
                self.calculate_red_trajectory()
            
            
            print('current: ' + str(self.i_r_s) + '/' + str(len(self.pose_red_start)))
            if self.i_r_s > len(self.pose_red_start)-2:
                self.fsm_state = 3
            
            pose_pub = PoseStamped()
            pose_pub.pose.position.x = float(self.pose_red_start[self.i_r_s][0])
            pose_pub.pose.position.y = float(self.pose_red_start[self.i_r_s][1])
            pose_pub.pose.position.z = float(self.pose_red_start[self.i_r_s][2])
            pose_pub.pose.orientation.x = float(self.pose_red_start[self.i_r_s][3])
            pose_pub.pose.orientation.y =  float(self.pose_red_start[self.i_r_s][4])
            pose_pub.pose.orientation.z = float(self.pose_red_start[self.i_r_s][5])      
            pose_pub.pose.orientation.w = float(self.pose_red_start[self.i_r_s][6])     
            
            self.publisher_.publish(pose_pub)
            self.i_r_s += 1
            
        if self.fsm_state == 3:
            print('closing')
            time.sleep(5)
            self.close_gripper()
            time.sleep(5)
            self.fsm_state = 4
        
        # move to red destination
        if self.fsm_state == 4:
            if not (self.calculated_red_trajectory_end):
                self.calculate_red_trajectory_end()
            
            
            print('current: ' + str(self.i_r_e) + '/' + str(len(self.pose_red_dest)))
            if self.i_r_e > len(self.pose_red_dest)-2:
                self.fsm_state = 5
            
            pose_pub = PoseStamped()
            pose_pub.pose.position.x = float(self.pose_red_dest[self.i_r_e][0])
            pose_pub.pose.position.y = float(self.pose_red_dest[self.i_r_e][1])
            pose_pub.pose.position.z = float(self.pose_red_dest[self.i_r_e][2])
            pose_pub.pose.orientation.x = float(self.pose_red_dest[self.i_r_e][3])
            pose_pub.pose.orientation.y =  float(self.pose_red_dest[self.i_r_e][4])
            pose_pub.pose.orientation.z = float(self.pose_red_dest[self.i_r_e][5])      
            pose_pub.pose.orientation.w = float(self.pose_red_dest[self.i_r_e][6])  
            
            self.publisher_.publish(pose_pub)
            self.i_r_e += 1
            
        if self.fsm_state == 5:
            print('opening')
            time.sleep(5)
            self.open_gripper()
            time.sleep(5)
            self.fsm_state = 6
            
        if self.fsm_state == 6:
            if not (self.calculated_red_to_blue_trajectory):
                self.calculate_red_to_blue_trajectory()
                
            print('current: ' + str(self.i_r_b) + '/' + str(len(self.pose_red_to_blue)))
            if self.i_r_b > len(self.pose_red_to_blue)-2:
                self.fsm_state = 7
            
            pose_pub = PoseStamped()
            pose_pub.pose.position.x = float(self.pose_red_to_blue[self.i_r_b][0])
            pose_pub.pose.position.y = float(self.pose_red_to_blue[self.i_r_b][1])
            pose_pub.pose.position.z = float(self.pose_red_to_blue[self.i_r_b][2])
            pose_pub.pose.orientation.x = float(self.pose_red_to_blue[self.i_r_b][3])
            pose_pub.pose.orientation.y =  float(self.pose_red_to_blue[self.i_r_b][4])
            pose_pub.pose.orientation.z = float(self.pose_red_to_blue[self.i_r_b][5])      
            pose_pub.pose.orientation.w = float(self.pose_red_to_blue[self.i_r_b][6])      
            
            self.publisher_.publish(pose_pub)

            self.i_r_b += 1
            #ADD UP BLUE
            
        if self.fsm_state == 7:
            print('closing')
            time.sleep(5)
            self.close_gripper()
            time.sleep(5)
            self.fsm_state = 8
        
        if self.fsm_state == 8:
            #moving blue to its destination
            if not (self.calculated_blue_trajectory):
                self.calculate_blu_trajectory()
            
            print('current: ' + str(self.i_b) + '/' + str(len(self.pose_blu)))
            if self.i_b > len(self.pose_blu)-2:
                self.fsm_state = 9
                
            pose_pub = PoseStamped()
            pose_pub.pose.position.x = float(self.pose_blu[self.i_b][0])
            pose_pub.pose.position.y = float(self.pose_blu[self.i_b][1])
            pose_pub.pose.position.z = float(self.pose_blu[self.i_b][2])
            pose_pub.pose.orientation.x = float(self.pose_blu[self.i_b][3])
            pose_pub.pose.orientation.y =  float(self.pose_blu[self.i_b][4])
            pose_pub.pose.orientation.z = float(self.pose_blu[self.i_b][5])
            pose_pub.pose.orientation.w = float(self.pose_blu[self.i_b][6])
            
            self.publisher_.publish(pose_pub)
            self.i_b += 1
            
        if self.fsm_state == 9:
            print('opening')
            time.sleep(5)
            self.open_gripper()
            time.sleep(5)
            self.fsm_state = 10
            
        if self.fsm_state == 10:
            if not (self.calculated_blue_to_yellow_trajectory):
                self.calculate_blue_to_yellow_trajectory()
            
            print('current:' + str(self.i_b_y) + '/' + str(len(self.pose_blue_to_yellow)))
            if self.i_b_y > len(self.pose_blue_to_yellow)-2:
                self.fsm_state = 11
            pose_pub = PoseStamped()
            pose_pub.pose.position.x = float(self.pose_blue_to_yellow[self.i_b_y][0])
            pose_pub.pose.position.y = float(self.pose_blue_to_yellow[self.i_b_y][1])
            pose_pub.pose.position.z = float(self.pose_blue_to_yellow[self.i_b_y][2])
            pose_pub.pose.orientation.x = float(self.pose_blue_to_yellow[self.i_b_y][3])
            pose_pub.pose.orientation.y =  float(self.pose_blue_to_yellow[self.i_b_y][4])
            pose_pub.pose.orientation.z = float(self.pose_blue_to_yellow[self.i_b_y][5])
            pose_pub.pose.orientation.w = float(self.pose_blue_to_yellow[self.i_b_y][6])
            
            self.publisher_.publish(pose_pub)
            self.i_b_y += 1
            
        if self.fsm_state == 11:
            print('closing')
            time.sleep(5)
            self.close_gripper()
            time.sleep(5)
            self.fsm_state = 12
        
        if self.fsm_state == 12:
            if not(self.calculated_yellow_trajectory):
                self.calculate_yellow_trajectory()
                
            print('current:' + str(self.i_y) + '/' + str(len(self.pose_yellow)))
            if self.i_y > len(self.pose_yellow)-2:
                self.fsm_state = 13
            
            pose_pub = PoseStamped()
            pose_pub.pose.position.x = float(self.pose_yellow[self.i_y][0])
            pose_pub.pose.position.y = float(self.pose_yellow[self.i_y][1])
            pose_pub.pose.position.z = float(self.pose_yellow[self.i_y][2])
            pose_pub.pose.orientation.x = float(self.pose_yellow[self.i_y][3])
            pose_pub.pose.orientation.y =  float(self.pose_yellow[self.i_y][4])
            pose_pub.pose.orientation.z = float(self.pose_yellow[self.i_y][5])
            pose_pub.pose.orientation.w = float(self.pose_yellow[self.i_y][6])
            
            self.publisher_.publish(pose_pub)
            self.i_y += 1
    
        if self.fsm_state == 13:
            print('opening')
            time.sleep(5)
            self.open_gripper()
            time.sleep(5)
            self.fsm_state = 14
            
        
        #getting to green 
        if self.fsm_state == 14:
            
            if not(self.calculated_yellow_to_green_trajectory):
                self.calculate_yellow_to_green_trajectory()
            
            print('current:' + str(self.i_y_g) + '/' + str(len(self.pose_yellow_to_green)))
            if self.i_y_g > len(self.pose_yellow_to_green)-2:
                self.fsm_state = 15
            
            pose_pub = PoseStamped()
            pose_pub.pose.position.x = float(self.pose_yellow_to_green[self.i_y_g][0])
            pose_pub.pose.position.y = float(self.pose_yellow_to_green[self.i_y_g][1])
            pose_pub.pose.position.z = float(self.pose_yellow_to_green[self.i_y_g][2])
            pose_pub.pose.orientation.x = float(self.pose_yellow_to_green[self.i_y_g][3])
            pose_pub.pose.orientation.y =  float(self.pose_yellow_to_green[self.i_y_g][4])
            pose_pub.pose.orientation.z = float(self.pose_yellow_to_green[self.i_y_g][5])
            pose_pub.pose.orientation.w = float(self.pose_yellow_to_green[self.i_y_g][6])
            
            self.publisher_.publish(pose_pub)
            self.i_y_g += 1
    
        if self.fsm_state == 15:
            print('closing')
            time.sleep(5)
            self.close_gripper()
            time.sleep(5)
            self.fsm_state = 16
        
        if self.fsm_state == 16:
            if not(self.calculated_green_trajectory):
                self.calculate_green_trajectory()
            
            print('current:' + str(self.i_g) + '/' + str(len(self.pose_green)))
            if self.i_g > len(self.pose_green)-2:
                self.fsm_state = 17
            
            pose_pub = PoseStamped()
            pose_pub.pose.position.x = float(self.pose_green[self.i_g][0])
            pose_pub.pose.position.y = float(self.pose_green[self.i_g][1])
            pose_pub.pose.position.z = float(self.pose_green[self.i_g][2])
            pose_pub.pose.orientation.x = float(self.pose_green[self.i_g][3])
            pose_pub.pose.orientation.y =  float(self.pose_green[self.i_g][4])
            pose_pub.pose.orientation.z = float(self.pose_green[self.i_g][5])
            pose_pub.pose.orientation.w = float(self.pose_green[self.i_g][6])
            
            self.publisher_.publish(pose_pub)
            self.i_g += 1
        
        if self.fsm_state == 17:
            print('opening')
            time.sleep(5)
            self.open_gripper()
            time.sleep(5)
            
            
    def calculate_green_trajectory(self):
        start_x = cube_data["green"]["pose_stamped_start"].pose.position.x
        start_y = cube_data["green"]["pose_stamped_start"].pose.position.y
        start_z = cube_data["green"]["pose_stamped_start"].pose.position.z
        start_ori_x = cube_data["green"]["pose_stamped_start"].pose.orientation.x
        start_ori_y = cube_data["green"]["pose_stamped_start"].pose.orientation.y
        start_ori_z = cube_data["green"]["pose_stamped_start"].pose.orientation.z
        start_ori_w = cube_data["green"]["pose_stamped_start"].pose.orientation.w
        
        check1_x = start_x
        check1_y = start_y
        check1_z = start_z + 0.2
        check1_pos = np.array([check1_x, check1_y, check1_z])
        start_pos = np.array([start_x, start_y, start_z])
        start_ori = np.array([start_ori_x, start_ori_y, start_ori_z,start_ori_w])
        save_trajectory(start_pos,check1_pos,start_ori,start_ori,1,0.001,'green_up.csv')
        
        end_x = cube_data["green"]["pose_stamped_dest"].pose.position.x
        end_y = cube_data["green"]["pose_stamped_dest"].pose.position.y
        end_z = cube_data["green"]["pose_stamped_dest"].pose.position.z
        end_ori_x = cube_data["green"]["pose_stamped_dest"].pose.orientation.x
        end_ori_y = cube_data["green"]["pose_stamped_dest"].pose.orientation.y
        end_ori_z = cube_data["green"]["pose_stamped_dest"].pose.orientation.z
        end_ori_w = cube_data["green"]["pose_stamped_dest"].pose.orientation.w
        
        check2_x = end_x
        check2_y = end_y
        check2_z = end_z + 0.2
        
        check2_pos = np.array([check2_x, check2_y, check2_z])
        end_pos = np.array([end_x, end_y, end_z])
        end_ori = np.array([end_ori_x, end_ori_y, end_ori_z,end_ori_w])
        save_trajectory(check1_pos,check2_pos,end_ori,end_ori,1,0.001,'green_ori.csv')
        save_trajectory(check2_pos,end_pos,end_ori,end_ori,1,0.001,'green_down.csv')
        
        g_up = np.genfromtxt('green_up.csv', delimiter=',',skip_header= 0,filling_values = np.nan)
        g_ori = np.genfromtxt('green_ori.csv', delimiter=',',skip_header= 0,filling_values = np.nan)
        g_down = np.genfromtxt('green_down.csv', delimiter=',',skip_header= 0,filling_values = np.nan)
        g_up = np.delete(g_up, 0,axis = 0)
        g_ori = np.delete(g_ori, 0,axis = 0)
        g_up = np.delete(g_up, 0,axis = 0)
        
        output = np.concatenate((g_up,g_ori,g_down),axis = 0)
        np.savetxt('green_trajectory.csv', output, delimiter=',',comments='')
        with open('green_trajectory.csv') as f:
            self.pose_green = list(csv.reader(f, delimiter=','))  
            
        self.calculated_green_trajectory = True      
        
        
               
    def calculate_yellow_to_green_trajectory(self):
        start_x = cube_data["yellow"]["pose_stamped_dest"].pose.position.x
        start_y = cube_data["yellow"]["pose_stamped_dest"].pose.position.y
        start_z = cube_data["yellow"]["pose_stamped_dest"].pose.position.z - 0.03
        
        check1_x = start_x
        check1_y = start_y
        check1_z = start_z + 0.2
        
        check1_o_x = cube_data["yellow"]["pose_stamped_dest"].pose.orientation.x
        check1_o_y = cube_data["yellow"]["pose_stamped_dest"].pose.orientation.y
        check1_o_z = cube_data["yellow"]["pose_stamped_dest"].pose.orientation.z
        check1_o_w = cube_data["yellow"]["pose_stamped_dest"].pose.orientation.w
        
        start_pos = np.array([start_x, start_y, start_z])
        check1_pos = np.array([check1_x, check1_y, check1_z])
        check1_ori = np.array([check1_o_x, check1_o_y, check1_o_z,check1_o_w])
        save_trajectory(start_pos,check1_pos,check1_ori,check1_ori,1,0.001,'yellow_g_up.csv')
        
        end_x = cube_data["green"]["pose_stamped_start"].pose.position.x
        end_y = cube_data["green"]["pose_stamped_start"].pose.position.y
        end_z = cube_data["green"]["pose_stamped_start"].pose.position.z
        end_o_x = cube_data["green"]["pose_stamped_start"].pose.orientation.x
        end_o_y = cube_data["green"]["pose_stamped_start"].pose.orientation.y
        end_o_z = cube_data["green"]["pose_stamped_start"].pose.orientation.z
        end_o_w = cube_data["green"]["pose_stamped_start"].pose.orientation.w
        
        check2_x = end_x
        check2_y = end_y
        check2_z = end_z + 0.2
        
        check2_pos = np.array([check2_x, check2_y, check2_z])
        end_ori = np.array([end_o_x, end_o_y,end_o_z,end_o_w])
        end_pos = np.array([end_x, end_y, end_z])
        
        save_trajectory(check1_pos,check2_pos,end_ori,end_ori,1,0.001,'y_g_oriz.csv')
        save_trajectory(check2_pos,end_pos,end_ori,end_ori,1,0.001,'y_g_down.csv')
        
        y_g_updata = np.genfromtxt('yellow_g_up.csv',delimiter=',',skip_header = 0, filling_values=np.nan)
        y_g_updata = np.delete(y_g_updata, 0, axis=0)
        
        y_g_orizdata = np.genfromtxt('y_g_oriz.csv',delimiter=',',skip_header=0, filling_values=np.nan)
        y_g_orizdata = np.delete(y_g_orizdata, 0, axis=0)
    
        y_g_downdata = np.genfromtxt('y_g_down.csv',delimiter=',',skip_header=0, filling_values=np.nan)
        y_g_downdata = np.delete(y_g_downdata, 0, axis=0)
        
        output_data = np.concatenate((y_g_updata,y_g_orizdata,y_g_downdata),axis=0)
        np.savetxt('yellow_green_trajectory.csv',output_data,delimiter=',',comments='')
        
        with open('yellow_green_trajectory.csv') as f:
            self.pose_yellow_to_green = list(csv.reader(f,delimiter=','))
            
        self.calculated_yellow_to_green_trajectory = True
            
    def calculate_yellow_trajectory(self):
        
        start_x = cube_data["yellow"]["pose_stamped_start"].pose.position.x
        start_y = cube_data["yellow"]["pose_stamped_start"].pose.position.y
        start_z = cube_data["yellow"]["pose_stamped_start"].pose.position.z
        
        end_x = cube_data["yellow"]["pose_stamped_dest"].pose.position.x
        end_y = cube_data["yellow"]["pose_stamped_dest"].pose.position.y
        end_z = cube_data["yellow"]["pose_stamped_dest"].pose.position.z - 0.03
        end_o_x = cube_data["yellow"]["pose_stamped_dest"].pose.orientation.x
        end_o_y = cube_data["yellow"]["pose_stamped_dest"].pose.orientation.y
        end_o_z = cube_data["yellow"]["pose_stamped_dest"].pose.orientation.z
        end_o_w = cube_data["yellow"]["pose_stamped_dest"].pose.orientation.w
        
        start_pos = np.array([start_x, start_y, start_z])
        end_pos = np.array([end_x, end_y, end_z])
        end_ori = np.array([end_o_x, end_o_y, end_o_z,end_o_w])
        
        save_trajectory(start_pos,end_pos,end_ori,end_ori,1,0.001,'yellow_transfer.csv')
        yellow_data = np.genfromtxt('yellow_transfer.csv',delimiter=',',skip_header=0, filling_values=np.nan)
        yellow_data = np.delete(yellow_data, 0, axis=0)
        np.savetxt('yellow_transfer_e.csv', yellow_data, delimiter=',', comments='')
        with open('yellow_transfer_e.csv') as f:
            self.pose_yellow = list(csv.reader(f, delimiter=','))
                                    
        self.calculated_yellow_trajectory = True
    def calculate_blue_to_yellow_trajectory(self):
        
        start_x = cube_data["blu"]["pose_stamped_dest"].pose.position.x 
        start_y = cube_data["blu"]["pose_stamped_dest"].pose.position.y
        start_z = cube_data["blu"]["pose_stamped_dest"].pose.position.z
        
        check_1x = start_x
        check_1y = start_y
        check_1z = start_z + 0.2
        check_1ox = cube_data["blu"]["pose_stamped_dest"].pose.orientation.x 
        check_1oy = cube_data["blu"]["pose_stamped_dest"].pose.orientation.y
        check_1oz = cube_data["blu"]["pose_stamped_dest"].pose.orientation.z
        check_1ow = cube_data["blu"]["pose_stamped_dest"].pose.orientation.w
        
        
        
        end_x = cube_data["yellow"]["pose_stamped_start"].pose.position.x 
        end_y = cube_data["yellow"]["pose_stamped_start"].pose.position.y
        end_z = cube_data["yellow"]["pose_stamped_start"].pose.position.z
        end_o_x = cube_data["yellow"]["pose_stamped_start"].pose.orientation.x 
        end_o_y = cube_data["yellow"]["pose_stamped_start"].pose.orientation.y
        end_o_z = cube_data["yellow"]["pose_stamped_start"].pose.orientation.z
        end_o_w = cube_data["yellow"]["pose_stamped_start"].pose.orientation.w
        
        check_2x = end_x
        check_2y = end_y
        check_2z = end_z + 0.2
        
        start_pos = np.array([start_x, start_y, start_z])
        check_pos = np.array([check_1x, check_1y, check_1z])
        check_2pos = np.array([check_2x, check_2y, check_2z])
        end_pos = np.array([end_x, end_y, end_z])
        check_ori = np.array([check_1ox, check_1oy, check_1oz,check_1ow])
        end_ori = np.array([end_o_x, end_o_y, end_o_z,end_o_w])
        
        save_trajectory(start_pos,check_pos,check_ori,check_ori,1, 0.001, 'blue_yellow_up.csv')
        save_trajectory(check_pos,check_2pos,end_ori,end_ori,1, 0.001, 'blue_yellow_oriz.csv')
        save_trajectory(check_2pos,end_pos,end_ori,end_ori,1, 0.001, 'blue_yellow_down.csv')
        blu_up_data = np.genfromtxt('blue_yellow_up.csv', delimiter=',',skip_header = 0,filling_values = np.nan)
        blu_down_data = np.genfromtxt('blue_yellow_down.csv', delimiter=',',skip_header = 0,filling_values = np.nan)
        blu_up_data = np.delete(blu_up_data, 0,axis = 0)
        blu_down_data = np.delete(blu_down_data, 0,axis = 0)
        
        blu_oriz_data = np.genfromtxt('blue_yellow_oriz.csv', delimiter=',', skip_header = 0,filling_values = np.nan)
        blu_oriz_data = np.delete(blu_oriz_data, 0,axis = 0)
        output_data = np.concatenate([blu_up_data,blu_oriz_data,blu_down_data])
        np.savetxt('blue_yellow_transfer.csv', output_data, delimiter=',', comments= '')
        
        with open('blue_yellow_transfer.csv') as f:
            self.pose_blue_to_yellow = list(csv.reader(f, delimiter=','))
        
        self.calculated_blue_to_yellow_trajectory = True
    def calculate_blu_trajectory(self):
        start_x = cube_data["blu"]["pose_stamped_start"].pose.position.x 
        start_y = cube_data["blu"]["pose_stamped_start"].pose.position.y
        start_z = cube_data["blu"]["pose_stamped_start"].pose.position.z
        start_o_x = cube_data["blu"]["pose_stamped_start"].pose.orientation.x
        start_o_y = cube_data["blu"]["pose_stamped_start"].pose.orientation.y
        start_o_z = cube_data["blu"]["pose_stamped_start"].pose.orientation.z
        start_o_w = cube_data["blu"]["pose_stamped_start"].pose.orientation.w
        
        
        check_x = start_x
        check_y = start_y
        check_z = start_z + 0.2
        
        
        end_x = cube_data["blu"]["pose_stamped_dest"].pose.position.x 
        end_y = cube_data["blu"]["pose_stamped_dest"].pose.position.y
        end_z = cube_data["blu"]["pose_stamped_dest"].pose.position.z - 0.03
        end_o_x = cube_data["blu"]["pose_stamped_dest"].pose.orientation.x
        end_o_y = cube_data["blu"]["pose_stamped_dest"].pose.orientation.y
        end_o_z = cube_data["blu"]["pose_stamped_dest"].pose.orientation.z
        end_o_w = cube_data["blu"]["pose_stamped_dest"].pose.orientation.w
        
        check2_x = end_x
        check2_y = end_y
        check2_z = end_z + 0.2
        
        check2_pos = np.array([check2_x, check2_y, check2_z])
        
        start_pos = np.array([start_x, start_y, start_z])
        check1_pos = np.array([check_x, check_y, check_z])
        check_ori = np.array([start_o_x, start_o_y, start_o_z,start_o_w])
        end_pos = np.array([end_x, end_y, end_z])
        end_ori = np.array([end_o_x, end_o_y, end_o_z, end_o_w])
        save_trajectory(start_pos,check1_pos,check_ori,check_ori,1, 0.001, 'blue_up.csv')
        save_trajectory(check1_pos,check2_pos,end_ori,end_ori,1, 0.001,'blu_check2.csv')
        save_trajectory(check2_pos,end_pos,end_ori,end_ori,1, 0.001, 'blue_transfer.csv')
        
        blue_data = np.genfromtxt('blue_transfer.csv', delimiter=',',skip_header=0,filling_values=np.nan)
        blue_data = np.delete(blue_data, 0,axis =0)
        blu_up = np.genfromtxt('blue_up.csv', delimiter=',',skip_header=0,filling_values=np.nan)
        blu_up = np.delete(blu_up, 0,axis =0)
        blue_2 = np.genfromtxt('blu_check2.csv', delimiter=',', skip_header=0,filling_values=np.nan)
        blue_2 = np.delete(blue_2, 0,axis =0)
        output_data = np.concatenate([blu_up,blue_2,blue_data])
        np.savetxt('blue_transfer_e.csv', output_data, delimiter=',',comments='')
        
        with open('blue_transfer_e.csv') as f:
            self.pose_blu = list(csv.reader(f, delimiter = ","))
        
        self.calculated_blue_trajectory = True
        
        
    def calculate_red_to_blue_trajectory(self):
        start_x = cube_data["red"]["pose_stamped_dest"].pose.position.x 
        start_y = cube_data["red"]["pose_stamped_dest"].pose.position.y
        start_z = cube_data["red"]["pose_stamped_dest"].pose.position.z
        start_o_x = cube_data["red"]["pose_stamped_dest"].pose.orientation.x
        start_o_y = cube_data["red"]["pose_stamped_dest"].pose.orientation.y
        start_o_z = cube_data["red"]["pose_stamped_dest"].pose.orientation.z
        start_o_w = cube_data["red"]["pose_stamped_dest"].pose.orientation.w
        
        end_x = cube_data["blu"]["pose_stamped_start"].pose.position.x 
        end_y = cube_data["blu"]["pose_stamped_start"].pose.position.y
        end_z = cube_data["blu"]["pose_stamped_start"].pose.position.z
        end_o_x = cube_data["blu"]["pose_stamped_start"].pose.orientation.x
        end_o_y = cube_data["blu"]["pose_stamped_start"].pose.orientation.y
        end_o_z = cube_data["blu"]["pose_stamped_start"].pose.orientation.z
        end_o_w = cube_data["blu"]["pose_stamped_start"].pose.orientation.w
        
        up_x = start_x
        up_y = start_y
        up_z = start_z + 0.15
        
        start_pos = np.array([up_x, up_y, up_z])
        start_pos2 = np.array([start_x, start_y, start_z])
        start_ori = np.array([start_o_x, start_o_y, start_o_z, start_o_w])
        end_pos = np.array([end_x, end_y, end_z])
        end_ori = np.array([end_o_x, end_o_y, end_o_z, end_o_w])
        save_trajectory(start_pos2,start_pos,start_ori,start_ori,1, 0.001, 'blue_pick_up.csv')
        save_trajectory(start_pos,end_pos,end_ori,end_ori,1, 0.001, 'blue_pick_up_down.csv')
        
        red_to_blue_start_data = np.genfromtxt('blue_pick_up.csv', delimiter=',', skip_header=0,filling_values=np.nan)
        red_to_blue_start_data = np.delete(red_to_blue_start_data, 0, axis=0)
        red_to_blue_down_data = np.genfromtxt('blue_pick_up_down.csv', delimiter=',',skip_header=0,filling_values= np.nan)
        red_to_blue_down_data = np.delete(red_to_blue_down_data, 0, axis=0)
        
        output_data = np.concatenate([red_to_blue_start_data,red_to_blue_down_data])
        

        # Save the combined numpy array to a new CSV file
        np.savetxt('red_to_blue.csv', output_data, delimiter=',', comments='')
        
        with open ('red_to_blue.csv') as f:
            self.pose_red_to_blue = list(csv.reader(f, delimiter = ","))
        
        self.calculated_red_to_blue_trajectory = True
        
        
    def calculate_red_trajectory(self):
        start_x = cube_data["red"]["pose_stamped_start"].pose.position.x 
        start_y = cube_data["red"]["pose_stamped_start"].pose.position.y
        start_z = cube_data["red"]["pose_stamped_start"].pose.position.z
        start_o_x = cube_data["red"]["pose_stamped_start"].pose.orientation.x
        start_o_y = cube_data["red"]["pose_stamped_start"].pose.orientation.y
        start_o_z = cube_data["red"]["pose_stamped_start"].pose.orientation.z
        start_o_w = cube_data["red"]["pose_stamped_start"].pose.orientation.w
        start_pos = np.array([start_x, start_y, start_z])
        start_ori = np.array([start_o_x, start_o_y, start_o_z, start_o_w])
        save_trajectory(up_left_pose[:3],start_pos,start_ori,start_ori,1, 0.001, 'red_pick_up.csv')
    
        red_start_data = np.genfromtxt('red_pick_up.csv', delimiter=',', skip_header=0, filling_values=np.nan)
        red_start_data = np.delete(red_start_data, 0, axis=0)
        
        with open ('red_pick_up.csv') as f:
            self.pose_red_start = list(csv.reader(f, delimiter = ","))
        
        self.calculated_red_trajectory = True
        
    def calculate_red_trajectory_end(self):
        
        start_x = cube_data["red"]["pose_stamped_dest"].pose.position.x 
        start_y = cube_data["red"]["pose_stamped_dest"].pose.position.y
        start_z = cube_data["red"]["pose_stamped_dest"].pose.position.z - 0.03
        start_o_x = cube_data["red"]["pose_stamped_dest"].pose.orientation.x
        start_o_y = cube_data["red"]["pose_stamped_dest"].pose.orientation.y
        start_o_z = cube_data["red"]["pose_stamped_dest"].pose.orientation.z
        start_o_w = cube_data["red"]["pose_stamped_dest"].pose.orientation.w
        
        start_pos = np.array([start_x, start_y, start_z]) #remvoe something from z?
        start_pos_up = np.array([start_x, start_y, start_z+0.15])
        start_ori = np.array([start_o_x, start_o_y, start_o_z, start_o_w])
        
        #get the starting position (from the pick)
        astart_x = cube_data["red"]["pose_stamped_start"].pose.position.x 
        astart_y = cube_data["red"]["pose_stamped_start"].pose.position.y
        astart_z = cube_data["red"]["pose_stamped_start"].pose.position.z
        astart_pos = np.array([astart_x, astart_y, astart_z])
        first_up = np.array([astart_x, astart_y, astart_z+0.15])
        save_trajectory(astart_pos,first_up,start_ori,start_ori,1, 0.001, 'red_initial_up.csv')
        save_trajectory(first_up,start_pos_up,start_ori,start_ori,1, 0.001, 'red_place.csv')
        save_trajectory(start_pos_up,start_pos,start_ori,start_ori,1, 0.001, 'red_final_up.csv')
    
    
        red_start_up = np.genfromtxt('red_initial_up.csv', delimiter=',', skip_header=0, filling_values=np.nan)
        red_start_up = np.delete(red_start_up, 0, axis=0)
        red_start_data = np.genfromtxt('red_place.csv', delimiter=',', skip_header=0, filling_values=np.nan)
        red_start_data = np.delete(red_start_data, 0, axis=0)
        red_final_up = np.genfromtxt('red_final_up.csv', delimiter=',', skip_header=0, filling_values=np.nan)
        red_final_up = np.delete(red_final_up, 0, axis=0)
        output_data = np.concatenate([red_start_up, red_start_data,red_final_up])
        

        # Save the combined numpy array to a new CSV file
        np.savetxt('red_place.csv', output_data, delimiter=',', comments='')
        
        with open ('red_place.csv') as f:
            self.pose_red_dest = list(csv.reader(f, delimiter = ","))
        
        self.calculated_red_trajectory_end = True
             
    def copy_pose_stamped(self, pose_stamped):
        posa = PoseStamped()
        posa.pose.position.x = pose_stamped.pose.position.x
        posa.pose.position.y = pose_stamped.pose.position.y
        posa.pose.position.z = pose_stamped.pose.position.z
        posa.pose.orientation.x = 0.707#pose_stamped.pose.orientation.x
        posa.pose.orientation.y = 0.707#pose_stamped.pose.orientation.y
        posa.pose.orientation.z = 0#pose_stamped.pose.orientation.z
        posa.pose.orientation.w = 0#pose_stamped.pose.orientation.w
        return posa
    
    
    def check_arrived_destination(self, pose_target, epsilon):
        err_x = abs(self.ee_actual_pose.pose.position.x - pose_target.pose.position.x)
        err_y = abs(self.ee_actual_pose.pose.position.y - pose_target.pose.position.y)
        err_z = abs(self.ee_actual_pose.pose.position.z - pose_target.pose.position.z)
        total_error = err_x + err_y + err_z
        return total_error < epsilon
    
    def check_arrived_orientation(self, pose_target):
        err_x = abs(self.ee_actual_pose.pose.orientation.x - pose_target.pose.orientation.x)
        err_y = abs(self.ee_actual_pose.pose.orientation.y - pose_target.pose.orientation.y)
        err_z = abs(self.ee_actual_pose.pose.orientation.z - pose_target.pose.orientation.z)
        err_w = abs(self.ee_actual_pose.pose.orientation.w - pose_target.pose.orientation.w)
        total_error = err_x + err_y + err_z + err_w
        return total_error < 0.1
    
    
    def callback_ee_actual_pose(self, msg):
        self.ee_actual_pose = msg
        
    def callback_blu_cube_pose(self, msg):
        cube_data["blu"]["pose_stamped_start"] = msg
        quat_orig = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        q_rot = self.ee_rotations["blu"]
        q_new = quaternion_multiply(q_rot, quat_orig)
        cube_data["blu"]["pose_stamped_start"].pose.orientation.x = q_new[0]
        cube_data["blu"]["pose_stamped_start"].pose.orientation.y = q_new[1]
        cube_data["blu"]["pose_stamped_start"].pose.orientation.z = q_new[2]
        cube_data["blu"]["pose_stamped_start"].pose.orientation.w = q_new[3]
        
    def callback_red_cube_pose(self, msg):

        cube_data["red"]["pose_stamped_start"] = msg
        quat_orig = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        q_rot = self.ee_rotations["red"]
        q_new = quaternion_multiply(q_rot, quat_orig)
        cube_data["red"]["pose_stamped_start"].pose.orientation.x = q_new[0]
        cube_data["red"]["pose_stamped_start"].pose.orientation.y = q_new[1]
        cube_data["red"]["pose_stamped_start"].pose.orientation.z = q_new[2]
        cube_data["red"]["pose_stamped_start"].pose.orientation.w = q_new[3]
        
    def callback_green_cube_pose(self, msg):

        cube_data["green"]["pose_stamped_start"] = msg
        quat_orig = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        q_rot = self.ee_rotations["green"]
        q_new = quaternion_multiply(q_rot, quat_orig)
        cube_data["green"]["pose_stamped_start"].pose.orientation.x = q_new[0] 
        cube_data["green"]["pose_stamped_start"].pose.orientation.y = q_new[1]
        cube_data["green"]["pose_stamped_start"].pose.orientation.z = q_new[2]
        cube_data["green"]["pose_stamped_start"].pose.orientation.w = q_new[3]
        
    def callback_yellow_cube_pose(self, msg):

        cube_data["yellow"]["pose_stamped_start"] = msg
        quat_orig = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        q_rot = self.ee_rotations["yellow"]
        q_new = quaternion_multiply(q_rot, quat_orig)
        cube_data["yellow"]["pose_stamped_start"].pose.orientation.x = q_new[0]
        cube_data["yellow"]["pose_stamped_start"].pose.orientation.y = q_new[1]
        cube_data["yellow"]["pose_stamped_start"].pose.orientation.z = q_new[2]
        cube_data["yellow"]["pose_stamped_start"].pose.orientation.w = q_new[3]
    
    def callback_blu_cube_pose_dest(self, msg):
        cube_data["blu"]["pose_stamped_dest"] = msg
        quat_orig = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        q_rot = self.ee_rotations["blu"]
        q_new = quaternion_multiply(q_rot, quat_orig)
        cube_data["blu"]["pose_stamped_dest"].pose.orientation.x = q_new[0]
        cube_data["blu"]["pose_stamped_dest"].pose.orientation.y = q_new[1]
        cube_data["blu"]["pose_stamped_dest"].pose.orientation.z = q_new[2]
        cube_data["blu"]["pose_stamped_dest"].pose.orientation.w = q_new[3]
        
    def callback_red_cube_pose_dest(self, msg):
        cube_data["red"]["pose_stamped_dest"] = msg 
        quat_orig = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w] 
        q_rot = self.ee_rotations["red"] 
        q_new = quaternion_multiply(q_rot, quat_orig) 
        cube_data["red"]["pose_stamped_dest"].pose.orientation.x = q_new[0]
        cube_data["red"]["pose_stamped_dest"].pose.orientation.y = q_new[1]
        cube_data["red"]["pose_stamped_dest"].pose.orientation.z = q_new[2]
        cube_data["red"]["pose_stamped_dest"].pose.orientation.w = q_new[3]
        
    def callback_green_cube_pose_dest(self, msg): 
        cube_data["green"]["pose_stamped_dest"] = msg 
        quat_orig = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w] 
        q_rot = self.ee_rotations["green"] 
        q_new = quaternion_multiply(q_rot, quat_orig)
        cube_data["green"]["pose_stamped_dest"].pose.orientation.x = q_new[0]
        cube_data["green"]["pose_stamped_dest"].pose.orientation.y = q_new[1]
        cube_data["green"]["pose_stamped_dest"].pose.orientation.z = q_new[2]
        cube_data["green"]["pose_stamped_dest"].pose.orientation.w = q_new[3]
        
    def callback_yellow_cube_pose_dest(self, msg): 
        cube_data["yellow"]["pose_stamped_dest"] = msg 
        quat_orig = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w] 
        q_rot = self.ee_rotations["yellow"] 
        q_new = quaternion_multiply(q_rot, quat_orig) 
        cube_data["yellow"]["pose_stamped_dest"].pose.orientation.x = q_new[0]
        cube_data["yellow"]["pose_stamped_dest"].pose.orientation.y = q_new[1]
        cube_data["yellow"]["pose_stamped_dest"].pose.orientation.z = q_new[2]
        cube_data["yellow"]["pose_stamped_dest"].pose.orientation.w = q_new[3]
        
        
    def close_gripper(self):
        if self.Gripper_status == self.Gripper_close:
            return
        
        gripper_bool = Bool()
        gripper_bool.data = self.Gripper_close
        for i in range(20):
            self.gripper_pub.publish(gripper_bool)
        self.Gripper_status = self.Gripper_close
        
    def open_gripper(self):
        if self.Gripper_status == self.Gripper_open:
            return
        
        gripper_bool = Bool()
        gripper_bool.data = self.Gripper_open
        for i in range(10):
            self.gripper_pub.publish(gripper_bool)
        print('done')
        self.Gripper_status = self.Gripper_open
        
def main(args=None):
    rclpy.init(args=args)
    moving_ur5_subscriber = Moving_ur5()
    rclpy.spin(moving_ur5_subscriber)
    moving_ur5_subscriber.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()