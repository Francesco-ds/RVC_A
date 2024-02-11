import numpy as np

def generate_linear_trajectory(initial, final, initial_quat, final_quat, time_range, time_step):
    
    delta_pos = final-initial
    delta_pos_norm = np.linalg.norm(delta_pos)
    s = np.arange(0,delta_pos_norm,time_step)
    p_x = initial[0] + s * delta_pos[0] / delta_pos_norm
    p_y = initial[1] + s * delta_pos[1] / delta_pos_norm   
    p_z = initial[2] + s * delta_pos[2] / delta_pos_norm
    quaternion_trajectory_x = np.array([final_quat[0] for t in s])
    quaternion_trajectory_y = np.array([final_quat[1] for t in s])
    quaternion_trajectory_z = np.array([final_quat[2] for t in s])
    quaternion_trajectory_w = np.array([final_quat[3] for t in s])
    path = np.array([p_x.T, p_y.T, p_z.T,quaternion_trajectory_x.T,quaternion_trajectory_y.T,quaternion_trajectory_z.T,quaternion_trajectory_w.T])
    return path.T

def save_trajectory(initial, final, initial_quat, final_quat, time_range,time_step,filename):
    #trajectory = generate_bezier_trajectory(initial, final, initial_quat, final_quat, time_range, time_step)
    trajectory = generate_linear_trajectory(initial, final, initial_quat, final_quat, time_range,time_step)
    with open(filename, 'a', newline='') as csvfile:
        np.savetxt(csvfile, trajectory, delimiter=',', header='', comments='')
