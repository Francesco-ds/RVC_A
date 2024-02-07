import numpy as np

# def normalize_quaternion(q):
#     norm = np.linalg.norm(q)
#     return q / norm

# def slerp(q1, q2, t):
#     q1 = normalize_quaternion(q1)
#     q2 = normalize_quaternion(q2)

#     dot_product = np.dot(q1, q2)

#     if dot_product < 0.0:
#         q1 = -q1
#         dot_product = -dot_product

#     if dot_product > 0.9995:
#         result = q1 + t * (q2 - q1)
#         return normalize_quaternion(result)

#     theta_0 = np.arccos(dot_product)
#     sin_theta_0 = np.sin(theta_0)

#     theta = theta_0 * t
#     sin_theta = np.sin(theta)
#     sin_theta_1 = np.sin(theta_0 - theta)

#     ratio_q1 = sin_theta_1 / sin_theta_0
#     ratio_q2 = sin_theta / sin_theta_0

#     result = q1 * ratio_q1 + q2 * ratio_q2

#     return normalize_quaternion(result)


# def bezier_curve(t, initial, final):
#     n = len(initial) - 1
#     b = [(1 - t) ** (n - i) * t ** i for i in range(n + 1)]
    
#     curve_point = np.zeros_like(initial)
    
#     for i in range(n + 1):
#         curve_point += b[i] * (final - initial)

#     return initial + curve_point

# def generate_bezier_trajectory(initial, final, initial_quat, final_quat, time_range, time_step=0.01):
#     t_values = np.arange(0, time_range + time_step, time_step)
#     position_trajectory = [bezier_curve(t, initial, final) for t in t_values]
#     #quaternion_trajectory = [slerp(initial_quat, final_quat, t) for t in t_values]
#     quaternion_trajectory = [final_quat for t in t_values]
#     combined_trajectory = np.column_stack((position_trajectory, quaternion_trajectory))

#     return combined_trajectory


#Actually used
def generate_linear_trajectory(initial, final, initial_quat, final_quat, time_range, time_step):
    
    delta_pos = final-initial
    delta_pos_norm = np.linalg.norm(delta_pos)
    s = np.arange(0,delta_pos_norm,time_step)
    p_x = initial[0] + s * delta_pos[0] / delta_pos_norm
    p_y = initial[1] + s * delta_pos[1] / delta_pos_norm   
    p_z = initial[2] + s * delta_pos[2] / delta_pos_norm
    # quaternion_trajectory_x = np.array([slerp(initial_quat[0], final_quat[0], t) for t in s])
    # quaternion_trajectory_y = np.array([slerp(initial_quat[1], final_quat[1], t) for t in s])
    # quaternion_trajectory_z = np.array([slerp(initial_quat[2], final_quat[2], t) for t in s])
    # quaternion_trajectory_w = np.array([slerp(initial_quat[3], final_quat[3], t) for t in s])
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
