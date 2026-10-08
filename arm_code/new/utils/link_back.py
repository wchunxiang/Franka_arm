import tf.transformations as ttf
import numpy as np


def link_back(end_x, end_y, end_z, end_roll, end_pitch, end_yaw, end_effector_length):

    end_length = end_effector_length # [m]

    p_end = np.mat([end_x, end_y, end_z, 1.0]).T
    p_local = np.mat([0, 0, end_length, 1.0]).T
    origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)

    quaternion = ttf.quaternion_from_euler(end_roll, end_pitch, end_yaw,'rxyz')
    Rq = np.mat(ttf.quaternion_matrix(quaternion))
    p_link = p_end - Rq@p_local

    link_x, link_y, link_z = np.array(p_link).reshape(-1)[:3] 
    link_roll, link_pitch, link_yaw = end_roll, end_pitch, end_yaw
    return [link_x, link_y, link_z, link_roll, link_pitch, link_yaw]

def link_back_L(end_x, end_y, end_z, end_roll, end_pitch, end_yaw, end_effector_height, end_effector_length, d_gamma = 0): # [m]

    p_end = np.mat([end_x, end_y, end_z, 1.0]).T
    p_local = np.mat([0, end_effector_length, end_effector_height, 1.0]).T
    origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
    #FIXME: switch pitch angle and yaw angle
    link_roll, link_yaw, link_pitch = end_roll, end_pitch, end_yaw

    quaternion = ttf.quaternion_from_euler(link_roll, link_pitch, link_yaw,'rxyz')
    Rq = np.mat(ttf.quaternion_matrix(quaternion))
    p_link = p_end - Rq@p_local

    link_x, link_y, link_z = np.array(p_link).reshape(-1)[:3] 
    
    return [link_x, link_y, link_z, link_roll, link_pitch, link_yaw]


def link_back4normal(end_x, end_y, end_z, end_roll, end_pitch, end_yaw, disp, yaw_offset2x):
    '''yaw_offset2x: probe yaw angle relative to the x axis'''
    

    p_end = np.mat([end_x, end_y, end_z, 1.0]).T
    p_local = np.mat([0, disp, 0, 1.0]).T
    origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
    quaternion = ttf.quaternion_from_euler(end_roll, end_pitch, end_yaw,'rxyz')
    Rq = np.mat(ttf.quaternion_matrix(quaternion))

    p_end_next = p_end + Rq@p_local

    p_x, p_y, p_z = np.array(p_end_next).reshape(-1)[:3] 
    p_roll, p_pitch, p_yaw = end_roll, end_pitch, end_yaw
    return [p_x, p_y, p_z, p_roll, p_pitch, p_yaw]

def link_back4tangent(end_x, end_y, end_z, end_roll, end_pitch, end_yaw, disp, yaw_offset2x):
    '''yaw_offset2x: probe yaw angle relative to the x axis'''
    

    p_end = np.mat([end_x, end_y, end_z, 1.0]).T
    p_local = np.mat([disp, 0, 0, 1.0]).T
    origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
    quaternion = ttf.quaternion_from_euler(end_roll, end_pitch, end_yaw,'rxyz')
    Rq = np.mat(ttf.quaternion_matrix(quaternion))


    p_end_next = p_end + Rq@p_local

    p_x, p_y, p_z = np.array(p_end_next).reshape(-1)[:3] 
    p_roll, p_pitch, p_yaw = end_roll, end_pitch, end_yaw
    return [p_x, p_y, p_z, p_roll, p_pitch, p_yaw]

def link_back_deprecate(end_x, end_y, end_z, end_roll, end_pitch, end_yaw, end_effector_length):

    end_length = end_effector_length # [m]

    p_end = np.mat([end_x, end_y, end_z, 1.0]).T
    p_local = np.mat([0, 0, end_length, 1.0]).T
    origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
    Rx = np.mat(ttf.rotation_matrix(end_roll, xaxis))
    Ry = np.mat(ttf.rotation_matrix(end_pitch, yaxis))
    Rz = np.mat(ttf.rotation_matrix(end_yaw, zaxis))


    p_link = p_end - Rz@Ry@Rx@p_local
    
    #print(p_end - p_link)

    link_x, link_y, link_z = np.array(p_link).reshape(-1)[:3] 
    link_roll, link_pitch, link_yaw = end_roll, end_pitch, end_yaw
    return [link_x, link_y, link_z, link_roll, link_pitch, link_yaw]

def link_back4tangent_deprecate(end_x, end_y, end_z, end_roll, end_pitch, end_yaw, disp, yaw_offset2x):
    '''yaw_offset2x: probe yaw angle relative to the x axis'''
    

    p_end = np.mat([end_x, end_y, end_z, 1.0]).T
    p_local = np.mat([disp, 0, 0, 1.0]).T
    origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
    Rx = np.mat(ttf.rotation_matrix(end_roll, xaxis))
    Ry = np.mat(ttf.rotation_matrix(end_pitch, yaxis))
    Rz = np.mat(ttf.rotation_matrix(end_yaw+yaw_offset2x, zaxis))


    p_end_next = p_end + Rz@Ry@Rx@p_local

    p_x, p_y, p_z = np.array(p_end_next).reshape(-1)[:3] 
    p_roll, p_pitch, p_yaw = end_roll, end_pitch, end_yaw
    return [p_x, p_y, p_z, p_roll, p_pitch, p_yaw]

def link_back4normal_deprecate(end_x, end_y, end_z, end_roll, end_pitch, end_yaw, disp, yaw_offset2x):
    '''yaw_offset2x: probe yaw angle relative to the x axis'''
    

    p_end = np.mat([end_x, end_y, end_z, 1.0]).T
    p_local = np.mat([0, disp, 0, 1.0]).T
    origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
    Rx = np.mat(ttf.rotation_matrix(end_roll, xaxis))
    Ry = np.mat(ttf.rotation_matrix(end_pitch, yaxis))
    Rz = np.mat(ttf.rotation_matrix(end_yaw+yaw_offset2x, zaxis))


    p_end_next = p_end + Rz@Ry@Rx@p_local

    p_x, p_y, p_z = np.array(p_end_next).reshape(-1)[:3] 
    p_roll, p_pitch, p_yaw = end_roll, end_pitch, end_yaw
    return [p_x, p_y, p_z, p_roll, p_pitch, p_yaw]