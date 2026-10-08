import numpy as np
from scipy.interpolate import make_interp_spline


def make_interp(keys_x, keys_y, keys_z, point_num=10000):
    t = np.linspace(0,1,keys_x.shape[0])
    key_p = np.zeros((keys_x.shape[0],3))
    key_p[:,0]=keys_x
    key_p[:,1]=keys_y
    key_p[:,2]=keys_z
    spl = make_interp_spline(t,key_p,bc_type="natural")

    t_new = np.linspace(0, 1, point_num)
    x_new, y_new, z_new = spl(t_new).T

    return x_new, y_new, z_new