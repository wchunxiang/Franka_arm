import numpy as np
from scipy.interpolate import make_interp_spline


def make_interp(keys_x, keys_y, keys_z, point_num=5000):
    t = np.linspace(0,1,keys_x.shape[0])
    key_p = np.zeros((keys_x.shape[0],3))
    key_p[:,0]=keys_x
    key_p[:,1]=keys_y
    key_p[:,2]=keys_z
    spl = make_interp_spline(t,key_p,bc_type="natural")

    t_new = np.linspace(0, 1, point_num)
    x_new, y_new, z_new = spl(t_new).T

    return x_new, y_new, z_new

def correct_path(xyzs_old: tuple, xyz_p: tuple, seg_ratio: float, discard_thresh = 0.6, interp_no = 10000):
    xs, ys, zs = xyzs_old
    x_a,y_a,z_a = xyz_p

    #caculate the cumsum length
    diff_x = np.r_[0,np.diff(xs)]
    diff_y = np.r_[0,np.diff(ys)]
    len_cum = np.cumsum(np.sqrt(diff_x**2+diff_y**2))
    #find the nearest points
    len_affect = seg_ratio*len_cum[-1]
    dist_temp = np.sqrt((xs-x_a)**2 + (ys-y_a)**2)
    indice_min = np.argmin(dist_temp)
    #if the point is too far from the original trajectory
    if dist_temp[indice_min] > len_affect*discard_thresh:
        return None, None, None, None, False
    #update the trajectory if the point is not that far
    else:
        x_temp,y_temp,z_temp = xs[indice_min],ys[indice_min],zs[indice_min]
        dist = np.sqrt((xs-x_temp)**2 + (ys-y_temp)**2)
        # find the points before and behind
        indice_dist = np.where(dist<len_affect)[0]
        indice_before = np.min(indice_dist)
        indice_behind = np.max(indice_dist)
        points_before = int(np.round(len_cum[indice_before]/len_affect, 0))
        points_after = int(np.round((len_cum[-1]-len_cum[indice_behind])/len_affect, 0))
        #check if close to start and end
        if (points_before > 1) and (points_after > 1):
            keys_before = np.linspace(0,indice_before,points_before).astype(int)
            keys_after = np.linspace(indice_behind,xs.shape[0]-1,points_after).astype(int)  
            #interpolate the spline
            keys_x = np.r_[xs[keys_before],x_a,xs[keys_after]]
            keys_y = np.r_[ys[keys_before],y_a,ys[keys_after]]
            keys_z = np.r_[zs[keys_before],z_a,zs[keys_after]]
            xs_new, ys_new, zs_new = make_interp(keys_x,keys_y,keys_z,point_num=interp_no)
            rads_xy_new = np.arctan2(np.gradient(ys_new),np.gradient(xs_new))
            return xs_new, ys_new, zs_new, rads_xy_new, True
        else:
            return None, None, None, None, False