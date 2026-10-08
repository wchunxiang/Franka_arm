import numpy as np
from numpy.linalg import inv
from scipy.linalg import block_diag
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

class AdaptiveFilter(object):
    def __init__(self, position, R_s = [10, 10, 10], Q_s = [0.5, 1, 3],p_s=[0.2, 0.4, 0.4]):
        self.dt = 1  # delta time
        self.std = 5
        position = np.asarray(position).reshape(-1,2)
        self.pos = np.array([position[0][0],position[0][1]],np.float)
        self.vel = np.array([0.,0.])
        self.acc = np.array([0.,0.])
        
        self.eps_p = 0
        self.eps_v = 0
        self.eps_a = 0

        self.cpfilter = self.make_cp_filter(self.dt, self.std)
        self.cvfilter = self.make_cv_filter(self.dt, self.std)
        self.cafilter = self.make_ca_filter(self.dt, self.std)

        self.initialize_cp_filter(self.cpfilter, pos_init=[self.pos[0], self.pos[1]], std_R=R_s[0], std_Q=Q_s[0])
        self.initialize_cv_filter(self.cvfilter, pos_init=[self.pos[0], self.pos[1]], std_R=R_s[1], std_Q=Q_s[1])
        self.initialize_ca_filter(self.cafilter, pos_init=[self.pos[0], self.pos[1]], std_R=R_s[2], std_Q=Q_s[2])

        self.Q_scale_factor_p = 3.
        self.Q_scale_factor_v = 5.
        self.Q_scale_factor_a = 100.
        
        self.count_P = 0
        self.count_V = 0
        self.count_A = 0
        
        self.eps_max_p = 20
        self.eps_max_v = 15
        self.eps_max_a = 6

        p_s=np.array(p_s)/np.sum(np.array(p_s))

        self.pp = p_s[0]
        self.pv = p_s[1]
        self.pa = p_s[2]
    
    def predict(self):
        self.cpfilter.predict()
        self.cvfilter.predict()
        self.cafilter.predict()

        x  = self.cpfilter.x[0] * self.pp + self.cvfilter.x[0] * self.pv + self.cafilter.x[0] * self.pa
        y  = self.cpfilter.x[1] * self.pp + self.cvfilter.x[2] * self.pv + self.cafilter.x[3] * self.pa
        vx = self.cvfilter.x[1] * self.pv + self.cafilter.x[1] * self.pa
        vy = self.cvfilter.x[3] * self.pv + self.cafilter.x[4] * self.pa
        ax = self.cafilter.x[3] * self.pa
        ay = self.cafilter.x[5] * self.pa

        self.pos = np.array([x, y])
        self.vel = np.array([vx, vy])
        self.acc = np.array([ax, ay])

    def update(self, pos):
        pos = np.asarray(pos).reshape(-1, 2)
        pos = np.array([pos[0][0],pos[0][1]],np.float)
        self.cpfilter.update(pos)
        y, S = self.cpfilter.y, self.cpfilter.S
        self.eps_p = np.dot(y.T, inv(S)).dot(y)
        
        if self.eps_p > self.eps_max_p:
            self.cpfilter.Q *= self.Q_scale_factor_p
            self.count_P += 1
        elif  self.count_P > 0:
            self.cpfilter.Q /= self.Q_scale_factor_p
            self.count_P -= 1

        self.cvfilter.update(pos)
        y, S = self.cvfilter.y, self.cvfilter.S
        self.eps_v = np.dot(y.T, inv(S)).dot(y)

        if self.eps_v > self.eps_max_v:
            self.cvfilter.Q *= self.Q_scale_factor_v
            self.count_V += 1
        elif  self.count_V > 0:
            self.cvfilter.Q /= self.Q_scale_factor_v
            self.count_V -= 1
        
        self.cafilter.update(pos)
        y, S = self.cafilter.y, self.cafilter.S
        self.eps_a = np.dot(y.T, inv(S)).dot(y)
        if self.eps_a > self.eps_max_a:
            self.cafilter.Q *= self.Q_scale_factor_a
            self.count_A += 1
        elif  self.count_A > 0:
            self.cafilter.Q /= self.Q_scale_factor_a
            self.count_A -= 1

        flag1 = (self.cafilter.x[1]) * (self.cafilter.x[1]) + (self.cafilter.x[4]) * (self.cafilter.x[4])
        thresh1 = 8 * 8
        thresh2 = 15*15

        if flag1 <= thresh1:
            self.pp, self.pv, self.pa = 0.8, 0.1, 0.1
        elif thresh1 < flag1 < thresh2:
            self.pp, self.pv, self.pa = 0.4, 0.5, 0.1
        
        if self.eps_a > self.eps_max_a:
            self.pp, self.pv, self.pa = 0.2, 0.2, 0.6
        
        cp_likelihood = self.cpfilter.likelihood * self.pp 
        cv_likelihood = self.cvfilter.likelihood * self.pv
        ca_likelihood = self.cafilter.likelihood * self.pa

        self.pp = (cp_likelihood) / (cv_likelihood + ca_likelihood + cp_likelihood)       
        self.pv = (cv_likelihood) / (cv_likelihood + ca_likelihood + cp_likelihood)
        self.pa = (ca_likelihood) / (cv_likelihood + ca_likelihood + cp_likelihood)
        
        x  = self.cpfilter.x[0] * self.pp + self.cvfilter.x[0] * self.pv + self.cafilter.x[0] * self.pa
        y  = self.cpfilter.x[1] * self.pp + self.cvfilter.x[2] * self.pv + self.cafilter.x[3] * self.pa
        vx = self.cvfilter.x[1] * self.pv + self.cafilter.x[1] * self.pa
        vy = self.cvfilter.x[3] * self.pv + self.cafilter.x[4] * self.pa
        ax = self.cafilter.x[3] * self.pa
        ay = self.cafilter.x[5] * self.pa

        self.pos = np.array([x, y])
        self.vel = np.array([vx, vy])
        self.acc = np.array([ax, ay])
    

    def make_cp_filter(self, dt, R_std):
        cpfilter = KalmanFilter(dim_x = 2, dim_z=2)
        cpfilter.x = np.array([0., 0.])
        cpfilter.P *= 3
        cpfilter.R *= np.eye(2) * R_std**2
        cpfilter.F = np.array([[1., 0],
                                [0, 1.]], dtype=float)
        cpfilter.H = np.array([[1., 0],
                                [0, 1.]], dtype=float)
        cpfilter.Q = np.eye(2) * 1
        return cpfilter

    
    def make_cv_filter(self, dt, R_std):
        cvfilter = KalmanFilter(dim_x = 4, dim_z=2)
        cvfilter.x = np.array([0., 0., 0., 0.])
        cvfilter.P *= 3
        cvfilter.R *= np.eye(2) * R_std**2
        cvfilter.F = np.array([[1, dt, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 1, dt],
                                [0, 0, 0, 1]], dtype=float)
        cvfilter.H = np.array([[1., 0, 0, 0],
                                [0, 0, 1., 0]], dtype=float)
        q = Q_discrete_white_noise(dim=2, dt=dt, var=0.01)
        cvfilter.Q =  block_diag(q, q)
        return cvfilter
    
    def make_ca_filter(self, dt, R_std):
        cafilter = KalmanFilter(dim_x = 6, dim_z=2)
        cafilter.x = np.array([0., 0., 0., 0., 0., 0.])
        cafilter.P *= 3
        cafilter.R *= np.eye(2) * R_std**2
        cafilter.F = np.array([[1, dt, 0.5*dt*dt, 0, 0, 0],
                            [0, 1,  dt,        0, 0, 0],
                            [0, 0,  1,         0, 0, 0],
                            [0, 0,  0,         1, dt, 0.5*dt*dt],
                            [0, 0,  0,         0, 1, dt],
                            [0, 0,  0,         0, 0, 1]], dtype=float)
        cafilter.H = np.array([[1., 0, 0, 0, 0, 0],
                                [0, 0, 0, 1.,0, 0]], dtype=float)
        q = Q_discrete_white_noise(dim=3, dt=dt, var=0.01)
        cafilter.Q =  block_diag(q, q)
        return cafilter
    
    def initialize_cp_filter(self, kf, pos_init = None, std_R=None, std_Q=None):
        if pos_init is None:
            pass 
        else:
            x_init = pos_init[0]
            y_init = pos_init[1]
            kf.x = np.array([x_init,y_init])    
            kf.P = np.eye(kf.dim_x) * 1   
        if std_R is not None:
            kf.R = np.eye(kf.dim_z) * std_R
        if std_Q is not None:
            kf.Q = np.eye(kf.dim_x) * std_Q
    
    def initialize_cv_filter(self, kf, pos_init = None, std_R=None, std_Q=None ):
        if pos_init is None:
            pass
        else:
            x_init = pos_init[0]
            y_init = pos_init[1]
            kf.x = np.array([x_init,0,y_init,0])    
            kf.P = np.eye(kf.dim_x) * 1
        if std_R is not None:
            kf.R = np.eye(kf.dim_z) * std_R
        if std_Q is not None:
            q = Q_discrete_white_noise(dim=2, dt=self.dt, var=std_Q)
            kf.Q = block_diag(q, q)
    
    
    def initialize_ca_filter(self, kf, pos_init = None, std_R=None, std_Q=None ):
        if pos_init is None:
            pass
        else:
            x_init = pos_init[0]
            y_init = pos_init[1]
            kf.x = np.array([x_init,0,0,y_init,0,0])    
            kf.P = np.eye(kf.dim_x) * 1
        if std_R is not None:
            kf.R = np.eye(kf.dim_z) * std_R
        if std_Q is not None:
            q = Q_discrete_white_noise(dim=3, dt=self.dt, var=std_Q)
            kf.Q = block_diag(q, q)



