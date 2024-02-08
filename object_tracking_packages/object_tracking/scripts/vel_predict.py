import numpy as np
import math
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Point

INIT_POS_STD = 0.1
INIT_VEL_STD = 0.1
ACCEL_STD = 1.5
OBS_STD = 0.3

# R이 커지거나 Q가 작아지면 K 값 증가 -> 측정값이 반영되는 비율 커짐
class VelKalman:
    def __init__ (self,x,y):
        self.state = np.array([[x],
                                [y],
                                [0.0],
                                [0.0]])
        self.cov = np.array([[INIT_POS_STD * INIT_POS_STD ,    0,      0,      0],
                                [0,        INIT_POS_STD*INIT_POS_STD,      0,      0],
                                [0,        0,      INIT_VEL_STD *INIT_VEL_STD,     0],
                                [0,        0,      0,      INIT_VEL_STD*INIT_VEL_STD]])
        self.z = np.array([[0.0],
                            [0.0]])
        
    def prediction(self, dt):
        # state(predicted) = A⋅state
        # cov=A⋅cov⋅A^T + L⋅R⋅L^T 

        # 상태 전이 행렬
        A = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        
        # 스테이트 공분산 행렬
        # 1은 위치의 변동에 대한 불확실성
        # 속도 변수인 vel_x와 vel_y에 대한 불확실성
        R = np.array([[1,       0],
                      [0,       ACCEL_STD * ACCEL_STD]])

        # 제어 입력이 [x의 가속도,y의 가속도] 형태일 때 가정
        # 가속도의 효과를 모델링하는 행렬
        # x 위치에 대한 가속도 효과, y 위치에 대한 가속도 효과, x 속도에 대한 가속도 효과, y 속도에 대한 가속도 효과
        # x = A * x + 0.5 * a * t^2 y = A * y + 0.5 * a * t^2 vel_x = vel_ x + a * t vel_y = vel_ y + a * t 
        L  = np.array([[(0.5*dt*dt),0],
                       [0,(0.5*dt*dt)],
                       [dt,         0],
                       [0,          dt]])
        
        state = A.dot(self.state)
        cov = A @ self.cov @ A.T + L @ R @ L.T

        self.state = state
        self.cov = cov

    def correction(self):
        
        # 측정 행렬
        C = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0]])
        # 측정 노이즈
        Q = np.array([[OBS_STD*OBS_STD, 0],
                     [0, OBS_STD*OBS_STD]])
        
        z_hat = C.dot(self.state)
        measure_diff = self.z - z_hat
        M = C @ self.cov @ C.T  + Q
        K = self.cov @ C.T @ np.linalg.inv(M)
        state = self.state + K @ measure_diff
        cov = (np.eye(4) - K.dot(C)) @ self.cov

        self.state = state
        self.cov = cov

    def calvel(self):
        velx = self.state[2][0]
        vely = self.state[3][0]

        vel = math.sqrt((velx ** 2) + (vely ** 2))

        return vel
    
    def getx(self):
        return float(self.state[0][0])
    
    def gety(self):
        return float(self.state[1][0])
    
    def getr(self):
        return math.sqrt(math.pow(self.state[0][0], 2) + math.pow(self.state[1][0], 2))
    
    # def predict_path(self, t):

    #     x = self.state[0][0]
    #     y = self.state[1][0]
    #     vx = self.state[2][0]
    #     vy = self.state[3][0]

    #     dt = 0.1

    #     if self.calvel() < 0.05:
    #         vx = 0
    #         vy = 0
            
    #     path_list = []
        
    #     for step in range(t):
    #         ps = Point()
    #         ps.x = x + vx * dt * step
    #         ps.y = y + vy * dt * step
    #         path_list.append(ps)

    #     return path_list

    def predict_path(self, t, world_x, world_y):

        x = world_x
        y = world_y
        vx = self.state[2][0]
        vy = self.state[3][0]

        dt = 0.1

        if self.calvel() < 0.05:
            vx = 0
            vy = 0
            
        path_list = []
        
        for step in range(t):
            ps = Point()
            ps.x = x + vx * dt * step
            ps.y = y + vy * dt * step
            path_list.append(ps)

        return path_list

