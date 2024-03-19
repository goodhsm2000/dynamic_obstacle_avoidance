#!/usr/bin/env python3
import rospy,rospkg
import cv2
import numpy as np
import math, time
from sensor_msgs.msg import CompressedImage,LaserScan
import sensor_msgs.point_cloud2 as pc2
from numpy.linalg import inv
from ultralytics import YOLO
import torch
import sys, os

class LiDARCameraCalib:
    def __init__(self, pc_np): 

        self.params_cam = {
        "WIDTH": 1280, # image width
        "HEIGHT": 720, # image height
        "FOV": 86, # Field of view
        "X": 0.0, # meter
        "Y": -0.0,
        "Z":  0.1,
        "YAW": 0, # radian
        "PITCH": 0.0,
        "ROLL": 0
        }

        self.params_lidar = {
            "X": 0.1, 
            "Y": 0.0,
            "Z": 0.0,
            "YAW": 0,
            "PITCH": 0,
            "ROLL": 0
        }

        self.pc_np = pc_np
        self.img = None
        self.width = self.params_cam["WIDTH"]
        self.height = self.params_cam["HEIGHT"]
        self.TransformMat = self.getTransformMat(self.params_cam, self.params_lidar)
        self.CameraMat = self.getCameraMat(self.params_cam)

    def img_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def transformLiDARToCamera(self, pc_lidar):
        ''' lidar coordi to cam coordi '''
        cam_temp = self.TransformMat.dot(pc_lidar)
        cam_temp = np.delete(cam_temp, 3, axis=0)
        return cam_temp
    
    def transformCameraToImage(self, pc_camera):
        
        cam_temp = self.CameraMat.dot(pc_camera)
        
        cam_temp = np.delete(cam_temp,np.where(cam_temp[2,:]<0),axis=1)
        ori_mat = cam_temp.copy()
        cam_temp /= cam_temp[2,:] # normalize

        # cam_temp = np.delete(cam_temp,np.where(cam_temp[0,:]>self.width),axis=1)
        # cam_temp = np.delete(cam_temp,np.where(cam_temp[0,:]<0),axis=1)
        # cam_temp = np.delete(cam_temp,np.where(cam_temp[1,:]>self.height),axis=1)
        # cam_temp = np.delete(cam_temp,np.where(cam_temp[1,:]<0),axis=1)

        return cam_temp , ori_mat
    

    def getRotMat(self,RPY):
        '''
        return (Yaw @ Pitch @ Roll) -> Rotation Mat
        '''
        cosR = math.cos(RPY[0])
        cosP = math.cos(RPY[1])
        cosY = math.cos(RPY[2])

        sinR = math.sin(RPY[0])
        sinP = math.sin(RPY[1])
        sinY = math.sin(RPY[2])
        
        rotRoll = np.array([1,0,0,
                            0,cosR,-sinR,
                            0,sinR,cosR]).reshape(3,3)
        rotPitch = np.array([cosP,0,sinP,
                            0,1,0,
                            -sinP,0,cosP]).reshape(3,3)
        rotYaw = np.array([ cosY,-sinY,0,
                            sinY,cosY, 0,
                            0,     0,  1]).reshape(3,3)
        
        rotMat = rotYaw.dot(rotPitch.dot(rotRoll))
        
        return rotMat


    def getTransformMat(self,params_cam, params_lidar):
        #With Respect to Vehicle ISO Coordinate
        lidarPosition = np.array([params_lidar.get(i) for i in (["X","Y","Z"])])
        #* 라이다 x,y,z 3차원 벡터
        camPosition = np.array([params_cam.get(i) for i in (["X","Y","Z"])])
        #* 카메라 x,y,z 3차원 벡터
        
        lidarRPY = np.array([params_lidar.get(i) for i in (["ROLL","PITCH","YAW"])])
        #* 라이다 roll ,pith, yaw 3차원 벡터
        camRPY = np.array([params_cam.get(i) for i in (["ROLL","PITCH","YAW"])])
        #* 카메라 roll , pitch , yaw 3차원 벡터

        camRPY = camRPY + np.array([-90 * math.pi/180, 0 ,-90 * math.pi/180])

        #lidarPositionOffset = np.array([0.02883,0,0.09081])
        lidarPositionOffset = np.array([0.0,
                                        0.0,
                                        0.01])
        lidarPosition = lidarPosition + lidarPositionOffset
        camPositionOffset = np.array([0.0,
                                    0.0,
                                    0])

        camPosition = camPosition + camPositionOffset

        camRot = self.getRotMat(camRPY)
        camTransl = np.array([camPosition])

        # 3x3 회전 행렬과 1x3 변환 벡터를 합쳐 4x4 행렬로 만들어 줌
        Tr_cam_to_vehicle = np.concatenate((camRot,camTransl.T),axis = 1)
        Tr_cam_to_vehicle = np.insert(Tr_cam_to_vehicle, 3, values=[0,0,0,1],axis = 0)
        # 4 x 4  Matrix
        
        lidarRot = self.getRotMat(lidarRPY)
        lidarTransl = np.array([lidarPosition]) 
        Tr_lidar_to_vehicle = np.concatenate((lidarRot,lidarTransl.T),axis = 1)
        Tr_lidar_to_vehicle = np.insert(Tr_lidar_to_vehicle, 3, values=[0,0,0,1],axis = 0)
        # 4 x 4  Matrix

        invTr = inv(Tr_cam_to_vehicle)
        Tr_lidar_to_cam = invTr.dot(Tr_lidar_to_vehicle).round(6)

        return Tr_lidar_to_cam



    def getCameraMat(self, params_cam):
        '''#* Camera Intrinsic Parameters'''
        focalLength = params_cam["WIDTH"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
        principalX = params_cam["WIDTH"]/2
        principalY = params_cam["HEIGHT"]/2
        skew_c = 0.
        # CameraMat = np.array([910.8676147460938, 0.0, 637.0319213867188, 0.0, 909.7587890625, 358.15234375, 0.0, 0.0, 1.0]).reshape(3,3)
        CameraMat = np.array([910.8676147460938, 0.0, 637.0319213867188, 0.0, 909.7587890625, 358.15234375, 0.0, 0.0, 1.0]).reshape(3,3)

        return CameraMat
    
    def targetaxis(self, box, pose, frame):
        # 라이더 값
        xyz_p = self.pc_np[:, 0:3]
        xyz_p = np.insert(xyz_p,3,1,axis=1).T
        xyz_p = np.delete(xyz_p,np.where(xyz_p[0,:]<0),axis=1)
        xyz_p = np.delete(xyz_p,np.where(xyz_p[0,:]>10),axis=1)
        # xyz_p = np.delete(xyz_p,np.where(xyz_p[2,:]<-1.2),axis=1) #Ground Filter
        xyz_c = self.transformLiDARToCamera(xyz_p)
        xy_i ,ori_mat = self.transformCameraToImage(xyz_c)
        xy_i = xy_i.astype(np.int32)

        x1 = box[0]
        y1 = box[1]
        x2 = box[2]
        y2 = box[3]

        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2 

        x = int(cx) ; y  = int(cy) ; z = 1
        # xy_i의 2번째 행(y값) 삭제
        xy_i_temp = np.delete(xy_i, 1, axis = 0)
        temp = np.array([[x],[z]])
        sub_mat = xy_i_temp - temp

        # 바운딩 박스 중심점과 라이더의 이미지 상으로 변환된 점들 사이의 거리를 계산하여 가장 가까운 점의 인덱스를 찾음 
        target_idx = np.argmin(np.linalg.norm(sub_mat,axis = 0))
        scale = ori_mat[2,target_idx]
        temp_t = np.array([[x],[y],[z]])
        xyz_img = scale * temp_t
        xyz_cam = np.linalg.inv(self.CameraMat) @ xyz_img
        xyz_cam = np.vstack((xyz_cam,1))
        lidar_xyz = np.linalg.inv(self.TransformMat) @ xyz_cam
        x,y,_,_ = lidar_xyz.flatten()
        wx,wy = self.Tr_obstacle_to_map(pose)

        cv2.putText(frame,f"distance : {np.linalg.norm(lidar_xyz.flatten()).round(2)}",(int(x1) ,int(cy)),cv2.FONT_HERSHEY_SIMPLEX, 0.75,(255,0,0),2)
        cv2.circle(frame, (int(wx), int(wy)), 10, (0, 0, 255), 3)

        return wx, wy
    

    def Tr_obstacle_to_map(self,pose):

        x = pose[0]
        y = pose[1]
        yaw = pose[2]

        Tr_Mat_to_map = np.array([[np.cos(yaw), -np.sin(yaw), 0 , x],
                                  [np.sin(yaw),  np.cos(yaw), 0 , y],
                                  [          0,            0, 1 , 0],
                                  [          0,            0, 0,  1]])
        obs_vec = np.array([x,y,0,1]).reshape(4,1)

        wx,wy,_,_ = np.dot(Tr_Mat_to_map , obs_vec).flatten()
        
        return (wx,wy)
    
def draw_pts_img(img, xi, yi):
    point_np = img    
    for ctr in zip(xi, yi):
        point_np = cv2.circle(point_np, ctr, 2, (0,255,0),-1)
    return point_np

def draw_line(image, x1, y1, x2, y2, index):
    w = 10
    h = 10
    color = (200, 0, 0)
    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 200, 0), 2)
    # Top left corner
    cv2.line(image, (x1, y1), (x1 + w, y1), color, 3)
    cv2.line(image, (x1, y1), (x1, y1 + h), color, 3)

    # Top right corner
    cv2.line(image, (x2, y1), (x2 - w, y1), color, 3)
    cv2.line(image, (x2, y1), (x2, y1 + h), color, 3)

    # Bottom right corner
    cv2.line(image, (x2, y2), (x2 - w, y2), color, 3)
    cv2.line(image, (x2, y2), (x2, y2 - h), color, 3)

    # Bottom left corner
    cv2.line(image, (x1, y2), (x1 + w, y2), color, 3)
    cv2.line(image, (x1, y2), (x1, y2 - h), color, 3)

    text = f'ID:{str(index)}'
    cv2.putText(image, text,
                (x1, y1 - 2),
                0, 1 / 2, (0, 255, 0),
                thickness=1, lineType=cv2.FILLED)