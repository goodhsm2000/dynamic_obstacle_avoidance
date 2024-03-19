#!/usr/bin/env python3

import cv2, os, time, rospy, psutil
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from yolov8_ros_msgs.msg import BoundingBox, BoundingBoxes
from ultralytics import YOLO
import pyrealsense2 as rs
import numpy as np
from std_msgs.msg import Header

class MainNode:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.Subscriber('/camera/color/image_raw',Image,self.image_callback)
        rospy.Subscriber('/camera/depth/image_rect_raw',Image,self.depth_callback)
        rospy.Subscriber( '/camera/color/camera_info', CameraInfo,self.camera_info_callback)

        # 리얼센스 카메라가 아닌 웹캡으로 실험하기 위한 subscriber
        # rospy.Subscriber("/camera1/usb_cam1/image_raw", Image, self.rgb_callback)
        # rospy.Subscriber("/camera1/usb_cam1/camera_info", CameraInfo, self.camera_info_callback)
        self.detect_publisher = rospy.Publisher('yolov8/detect_with_dis', BoundingBoxes, queue_size=10)
        self.depth_image = None
        self.classes = [0, 1]
        self.camera_intrinsics = None
        self.frame = None
        self.getImageStatus = False
        self.visualize = True
                
        # cpu 사용을 위한 코드
        if (rospy.get_param('/use_cpu', 'false')):
            self.device = 'cpu'
        else:
            self.device = 'cuda'

        self.model = YOLO('yolov8n-seg.pt')
        # YOLO 모델 내에서 Conv2d + BatchNorm2d 레이어를 융합
        # 모델을 최적화하여 추론 시간을 단축
        self.model.fuse() 

    def image_callback(self, image):

        self.boundingBoxes = BoundingBoxes()
        self.boundingBoxes.header = image.header
        self.boundingBoxes.image_header = image.header
        self.getImageStatus = True
        self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)

        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)

        results = self.model(self.color_image, show=False, conf=0.7, iou=0.6)

        self.dectshow(results, image.height, image.width)

        cv2.waitKey(3)

    def dectshow(self, results, height, width):

        self.frame = results[0].plot()
        # print(str(results[0].speed['inference']))
        fps = 1000.0/ results[0].speed['inference']
        cv2.putText(self.frame, f'FPS: {int(fps)}', (20,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

        # mask 정보 사용
        masks = results[0].masks.xy

        # for result in results[0].boxes:
        #     distance = self.extract_distance(self.depth_image, result, self.camera_intrinsics)

        for i, result in enumerate(results[0].boxes):
            distance = self.extract_distance(self.depth_image, masks[i], self.camera_intrinsics)
            
            boundingBox = BoundingBox()
            boundingBox.xmin = np.int64(result.xyxy[0][0].item())
            boundingBox.ymin = np.int64(result.xyxy[0][1].item())
            boundingBox.xmax = np.int64(result.xyxy[0][2].item())
            boundingBox.ymax = np.int64(result.xyxy[0][3].item())
            boundingBox.Class = results[0].names[result.cls.item()]
            boundingBox.probability = result.conf.item()
            boundingBox.distance = distance
            self.boundingBoxes.bounding_boxes.append(boundingBox)

        self.detect_publisher.publish(self.boundingBoxes)
        # self.publish_image(self.frame, height, width)

        if self.visualize :
            cv2.imshow('YOLOv8', self.frame)

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def camera_info_callback(self, camera_info_msg):
        self.camera_intrinsics = camera_info_msg

    def convert_to_intrinsics(self, camera_info_msg):
        # Create a new intrinsics object
        intrinsics = rs.intrinsics()

        # Set the width and height from the CameraInfo message
        intrinsics.width = camera_info_msg.width
        intrinsics.height = camera_info_msg.height

        # Set the focal lengths (fx, fy)
        intrinsics.fx = camera_info_msg.K[0]  
        intrinsics.fy = camera_info_msg.K[4]

        # Set the principal point (cx, cy)
        intrinsics.ppx = camera_info_msg.K[2]  
        intrinsics.ppy = camera_info_msg.K[5]  

        # Set the distortion model
        intrinsics.model = rs.distortion.brown_conrady

        # Set the distortion coefficients
        intrinsics.coeffs = list(camera_info_msg.D)[:5]  

        return intrinsics
    
    # def extract_distance(self, depth_image, result, camera_intrinsics):  
    #     depth_frame = np.asarray(depth_image, dtype=np.uint16)

    #     intrinsics = self.convert_to_intrinsics(camera_intrinsics)

    #     xmin = np.int64(result.xyxy[0][0].item())
    #     ymin = np.int64(result.xyxy[0][1].item())
    #     xmax = np.int64(result.xyxy[0][2].item())
    #     ymax = np.int64(result.xyxy[0][3].item())

    #     # Calculate the center of the bounding box
    #     x_center = int((xmin + xmax)  // 2)
    #     y_center = int((ymin + ymax) // 2)
    #     print("depth 목표 좌표: ", x_center, y_center)

    #     # Extract the depth value at the center of the bounding box
    #     depth_value = depth_frame[y_center, x_center]

    #     depth_value_meter = float(depth_value/1000)
    #     # print(depth_value_meter)
        
    #     # 이미지 픽셀 좌표를 3D 공간에서의 좌표로 변경(x, y 좌표, z 깊이)
    #     x,y,z = rs.rs2_deproject_pixel_to_point(intrinsics,[x_center,y_center],depth_value)
    #     # print(x,y,z)
    #     print(depth_value)
    #     print(depth_value_meter)
    #     return depth_value_meter
    
    def extract_distance(self, depth_image, mask, camera_intrinsics):  
        depth_frame = np.asarray(depth_image, dtype=np.uint16)

        intrinsics = self.convert_to_intrinsics(camera_intrinsics)

        # mask 좌표의 중간 좌표를 3D 좌표로 변환
        # target_xy = mask[len(mask)//2]
        # x_center = int(target_xy[0])
        # y_center = int(target_xy[1])
        # print(x_center, y_center)

        # mask 좌표들의 무게 중심 좌표를 3D 좌표로 변환
        zero_mask = np.zeros(depth_frame.shape[0:2]) # mask 정보는 2차원 선호
        masked_polygon = cv2.fillPoly(zero_mask, [mask.astype(np.int32)], 255)
        contours, hierarchy = cv2.findContours(masked_polygon.astype(np.uint8), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

        # dst = results[0].orig_img

        for i in contours:
            M = cv2.moments(i)
            cX = int(M['m10'] / M['m00'])
            cY = int(M['m01'] / M['m00'])

            cv2.circle(self.frame, (cX, cY), 3, (255, 0, 0), -1)
            cv2.drawContours(self.frame, [i], 0, (0, 0, 255), 2)

        # cv2.imshow("mask", copy_img)

        # Extract the depth value at the center of the bounding box
        depth_value = depth_frame[cY, cX]

        depth_value_meter = float(depth_value/1000)
        # print(depth_value_meter)
        
        # 이미지 픽셀 좌표를 3D 공간에서의 좌표로 변경(x, y 좌표, z 깊이)
        x,y,z = rs.rs2_deproject_pixel_to_point(intrinsics,[cX,cY],depth_value)
        # print(x,y,z)
        # print(x/1000,y/1000,z/1000)
        return depth_value_meter          
            
def main():
    rospy.init_node('yolov8_ros', anonymous=True)
    yolo_dect = MainNode()
    rospy.spin()

if __name__ == "__main__":
    main()
