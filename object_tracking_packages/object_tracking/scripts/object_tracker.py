#!/usr/bin/env python3

from ultralytics import YOLO
import numpy as np
import supervision as sv
from supervision import Detections
import cv2, rospy
from geometry_msgs.msg import Point
from object_tracking_msgs.msg import Trajectory, Path
import pyrealsense2 as rs
import time
import math
from vel_predict import VelKalman

class ObjectTracker:
    def __init__(self, model_path,publisher):
        # self.model = YOLO(model_path).cuda()
        self.tracker = sv.ByteTrack()
        self.box_annotator = sv.BoundingBoxAnnotator()
        self.frame_counters = {}
        self.trajectories = {}
        self.kalman_list = {}
        self.publisher = publisher
        
        # cpu 사용을 위한 코드
        if (rospy.get_param('/use_cpu', 'false')):
            self.device = 'cpu'
        else:
            self.device = 'cuda'

        self.model = YOLO(model_path)
        # YOLO 모델 내에서 Conv2d + BatchNorm2d 레이어를 융합
        # 모델을 최적화하여 추론 시간을 단축
        self.model.fuse()

    def update_tracker(self, frame, detections):
        # Convert YOLO detections to the format expected by ByteTrack
        formatted_detections = self.format_detections_for_byte_track(detections)

        # Update ByteTrack with formatted detections
        # update_with_detections : 현재 측정 정보를 받고 업데이트된 detection 결과 반환
        tracked_objects = self.tracker.update_with_detections(formatted_detections)
        # tracked_objects 형태
        # Detections(xyxy=array([[22.516, 7.1166, 648.69, 479.38]], dtype=float32), 
        # mask=None, confidence=array([0.85462], dtype=float32), class_id=array([0]), tracker_id=array([1]))
        # print(tracked_objects)

        formatted_tracked_objects = []
        current_tracked_id = []

        for obj in tracked_objects:
            bbox_array, _, confidence, class_id, obj_id = obj
            x1, y1, x2, y2 = bbox_array

            # Check each coordinate separately for finiteness
            if np.isfinite(x1) and np.isfinite(y1) and np.isfinite(x2) and np.isfinite(y2):
                # Draw bounding boxes and labels
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                label = f"ID: {obj_id} Conf: {confidence:.2f} Class: {class_id}"
                cv2.putText(frame, label, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                # Update frame counter for each tracked object
                # bytetrack 알고리즘에 의해 tracked된 object의 frame 수를 측정
                self.frame_counters[obj_id] = self.frame_counters.get(obj_id, 0) + 1
                # Add to formatted_tracked_objects only if frame_count exceeds threshold
                # frame 수가 특정 값 이상이면 궤적 생성을 위해 formatted_tracked_objects 리스트에 추가
                # 여기가 문제
                # add_to_trajectory 함수에서 for문이 돌 때 잘못된 depth 값의 대상과 bounding box의 대상이 다를 수 있음
                # else 문으로 formatted_tracked_objects.append([]) 빈 리스트 추가하고 add_to_trajectory 함수에서
                # formatted_tracked_objects의 값이 빈 리스트라면 continue하도록 만들 수는 있음
                if self.frame_counters[obj_id] > 5:
                    formatted_tracked_objects.append([x1, y1, x2, y2, obj_id])
                    current_tracked_id.append(obj_id)
                else:
                    formatted_tracked_objects.append([])
            
            # print(self.frame_counters)

        # 현재 track하고 있는 obj들의 예상경로를 하나의 topic으로 모아 publish
        trajectory_msg = Trajectory()
        trajectory_msg.paths = []

        for obj_id, trajectory in self.trajectories.items():
            if len(trajectory) > 0 and obj_id in current_tracked_id:
                # path 값 초기화
                path_msg = Path()
                # rviz 상에서의 좌표로 변환 시
                # x = z, y = -x z = -y   
                x = trajectory[-1][2]
                y = - trajectory[-1][0]
                if obj_id not in self.kalman_list:
                    self.kalman_list[obj_id] = VelKalman(x,y)

                if len(trajectory) > 1:
                    dt = trajectory[-1][3] - trajectory[-2][3]
                else:
                    dt = 1/50
                # 측정값이 너무 이상한 값은 제외하는 코드 필요할지 실험
                self.kalman_list[obj_id].z = np.array([[x],
                                                      [y]])
                
                self.kalman_list[obj_id].prediction(dt)
                self.kalman_list[obj_id].correction()
                last_point = trajectory[-1]
                # trajectory_msg.trajectory_points = Point(x=last_point[0], y=last_point[1], z=last_point[2])
                trajectory_msg.trajectory_points = Point(x=self.kalman_list[obj_id].getx(), y=self.kalman_list[obj_id].gety(), z=0.0)
                trajectory_msg.cur_tracked_id = current_tracked_id     
                  
                path_msg.path_list = self.kalman_list[obj_id].predict_path(20)
                print("{}의 초기 x 값: {}, y 값: {}".format(obj_id, path_msg.path_list[0].x, path_msg.path_list[0].y))

                vel = self.kalman_list[obj_id].calvel()
                print("{}의 추정 속도: {}".format(obj_id, round(vel,3)))
                trajectory_msg.paths.append(path_msg)


        print(len(trajectory_msg.paths))
        self.publisher.publish(trajectory_msg)

        return frame, formatted_tracked_objects

    def perform_detection(self, model, frame_path, classes):
        # Perform detection using the model
        # confidence 등 여러 파라미터 값 추가 가능
        results = model(frame_path, classes = classes, conf = 0.6, iou = 0.7)

        # Extract confidences, classes, and bounding boxes from the results
        confidences = results[0].boxes.conf.tolist()
        classes = results[0].boxes.cls.tolist()
        boxes = results[0].boxes.xyxy.tolist()

        # segmentation 픽셀 좌표 정보
        if results[0].masks == None:
            masks = []
        else:
            masks = results[0].masks.xy
            
        img = results[0].orig_img

        # print(boxes)  

        # Combine boxes with scores and classes
        # bounding box 좌표, confidence score, class를 2차원 array로 묶어서 표현
        boxes_with_scores_classes = [box + [score] + [cls] for box, score, cls in zip(boxes, confidences, classes)]
        
        return boxes_with_scores_classes, masks, img

    def add_to_trajectory(self, trajectories, tracked_objects, depths):
        # This function adds the new distance data to the trajectory for the given object ID
        for tracked_obj, (x,y,z) in zip(tracked_objects, depths):
            
            if len(tracked_obj) == 0:
                continue

            # Extract the object ID and convert it to an integer
            obj_id = int(tracked_obj[4])  
            
            x_meter = x
            y_meter = y
            depth = z
            # 속도 계산을 위한 변수
            cur_time = time.time()

            # If the object ID is not in the dictionary, initialize it with an empty list
            if obj_id not in trajectories:
                trajectories[obj_id] = []
            
            if len(trajectories[obj_id]) > 1:
                last_depth = trajectories[obj_id][-1][2]
                second_last_depth = trajectories[obj_id][-2][2]
                avg_last_depths = (last_depth + second_last_depth) / 2
                depth_change = depth - avg_last_depths
                # print(depth)

                if abs(depth_change) > 0.4:
                    # If spike detected, moderate the depth change
                    adjusted_depth = last_depth + depth_change / 200
                    trajectories[obj_id].append((x_meter,y_meter, adjusted_depth, cur_time))
                # elif abs(depth_change) > 1.0:
                #     trajectories[obj_id].append(trajectories[obj_id][-1])
                else:
                    # No spike, append the new depth value
                    trajectories[obj_id].append((x_meter,y_meter, depth, cur_time))
            else:
                if(depth>0):
                    # Less than two previous values, append the new depth value
                    trajectories[obj_id].append((x_meter,y_meter, depth, cur_time))
            # print(len(trajectories))
        return trajectories

    def convert_to_intrinsics(self,camera_info_msg):
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

    # def extract_distance(self,depth_image,bbox,camera_intrinsics):  
    #     depth_frame = np.asarray(depth_image, dtype=np.uint16)

    #     intrinsics = self.convert_to_intrinsics(camera_intrinsics)

    #     # Calculate the center of the bounding box
    #     x_center = int((bbox[0] + bbox[2])  // 2)
    #     y_center = int((bbox[1] + bbox[3]) // 2)
    #     # print(x_center, y_center)

    #     # Extract the depth value at the center of the bounding box
    #     depth_value = depth_frame[y_center, x_center]

    #     depth_value_meter = float(depth_value/1000)
    #     # print(depth_value_meter)
        
    #     # 이미지 픽셀 좌표를 3D 공간에서의 좌표로 변경(x, y 좌표, z 깊이)
    #     x,y,z = rs.rs2_deproject_pixel_to_point(intrinsics,[x_center,y_center],depth_value)
    #     # print(x,y,z)

    #     return x/1000,y/1000,z/1000

    def extract_distance(self,depth_image,mask,camera_intrinsics, copy_img):  
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

            cv2.circle(copy_img, (cX, cY), 3, (255, 0, 0), -1)
            cv2.drawContours(copy_img, [i], 0, (0, 0, 255), 2)

        cv2.imshow("mask", copy_img)

        # Extract the depth value at the center of the bounding box
        depth_value = depth_frame[cY, cX]

        depth_value_meter = float(depth_value/1000)
        # print(depth_value_meter)
        
        # 이미지 픽셀 좌표를 3D 공간에서의 좌표로 변경(x, y 좌표, z 깊이)
        x,y,z = rs.rs2_deproject_pixel_to_point(intrinsics,[cX,cY],depth_value)
        # print(x,y,z)
        print(x/1000,y/1000,z/1000)
        return x/1000,y/1000,z/1000


    def format_detections_for_byte_track(self, yolov_detections):
        # Extract the different attributes from YOLOv8 detections
        bboxes = [det[:4] for det in yolov_detections]         
        confidences = [det[4] for det in yolov_detections]     
        class_ids = [int(det[5]) for det in yolov_detections]  

        # Create an empty Detections instance
        byte_track_detections = Detections.empty()

        # Populate the Detections instance
        byte_track_detections.xyxy = np.array(bboxes, dtype=np.float32)
        byte_track_detections.confidence = np.array(confidences, dtype=np.float32)
        byte_track_detections.class_id = np.array(class_ids, dtype=int)

        return byte_track_detections
