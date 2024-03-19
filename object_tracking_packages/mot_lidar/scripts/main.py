#!/usr/bin/env python3

from object_tracker import ObjectTracker
from trajectory_visualizer_node import TrajectoryVisualizer
import cv2, os, time, rospy, psutil
from cv_bridge import CvBridge
from object_tracking_msgs.msg import Trajectory
from geometry_msgs.msg import Point
from nav_msgs.msg import Path, Odometry
import tf.transformations as tf
import numpy as np

# 라이더 값 사용
from sensor_msgs.msg import CompressedImage, LaserScan, Image
import sensor_msgs.point_cloud2 as pc2
from calib_laser import LiDARCameraCalib


class MainNode:
    def __init__(self):
        self.bridge = CvBridge()
        # rospy.Subscriber('/camera/color/image_raw/compressed',CompressedImage,self.rgb_callback)
        rospy.Subscriber('/camera/color/image_raw',Image,self.rgb_callback)
        rospy.Subscriber('/scan', LaserScan, self.Lscan_callback)
        # rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # 리얼센스 카메라가 아닌 웹캡으로 실험하기 위한 subscriber
        # rospy.Subscriber("/camera1/usb_cam1/image_raw", Image, self.rgb_callback)
        # rospy.Subscriber("/camera1/usb_cam1/camera_info", CameraInfo, self.camera_info_callback)
        self.trajectory_publisher = rospy.Publisher('trajectory_topic',Trajectory, queue_size =10)
        # self.path_publisher = rospy.Publisher('predicted_path', Path, queue_size=10)
        self.tracker = ObjectTracker('yolov8n.pt',self.trajectory_publisher)  
        self.calib = LiDARCameraCalib(self.pc_np)
        self.classes = [0, 1]
        self.frame = None
        self.track_id = 1
        self.pose = [0, 0, 0]

    def rgb_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.process_frame(frame)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        _,_, yaw = tf.euler_from_quaternion([msg.pose.pose.orientation.x , msg.pose.pose.orientation.y , msg.pose.pose.orientation.z , msg.pose.pose.orientation.w])
        self.pose = [0, 0, 0]

    def Lscan_callback(self, msg):
        point_list = []        
        for index, distance in enumerate(msg.ranges):
            x = distance * np.cos(msg.angle_min + (msg.angle_increment * index))
            y = distance * np.sin(msg.angle_min + (msg.angle_increment * index))
            # point_list.append((x, y, parameters_lidar["Z"], 1))
            point_list.append((x, y, 0, 1))
        # for point in pc2.read_points(msg, skip_nans=True):      
        #     point_list.append((point[0], point[1], point[2], 1))
        self.pc_np = np.array(point_list, np.float32)


    def process_frame(self, frame):

        self.frame = frame

        # Object detection and tracking
        # object detection 결과 bounding box와 confidence score, class를 합친 2차원 array(boxes_with_scores_classes),
        # bounding box값(boxes)를 저장
        # boxes_with_scores_classes, boxes = self.tracker.perform_detection(self.tracker.model, frame, self.classes)
        boxes_with_scores_classes, boxes = self.tracker.perform_detection(self.tracker.model, frame, self.classes)
        # Process depths
        # detection된 객체들의 3D 좌표를 저장 
        # depths = [self.tracker.extract_distance(depth_image, box[:4], self.camera_intrinsics) for box in boxes]
        # print(depths)

        lidaraxis = [self.calib.targetaxis(box[:4], self.pose, frame) for box in boxes]

        # 검출된 객체가 있을 경우
        if len(boxes_with_scores_classes) > 0:

            # 검출된 결과를 바탕으로 bytetrack으로 검출 결과를 update하고 입력 이미지에 annotation 결과 표시(frame_with_detections), 
            # tracking 할 정보가 담긴 값 반환(tracked_objects) [x1, y1, x2, y2, obj_id] 형태
            frame_with_detections, tracked_objects = self.tracker.update_tracker(frame, boxes_with_scores_classes, self.pose)

            # Distance extraction and trajectory updating
            self.tracker.trajectories = self.tracker.add_to_trajectory(self.tracker.trajectories, tracked_objects, lidaraxis)
            self.frame = frame_with_detections
        
            # path_pred_msg = Path()
            # path_pred_msg.header.stamp = rospy.Time.now()
            # path_pred_msg.header.frame_id = "camera_color_frame"
            # path_pred_msg.poses = path_list

            # self.path_publisher.publish(path_pred_msg)

        # Display the frame with trajectories
        img = cv2.resize(self.frame, (480, 360))
        cv2.imshow('Trajectories', img)
        cv2.waitKey(1)
            
def main(args=None):
    rospy.init_node('main_node', anonymous = True)
    main_node = MainNode()
    rospy.spin()
    cv2.destroyAllWindows()
    rospy.signal_shutdown("KeyboardInterrupt")

if __name__ == "__main__":
    main()


