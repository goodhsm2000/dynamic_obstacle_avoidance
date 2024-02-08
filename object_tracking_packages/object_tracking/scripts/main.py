#!/usr/bin/env python3

from object_tracker import ObjectTracker
from trajectory_visualizer_node import TrajectoryVisualizer
import cv2, os, time, rospy, psutil
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from object_tracking_msgs.msg import Trajectory
from geometry_msgs.msg import Point
from nav_msgs.msg import Path, Odometry
import tf.transformations as tf

class MainNode:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.Subscriber('/camera/color/image_raw',Image,self.rgb_callback)
        rospy.Subscriber('/camera/depth/image_rect_raw',Image,self.depth_callback)
        rospy.Subscriber( '/camera/color/camera_info', CameraInfo,self.camera_info_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # 리얼센스 카메라가 아닌 웹캡으로 실험하기 위한 subscriber
        # rospy.Subscriber("/camera1/usb_cam1/image_raw", Image, self.rgb_callback)
        # rospy.Subscriber("/camera1/usb_cam1/camera_info", CameraInfo, self.camera_info_callback)
        self.trajectory_publisher = rospy.Publisher('trajectory_topic',Trajectory,queue_size =10)
        self.path_publisher = rospy.Publisher('predicted_path', Path, queue_size=10)
        self.tracker = ObjectTracker('yolov8n-seg.pt',self.trajectory_publisher)  
        self.last_depth_image = None
        self.classes = [0, 1]
        self.camera_intrinsics = None
        self.frame = None
        self.track_id = 1
        self.pose = None

    def rgb_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.last_depth_image is not None:
            self.process_frame(frame, self.last_depth_image)

    def depth_callback(self, msg):
        self.last_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def camera_info_callback(self, camera_info_msg):
        self.camera_intrinsics = camera_info_msg

    def odom_callback(self, msg):
        self.pose = msg.pose

    def process_frame(self, frame, depth_image):

        self.frame = frame

        # Object detection and tracking
        # object detection 결과 bounding box와 confidence score, class를 합친 2차원 array(boxes_with_scores_classes),
        # bounding box값(boxes)를 저장
        # boxes_with_scores_classes, boxes = self.tracker.perform_detection(self.tracker.model, frame, self.classes)
        boxes_with_scores_classes, masks, copy_img = self.tracker.perform_detection(self.tracker.model, frame, self.classes)
        # Process depths
        # detection된 객체들의 3D 좌표를 저장 
        # depths = [self.tracker.extract_distance(depth_image, box[:4], self.camera_intrinsics) for box in boxes]
        # print(depths)

        depths = [self.tracker.extract_distance(depth_image, mask, self.camera_intrinsics, copy_img) for mask in masks]   

        # 검출된 객체가 있을 경우
        if len(boxes_with_scores_classes) > 0:

            # 검출된 결과를 바탕으로 bytetrack으로 검출 결과를 update하고 입력 이미지에 annotation 결과 표시(frame_with_detections), 
            # tracking 할 정보가 담긴 값 반환(tracked_objects) [x1, y1, x2, y2, obj_id] 형태
            frame_with_detections, tracked_objects = self.tracker.update_tracker(frame, boxes_with_scores_classes, self.pose)

            # Distance extraction and trajectory updating
            self.tracker.trajectories = self.tracker.add_to_trajectory(self.tracker.trajectories, tracked_objects, depths)
            self.frame = frame_with_detections
        
            # path_pred_msg = Path()
            # path_pred_msg.header.stamp = rospy.Time.now()
            # path_pred_msg.header.frame_id = "camera_color_frame"
            # path_pred_msg.poses = path_list

            # self.path_publisher.publish(path_pred_msg)

        # Display the frame with trajectories
        cv2.imshow('Trajectories', self.frame)
        cv2.waitKey(1)
            
def main(args=None):
    rospy.init_node('main_node', anonymous = True)
    main_node = MainNode()
    rospy.spin()
    cv2.destroyAllWindows()
    rospy.signal_shutdown("KeyboardInterrupt")

if __name__ == "__main__":
    main()