#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from object_tracking_msgs.msg import Trajectory, Path
import time, random

def get_color_for_id(object_id):    
    random.seed(object_id)
    return{
        'r': random.random(),
        'g': random.random(),
        'b': random.random(),
        'a': 1.0
    }

class TrajectoryVisualizer:
    def __init__(self):
        rospy.init_node('trajectory_visualizer', anonymous = True)
        self.pb = rospy.Publisher('visualization_marker_array', MarkerArray,queue_size=1)
        self.sb = rospy.Subscriber(
            'trajectory_topic',
            Trajectory,
            self.trajectory_callback)
        self.trajectories = {}

    """
    def trajectory_callback(self, msg):
        current_time = time.time()
        max_trajectory_age = 3

        object_id = msg.object_id
        if object_id not in self.trajectories:
            self.trajectories[object_id] = []

        self.trajectories[object_id].extend([(point, current_time) for point in msg.trajectory_points])

        for obj_id in list(self.trajectories.keys()):
            self.trajectories[obj_id] = [(pt, ts) for pt, ts in self.trajectories[obj_id] if current_time - ts < max_trajectory_age]
            if not self.trajectories[obj_id]:
                del self.trajectories[obj_id]

        marker_array = MarkerArray()
        marker_id = 0

        # Create markers for each point in the trajectory
        
        for obj_id, trajectory_data in self.trajectories.items():
            color = get_color_for_id(obj_id)  # Get color based on object ID
            for point, _ in trajectory_data:
                marker = Marker()
                marker.header.frame_id = "test_frame"  # Replace with your frame_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                #marker.lifetime = rospy.duration.Duration(seconds=5)  # Marker lifetime
                marker.pose.position.x = point.z
                marker.pose.position.y = point.x
                marker.pose.position.z = point.y
                marker.scale.x = 0.1  # Adjust size as needed
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.r = color['r']
                marker.color.g = color['g']
                marker.color.b = color['b']
                marker.color.a = color['a']
                marker.id = marker_id
                marker_array.markers.append(marker)
                marker.text = str(obj_id)
                marker_id += 1  # Increment marker ID for the next marker

        self.publisher_.publish(marker_array)
        """
    def trajectory_callback(self, msg):
        current_time = time.time()
        object_id = msg.object_id
        cur_tracked_id = msg.cur_tracked_id
        path_list = msg.paths
        # trajectory 토픽 - 객체 하나에 대한 값 publish
        if msg.trajectory_points:
            # self.trajectories[object_id] = msg.trajectory_points[-1]
            self.trajectories[object_id] = msg.trajectory_points
        else:
            return

        marker_array = MarkerArray()
        marker_id = 0

        # Create markers for the last point in the trajectory
        for obj_id, point in self.trajectories.items():
            if obj_id in cur_tracked_id:
                color = get_color_for_id(obj_id)  

                marker = Marker()
                # marker.header.frame_id = "test_frame"  
                marker.header.frame_id = "camera_color_frame"
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                # marker.pose.position.x = -1*(point.z)
                # marker.pose.position.y = point.x
                # marker.pose.position.z = point.y
                marker.pose.position.x = point.x
                marker.pose.position.y = point.y
                marker.pose.position.z = point.z
                marker.points = path_list
                marker.scale.x = 0.01  
                marker.scale.y = 0.01
                marker.scale.z = 0.01
                marker.color.r = color['r']
                marker.color.g = color['g']
                marker.color.b = color['b']
                marker.color.a = color['a']
                marker.id = marker_id
                marker_array.markers.append(marker)
                marker.text = str(obj_id)
                marker_id += 1

        self.pb.publish(marker_array)
        # self.trajectories = {}
        
def main(args=None):
    visualizer = TrajectoryVisualizer()
    rospy.spin()
    rospy.signal_shutdown("KeyboardInterrupt")


if __name__ == '__main__':
    main()