#!/usr/bin/env python3
import numpy as np
import os
import math
import cv2

import rospy
import subprocess
import sys
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, PointCloud, ChannelFloat32, Imu
from std_msgs.msg import String
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge, CvBridgeError
import rospkg
import tf2_ros
import tf
import geometry_msgs

import struct_io
import threading
import json


HOST_NAME = os.environ["VEHICLE_NAME"]
USE_IMU = False


def send_request(server, im, tsec, imu_point_arr):
    struct_io.write_im(server, im)
    struct_io.write_double(server, tsec)
    if USE_IMU:
        struct_io.write_imu_point_array(server, imu_point_arr)
    server.stdin.flush()


def read_reply(server):
    pose_mat = struct_io.read_matrix4f(server)
    point_cloud = struct_io.read_vector3f_array(server)
    debug_im = struct_io.read_im(server)
    return point_cloud, pose_mat, debug_im


class SLAMNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(SLAMNode, self).__init__(node_name=node_name,node_type=NodeType.GENERIC)
        self.veh = rospy.get_namespace().strip("/")

        rospack = rospkg.RosPack()
        src_path = rospack.get_path('slam_node') + '/src'
        popen_env = {'LD_LIBRARY_PATH': src_path + '/../../../assets/lib'}

        if USE_IMU:
            self.slam_server = subprocess.Popen(
                [
                    src_path + '/orb_slam_server_monoi', 
                    src_path + '/ORBvoc.txt', 
                    src_path + '/EuRoC_monoi.yaml',
                ], 
                stdin=subprocess.PIPE, 
                stdout=subprocess.PIPE,
                env=popen_env)
        else:
            self.slam_server = subprocess.Popen(
                [
                    src_path + '/orb_slam_server', 
                    src_path + '/ORBvoc.txt', 
                    src_path + '/EuRoC.yaml',
                ], 
                stdin=subprocess.PIPE, 
                stdout=subprocess.PIPE,
                env=popen_env)

        # imu
        self.imu_arr = []
        self.imu_lock = threading.Lock()

        if USE_IMU:
            self.imu_sub = rospy.Subscriber(f'/{HOST_NAME}/imu_node/imu_data', Imu, self.imu_callback)
        self.camera_sub = rospy.Subscriber(f'/{HOST_NAME}/camera_node/image/compressed', CompressedImage, self.callback)
        self.im_pub = rospy.Publisher(f'/{HOST_NAME}/slam_node/image/compressed', CompressedImage, queue_size=10)
        self.point_cloud_pub = rospy.Publisher(f'/{HOST_NAME}/slam_node/point_cloud', PointCloud, queue_size=1)

        self.json_pub = rospy.Publisher(f'/{HOST_NAME}/slam_node/json', String, queue_size=5)

        self.im_seq = 0
        self.im_count = 0
    
    def callback(self, msg):
        # how to decode compressed image
        # reference: http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
        self.im_count += 1
        if self.im_count % 1 == 0:
            timestamp = rospy.Time.now().to_sec()
            compressed_image = np.frombuffer(msg.data, np.uint8)
            im = cv2.imdecode(compressed_image, cv2.IMREAD_COLOR)

            # retrieve all the measurements so far
            self.imu_lock.acquire()
            imu_point_arr = self.imu_arr
            self.imu_arr = []
            self.imu_lock.release()
            if len(imu_point_arr) == 0 and USE_IMU:
                print('NO IMU POINT ARR RECEIVED YET')
            send_request(self.slam_server, im, timestamp, imu_point_arr)

    def run(self):
        while True:
            point_cloud, pose_mat, debug_im = read_reply(self.slam_server)
            self.publish_im(debug_im)

            # publish pose
            rot_mat = np.eye(4)
            rot_mat[0:3, 0:3] = pose_mat[0:3, 0:3].T
            pose_t = pose_mat[3, 0:3]
            quaternion = tf.transformations.quaternion_from_matrix(rot_mat)

            br = tf2_ros.TransformBroadcaster()
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = 'world'
            t.child_frame_id = f'{HOST_NAME}/slam_odometry'
            t.transform.translation = geometry_msgs.msg.Vector3(pose_t[0].item(), pose_t[1].item(), pose_t[2].item())
            t.transform.rotation = geometry_msgs.msg.Quaternion(quaternion[0].item(), quaternion[1].item(), quaternion[2].item(), quaternion[3].item())
            br.sendTransform(t)

            # publish map points
            pcmsg = PointCloud()
            pcmsg.header.frame_id = 'world'
            pcmsg.header.stamp = rospy.Time.now()
            for i in range(point_cloud.shape[0]):
                pt = point_cloud[i]
                point32 = Point32(pt[0].item(), pt[1].item(), pt[2].item())
                pcmsg.points.append(point32)
            self.point_cloud_pub.publish(pcmsg)

            json_dict = {
                "translation": pose_t.tolist(),
                "rotation": quaternion.tolist(),
                "point_cloud": point_cloud.tolist(),
            }
            self.json_pub.publish(String(json.dumps(json_dict)))

    def imu_callback(self, msg):
        acc = np.array((msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z), dtype=np.float32)
        gyro = np.array((msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z), dtype=np.float32)
        timestamp = msg.header.stamp
        self.imu_lock.acquire()
        self.imu_arr.append((acc, gyro, timestamp))
        self.imu_lock.release()
    
    def publish_im(self, im):
        msg = CompressedImage()
        msg.header.seq = self.im_seq
        msg.header.stamp = rospy.Time.now()
        msg.format = 'jpeg'
        ret, buffer = cv2.imencode('.jpg', im)
        if not ret:
            print('failed to encode image!')
        else:
            msg.data = np.array(buffer).tostring()
            self.im_pub.publish(msg)
            self.im_seq += 1


    def onShutdown(self):
        super(SLAMNode, self).onShutdown()


if __name__ == '__main__':
    print('SLAM node launched')
    ar_node = SLAMNode(node_name='slam_node')
    ar_node.run()
    rospy.spin()