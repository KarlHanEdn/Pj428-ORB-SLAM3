

import subprocess
import cv2
import numpy as np
import time
import struct_io


def send_request(server, im, tsec):
    struct_io.write_im(server, im)
    struct_io.write_float(server, tsec)
    server.stdin.flush()


def read_reply(server):
    pose_mat = struct_io.read_matrix4f(server)
    point_cloud = struct_io.read_vector3f_array(server)
    debug_im = struct_io.read_im(server)
    return point_cloud, pose_mat, debug_im


def main():
    cam = cv2.VideoCapture(0)

    slam_server = subprocess.Popen(['./orb_slam_server', 'ORBvoc.txt', 'EuRoC.yaml'], stdin=subprocess.PIPE, stdout=subprocess.PIPE)

    im_count = 0

    debug_im = None
    start_tsec = time.time()

    while True:
        ret, im = cam.read()
        if not ret:
            break
        elapsed_tsec = time.time() - start_tsec
        send_request(slam_server, im, elapsed_tsec)

        point_cloud, pose_mat, debug_im = read_reply(slam_server)

        im_count += 1
        if im_count % 10 == 0:
            print(pose_mat.T)
            cv2.imwrite(f'out_im/im_{im_count}.jpg', debug_im)



if __name__ == '__main__':
    main()
