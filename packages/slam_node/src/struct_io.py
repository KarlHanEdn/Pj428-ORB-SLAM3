import cv2
import numpy as np
import struct


# note: flush after write


def write_im(server, im):
    im_data = cv2.imencode('.jpg', im)[1].tobytes()
    im_size = len(im_data)
    server.stdin.write(struct.pack('<I', im_size))
    server.stdin.write(im_data)


def write_uint32(server, integer):
    server.stdin.write(struct.pack('<I', integer))


def write_float(server, fv):
    server.stdin.write(struct.pack('<f', fv))


def write_double(server, dv):
    server.stdin.write(struct.pack('<d', dv))


def write_imu_point_array(server, imu_arr):
    """
    imu_arr is an array [(acc0, gyro0, timestamp0), (acc1, ...), ...] with numpy float32 arrays acci and gyroi, and a double typed timestampi
    """
    write_uint32(server, len(imu_arr))
    for imu_point in imu_arr:
        acc, gyro, timestamp = imu_point
        server.stdin.write(np.getbuffer(acc))
        server.stdin.write(np.getbuffer(gyro))
        write_double(server, timestamp)


def read_im(server):
    len_data = server.stdout.read(4)
    len_im = struct.unpack('<I', len_data)[0]
    im_data = server.stdout.read(len_im)
    im = cv2.imdecode(np.frombuffer(im_data, np.uint8), cv2.IMREAD_COLOR)
    return im


def read_uint32(server):
    data = server.stdout.read(4)
    value = struct.unpack('<I', data)[0]
    return value


def read_float(server):
    data = server.stdout.read(4)
    value = struct.unpack('<f', data)[0]
    return value


def read_vector3f_array(server):
    len_data = server.stdout.read(4)
    if not len_data:
        print(f'ERROR: received invalid data {len_data} when reading')
    len_arr = struct.unpack('<I', len_data)[0]
    len_arr_bytes = len_arr * 3 * 4  # a vector has 3 elements each 4 bytes
    arr_data = server.stdout.read(len_arr_bytes)
    arr = np.frombuffer(arr_data, np.float32).reshape((len_arr, 3))
    return arr


def read_matrix4f(server):
    len_matrix = 16 * 4  # 16 entries each 4 bytes
    arr_data = server.stdout.read(len_matrix)
    arr = np.frombuffer(arr_data, np.float32).reshape((4, 4))
    return arr
