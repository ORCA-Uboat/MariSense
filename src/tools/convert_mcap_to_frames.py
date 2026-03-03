import sys
import numpy as np
import struct
from mcap_ros2.decoder import DecoderFactory
from mcap.reader import make_reader
import tqdm
import os
import shutil



import argparse
from pathlib import Path

rostime_dump = lambda msg: msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

LIDAR_PATH_AVIA = None
LIDAR_PATH_MID360 =  None
RADAR_PATH     =  None
GPS_PATH       =  None
GPSROVER_PATH      =  None
PR_PATH        =  None

LIDAR_TOPICS = [
    '/lidar_mid360',
    '/lidar_avia',
]

RADAR_TOPICS_1 = {
    '/radar_front_0': 0,
    '/radar_left_1': 1,
    '/radar_back_2': 2,
    '/radar_right_3': 3,
    '/radar_front_4': 4,
    '/radar_front_5': 5,
    '/radar_front_6': 6,
    '/radar_front_7': 7,
    '/radar_front_8': 8,
    '/radar_front_9': 9
}

RADAR_TOPICS_2 = {
    '/radar_left_0': 0,
    '/radar_back_1': 1,
    '/radar_right_2': 2,
    '/radar_front_3': 3,
    '/radar_front_4': 4,
    '/radar_front_5': 5,
    '/radar_front_6': 6,
    '/radar_front_7': 7,
    '/radar_front_8': 8,
    '/radar_front_9': 9
}

def make_dirs(output_path):
    global LIDAR_PATH_AVIA, LIDAR_PATH_MID360, RADAR_PATH, GPS_PATH, GPSROVER_PATH, PR_PATH
    LIDAR_PATH_AVIA = os.path.join(output_path, 'lidar_avia')
    LIDAR_PATH_MID360 = os.path.join(output_path, 'lidar_mid360')
    RADAR_PATH     = os.path.join(output_path, 'radar')
    GPS_PATH       = os.path.join(output_path, 'gps')
    GPSROVER_PATH      = os.path.join(output_path, 'gpsrover')
    PR_PATH        = os.path.join(output_path, 'imu')

    if os.path.exists(LIDAR_PATH_AVIA):
        shutil.rmtree(LIDAR_PATH_AVIA)
    os.makedirs(LIDAR_PATH_AVIA)

    if os.path.exists(LIDAR_PATH_MID360):
        shutil.rmtree(LIDAR_PATH_MID360)
    os.makedirs(LIDAR_PATH_MID360)

    if os.path.exists(RADAR_PATH):
        shutil.rmtree(RADAR_PATH)
    for i in range(10):
        os.makedirs(os.path.join(RADAR_PATH, str(i)))

    for path in [GPS_PATH, GPSROVER_PATH, PR_PATH]:
        if os.path.exists(path):
            shutil.rmtree(path)
        os.makedirs(path)


def write_pcd_file(file_path, points, sensor_type='radar'):
    """
    将点云数据写入PCD文件
    :param file_path: PCD文件保存路径
    :param points: 点云数据，shape=(N, 6)，dtype=float32
    :param fields: 点云字段名，需与points列数匹配
    """
    if sensor_type == 'lidar':
        fields = ['x', 'y', 'z', 'intensity', 'tag', 'line']
    else:
        fields = ['x', 'y', 'z', 'speed', 'snr', 'noise']

    pcd_header = [
        '# .PCD v0.7 - Point Cloud Data file format',
        'VERSION 0.7',
        f'FIELDS {" ".join(fields)}',
        'SIZE 4 4 4 4 4 4',
        'TYPE F F F F F F',
        'COUNT 1 1 1 1 1 1',
        f'WIDTH {points.shape[0]}',
        'HEIGHT 1',
        'VIEWPOINT 0 0 0 1 0 0 0',
        f'POINTS {points.shape[0]}',
        'DATA ascii'
    ]

    with open(file_path, 'w') as f:
        f.write('\n'.join(pcd_header) + '\n')
        for point in points:
            f.write(' '.join(map(str, point)) + '\n')


def lidar_analyze_online(msg) -> np.ndarray:
    """解析Livox激光雷达PointCloud2数据
    
    Args:
        msg (PointCloud2): ROS2 PointCloud2消息
    
    Returns:
        np.ndarray: 点云数据，shape=(N, 6)，列：x,y,z,intensity,tag,line
    """
    data_l = msg.width
    points = struct.unpack('<' + 'ffffBB' * data_l, msg.data)
    points = np.asarray(points).reshape((-1, 6)).astype(np.float32)
    return points


def radar_analyze_online(msg) -> np.ndarray:
    """解析雷达PointCloud2数据
    
    Args:
        msg (PointCloud2): ROS2 PointCloud2消息
    
    Returns:
        np.ndarray: 点云数据，shape=(N, 6)，列：x,y,z,speed,snr,noise
    """
    data_l = msg.width
    points = struct.unpack('<' + 'ffffHH' * data_l, msg.data)
    points = np.asarray(points).reshape((-1, 6)).astype(np.float32)
    return points


def lidar_process_avia(ros_msg):
    points = lidar_analyze_online(ros_msg)
    data_time = rostime_dump(ros_msg)
    pcd_path = os.path.join(LIDAR_PATH_AVIA, f'avia_{data_time:.3f}.pcd')
    write_pcd_file(pcd_path, points)

def lidar_process_mid360(ros_msg):
    points = lidar_analyze_online(ros_msg)
    data_time = rostime_dump(ros_msg)
    pcd_path = os.path.join(LIDAR_PATH_MID360, f'mid360_{data_time:.3f}.pcd')
    write_pcd_file(pcd_path, points, 'lidar')

def mid360_imu_process(ros_msg):
    data_time = rostime_dump(ros_msg)
    data = {
        'ros_time': round(data_time, 3),
        'orientation_x': round(ros_msg.orientation.x, 5),
        'orientation_y': round(ros_msg.orientation.y, 5),
        'orientation_z': round(ros_msg.orientation.z, 5),
        'orientation_w': round(ros_msg.orientation.w, 5),
        'angular_velocity_x': round(ros_msg.angular_velocity.x, 5),
        'angular_velocity_y': round(ros_msg.angular_velocity.y, 5),
        'angular_velocity_z': round(ros_msg.angular_velocity.z, 5),
        'linear_acceleration_x': round(ros_msg.linear_acceleration.x, 5),
        'linear_acceleration_y': round(ros_msg.linear_acceleration.y, 5),
        'linear_acceleration_z': round(ros_msg.linear_acceleration.z, 5)
    }

    txt_path = os.path.join(PR_PATH, 'imu_mid360.txt')
    with open(txt_path, 'a') as f:
        f.write(str(data) + '\n')

def avia_imu_process(ros_msg):
    data_time = rostime_dump(ros_msg)
    data = {
        'ros_time': round(data_time, 3),
        'orientation_x': round(ros_msg.orientation.x, 5),
        'orientation_y': round(ros_msg.orientation.y, 5),
        'orientation_z': round(ros_msg.orientation.z, 5),
        'orientation_w': round(ros_msg.orientation.w, 5),
        'angular_velocity_x': round(ros_msg.angular_velocity.x, 5),
        'angular_velocity_y': round(ros_msg.angular_velocity.y, 5),
        'angular_velocity_z': round(ros_msg.angular_velocity.z, 5),
        'linear_acceleration_x': round(ros_msg.linear_acceleration.x, 5),
        'linear_acceleration_y': round(ros_msg.linear_acceleration.y, 5),
        'linear_acceleration_z': round(ros_msg.linear_acceleration.z, 5)
    }

    txt_path = os.path.join(PR_PATH, 'imu_avia.txt')
    with open(txt_path, 'a') as f:
        f.write(str(data) + '\n')

def radar_process(ros_msg, radar_id):
    points = radar_analyze_online(ros_msg)
    data_time = rostime_dump(ros_msg)
    pcd_path = os.path.join(RADAR_PATH, str(radar_id),f'{data_time:.3f}.pcd')
    write_pcd_file(pcd_path, points, 'radar')


def gps_process(ros_msg):
    data_time = rostime_dump(ros_msg)
    data = {
        'ros_time': round(data_time, 3),
        'lat': ros_msg.lat,
        'lng': ros_msg.lng,
        'alt': ros_msg.alt,
        'speed': round(ros_msg.speed, 3),
        'yaw': round(ros_msg.yaw, 3),
        'vel_n': round(ros_msg.vel_n, 3),
        'vel_e': round(ros_msg.vel_e, 3),
        'vel_d': round(ros_msg.vel_d, 3),
        'stars': ros_msg.stars,
        'hdop': round(ros_msg.hdop, 3),
        'mode': ros_msg.mode,
        'rms': round(ros_msg.rms, 3),
        'lat_dev': round(ros_msg.lat_dev, 3),
        'lng_dev': round(ros_msg.lng_dev, 3),
        'alt_dev': round(ros_msg.alt_dev, 3),
        'deviation': round(ros_msg.deviation, 3)
    }
    txt_path = os.path.join(GPS_PATH, 'gps.txt')
    with open(txt_path, 'a') as f:
        f.write(str(data) + '\n')


def gpsr_process(ros_msg):
    """处理GPSR数据，追加写入TXT文件"""
    data_time = rostime_dump(ros_msg)
    data = {'ros_time': round(data_time, 3),
        'length': round(ros_msg.length, 3),
        'yaw': round(ros_msg.yaw, 3),
        'acc': round(ros_msg.acc, 3),
        'mode': ros_msg.mode
    }

    txt_path = os.path.join(GPSROVER_PATH, 'gpsrover.txt')
    with open(txt_path, 'a') as f:
        f.write(str(data) + '\n')


def vessel_imu_process(ros_msg):
    """处理IMU（pitch/roll）数据，追加写入TXT文件"""
    data_time = rostime_dump(ros_msg)
    data = {
        'ros_time': round(data_time, 3),
        'roll': round(ros_msg.roll, 3),
        'pitch': round(ros_msg.pitch, 3),
        'yaw': round(ros_msg.yaw, 3),
        'gx': round(ros_msg.gx, 3),
        'gy': round(ros_msg.gy, 3),
        'gz': round(ros_msg.gz, 3),
        'ax': round(ros_msg.ax, 3),
        'ay': round(ros_msg.ay, 3),
        'az': round(ros_msg.az, 3)
    }

    txt_path = os.path.join(PR_PATH, 'imu_vessel.txt')
    with open(txt_path, 'a') as f:
        f.write(str(data) + '\n')


def main():

    parser = argparse.ArgumentParser(description='Source Data')
    parser.add_argument('--input', type=str, required=True)
    parser.add_argument('--output', type=str, required=True)
    parser.add_argument('--sensors', type=str, required=True)

    args = parser.parse_args()

    make_dirs(str(Path(args.output_path)))

    with open(str(Path(args.input_path)), "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for schema, channel, message, ros_msg in tqdm.tqdm(
                reader.iter_decoded_messages()):
            current_topic = channel.topic
            if 'lidar' in sensor_str:
                if current_topic in LIDAR_TOPICS:
                    lidar_process_avia(ros_msg)
                if current_topic in LIDAR_TOPICS:
                    lidar_process_mid360(ros_msg)

            if 'radar' in sensor_str:
                radar_id = None
                if current_topic in RADAR_TOPICS_1:
                    radar_id = RADAR_TOPICS_1[current_topic]
                    radar_process(ros_msg, radar_id)
                elif current_topic in RADAR_TOPICS_2:
                    radar_id = RADAR_TOPICS_2[current_topic]
                    radar_process(ros_msg, radar_id)

            if 'imu' in sensor_str:
                if current_topic == '/imu_mid360':
                    mid360_imu_process(ros_msg)

                if current_topic == '/imu_avia':
                    avia_imu_process(ros_msg)

                if current_topic == '/vessel_imu':
                    vessel_imu_process(ros_msg)

            if 'gnss' in sensor_str:
                if current_topic == '/gnss_gps':
                    gps_process(ros_msg)

                if current_topic == '/gnss_gpsrover':
                    gpsr_process(ros_msg)


if __name__ == "__main__":
    main()
