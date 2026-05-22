"""
export PYTHONPATH=/century/bazel-bin/cyber/python/internal/:$PYTHONPATH
"""
import sys
sys.path.append('.')
from cyber.python.cyber_py3 import cyber
sys.path.append('/century/bazel-bin')
from modules.drivers.proto.pointcloud_pb2 import PointXYZIRTCloud
from modules.drivers.proto.pointcloud_pb2 import PointCloudPacked
from modules.drivers.proto.sensor_image_pb2 import CompressedImage
from modules.localization.proto.localization_pb2 import LocalizationEstimate
from cyber.python.cyber_py3 import record
from pathlib import Path
import numpy as np
import cv2
import struct
from fire import Fire 
import json
import struct
from tqdm import tqdm
import multiprocessing
import os
import traceback
from concurrent.futures import ProcessPoolExecutor, as_completed

CLASS_MAPPING = {"century.drivers.PointXYZIRTCloud" : "PointXYZIRTCloud",
                 "century.drivers.CompressedImage" : "CompressedImage",
                 "century.localization.LocalizationEstimate" : "LocalizationEstimate",
                 "century.drivers.PointCloudPacked" : "PointCloudPacked"}

def convert_point(x, y, z, intensity, ring, timestamp):
    intensity_u8 = max(0, min(int(intensity), 255))
    ring_u8 = max(0, min(int(ring), 255))
    timestamp_u16 = int(timestamp) % 65536
    return struct.pack('fffBBH', 
        float(x), float(y), float(z),
        intensity_u8, ring_u8, timestamp_u16
    )

def write_pcd_header(f, point_count):
    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity ring timestamp_2us
SIZE 4 4 4 1 1 2
TYPE F F F U U U
COUNT 1 1 1 1 1 1
WIDTH {point_count}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {point_count}
DATA binary\n"""
    f.write(header.encode('ascii'))

def save_as_pcd(points, filename):
    with open(filename, 'wb') as f:
        write_pcd_header(f, len(points))
        for point in points:
            f.write(point)

def process_message(args):
    topic, msg, data_type, time_stamp, bag_file_name, out_dir = args
    try:
        if (data_type in CLASS_MAPPING.keys()) and (CLASS_MAPPING[data_type] == PointXYZIRTCloud.__name__):
            topic_clean = topic.replace('/', '_')
            while (topic_clean.startswith('_')):
                topic_clean = topic_clean[1:]
            if topic_clean.startswith('lidar_'):
                topic_clean = topic_clean[6:]
            file_dir = Path(out_dir) / bag_file_name / 'lidar' / topic_clean
            if not file_dir.exists():
                file_dir.mkdir(parents=True, exist_ok=True)
            points = PointXYZIRTCloud()
            points.ParseFromString(msg)
            new_time_stamp = str(time_stamp)
            new_time_stamp = new_time_stamp[:10] + '.' + new_time_stamp[10:]
            output_file_name = file_dir / (str(new_time_stamp) + ".pcd")
            pcd_points = []
            for point in points.point:
                x = point.x
                y = point.y
                z = point.z
                intensity = point.intensity
                ring = point.ring
                timestamp = point.timestamp
                pcd_points.append(convert_point(x, y, z, intensity, ring, timestamp))
            save_as_pcd(pcd_points, str(output_file_name))
            return True
            
        elif (data_type in CLASS_MAPPING.keys()) and (CLASS_MAPPING[data_type] == CompressedImage.__name__):
            topic_clean = topic.replace('/', '_')
            while (topic_clean.startswith('_')):
                topic_clean = topic_clean[1:]
            topic_clean = topic_clean.replace('_compressed', '').replace('century_sensor_camera_', '')
            file_dir = Path(out_dir) / bag_file_name / 'camera' / topic_clean
            if not file_dir.exists():
                file_dir.mkdir(parents=True, exist_ok=True)
            compressed_img = CompressedImage()
            compressed_img.ParseFromString(msg)
            np_arr = np.frombuffer(compressed_img.data, np.uint8)
            new_time_stamp = str(time_stamp)
            new_time_stamp = new_time_stamp[:10] + '.' + new_time_stamp[10:]
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img is not None:
                output_path = file_dir / f"{new_time_stamp}.jpg"
                cv2.imwrite(str(output_path), img)
                return True
            return False
            
        elif (data_type in CLASS_MAPPING.keys()) and (CLASS_MAPPING[data_type] == LocalizationEstimate.__name__):
            topic_clean = topic.replace('/', '_')
            while (topic_clean.startswith('_')):
                topic_clean = topic_clean[1:]
            file_dir = Path(out_dir) / bag_file_name / 'localization'
            if not file_dir.exists(): 
                file_dir.mkdir(parents=True, exist_ok=True)
            localization = LocalizationEstimate()
            localization.ParseFromString(msg)
            new_time_stamp = str(time_stamp)
            new_time_stamp = new_time_stamp[:10] + '.' + new_time_stamp[10:]
            output_file_name = file_dir / (str(new_time_stamp) + ".json")
            loc_dict = {
              "position" : {
                "x" : localization.pose.position.x,
                "y" : localization.pose.position.y,
                "z" : localization.pose.position.z
              },
              "orientation" : {
                "w" : localization.pose.orientation.qw,
                "x" : localization.pose.orientation.qx,
                "y" : localization.pose.orientation.qy,
                "z" : localization.pose.orientation.qz,
              },
              "linear_velocity" : {
                "x" : localization.pose.linear_velocity.x,
                "y" : localization.pose.linear_velocity.y,
                "z" : localization.pose.linear_velocity.z
              },
              "linear_acceleration" : {
                "x" : localization.pose.linear_acceleration.x,
                "y" : localization.pose.linear_acceleration.y,
                "z" : localization.pose.linear_acceleration.z
              },
              "angular_velocity" : {
                "x" : localization.pose.angular_velocity.x,
                "y" : localization.pose.angular_velocity.y,
                "z" : localization.pose.angular_velocity.z
              }
            }
            with open(str(output_file_name), "w") as f:
                json.dump(loc_dict, f)
            return True
            
        elif (data_type in CLASS_MAPPING.keys()) and (CLASS_MAPPING[data_type] == PointCloudPacked.__name__):
            topic_clean = topic.replace('/', '_')
            while (topic_clean.startswith('_')):
                topic_clean = topic_clean[1:]
            if topic_clean.startswith('lidar_'):
                topic_clean = topic_clean[6:]
            file_dir = Path(out_dir) / bag_file_name / 'lidar' / topic_clean
            if not file_dir.exists():
                file_dir.mkdir(parents=True, exist_ok=True)
            packed_points = PointCloudPacked()
            packed_points.ParseFromString(msg)
            new_time_stamp = str(time_stamp)
            new_time_stamp = new_time_stamp[:10] + '.' + new_time_stamp[10:]
            output_file_name = file_dir / (str(new_time_stamp) + ".pcd")
            data_size = packed_points.point_size
            data_len = packed_points.data.__len__()
            point_format = 'fffHHd'
            point_type_size = struct.calcsize('fffHHd')
            assert (data_len // data_size) == point_type_size, f"Data size mismatch"
            pcd_points = []

            try:
                dtype = np.dtype([('x', 'f4'), ('y', 'f4'), ('z', 'f4'), 
                                ('intensity', 'u2'), ('ring', 'u2'), 
                                ('timestamp', 'f8')])
                
                if dtype.names is not None:
                    data_array = np.frombuffer(packed_points.data, dtype=dtype)
                    
                    for point in data_array:
                        pcd_points.append(convert_point(
                            point['x'], point['y'], point['z'],
                            point['intensity'], point['ring'], point['timestamp']
                        ))
                    
                    save_as_pcd(pcd_points, str(output_file_name))
                    return True
                    
            except Exception as e:
                pcd_points = []
                print(f"Error in numpy parsing: {e}")
                for i in range(0, data_size):
                    start_idx = i * point_type_size
                    end_idx = start_idx + point_type_size
                    try:
                        point_bytes = packed_points.data[start_idx:end_idx]
                        x, y, z, intensity, ring, timestamp = struct.unpack(point_format, point_bytes)
                        pcd_points.append(convert_point(x, y, z, intensity, ring, timestamp))
                    except struct.error as e:
                        continue

                save_as_pcd(pcd_points, str(output_file_name))
                return True
                
        return False
        
    except Exception as e:
        print(f"Error processing message: {e}")
        return False

def parse_single_bag(bag_name, out_dir):
    print(f"\n{'='*60}")
    print(f"Processing: {bag_name}")
    print(f"{'='*60}")
    
    try:
        bag_reader = record.RecordReader(bag_name)
        bag_file_name = Path(bag_name).name.replace('.', '_')
        
        # 收集所有消息
        all_messages = []
        for topic, msg, data_type, time_stamp in bag_reader.read_messages():
            all_messages.append((topic, msg, data_type, time_stamp, bag_file_name, out_dir))
        
        total_messages = len(all_messages)
        print(f"Total messages: {total_messages}")
        
        num_workers = max(multiprocessing.cpu_count() // 2, 8)
        print(f"Using {num_workers} workers for this file")
        
        success_count = 0
        with ProcessPoolExecutor(max_workers=num_workers) as executor:
            futures = {executor.submit(process_message, msg): msg for msg in all_messages}
            
            with tqdm(total=total_messages, desc=f"Processing {Path(bag_name).name}", 
                     unit="msg", dynamic_ncols=True) as pbar:
                
                for future in as_completed(futures):
                    try:
                        result = future.result()
                        if result:
                            success_count += 1
                    except Exception as e:
                        print(f"Task failed: {e}")
                    finally:
                        pbar.update(1)
        
        print(f"Completed: {success_count}/{total_messages} messages processed successfully")
        return True
        
    except Exception as e:
        print(f"Error processing bag {bag_name}: {e}")
        traceback.print_exc()
        return False

def parse(bag_name="/century/data/camera_data/202506201730.record",
          out_dir="/century/data/bag/"):
    print("Processing {} with multiprocessing".format(bag_name))
    return parse_single_bag(bag_name, out_dir)

def parse_bag(file_lst="modules/perception/tool/bag_process/file_lst"):
    fail_log = Path(file_lst).parent / "fail_lst"
    result = []
    
    with open(file_lst, 'r') as file:
        for line in file:
            stripped_line = line.strip()
            if not stripped_line.startswith('#'):
                if stripped_line:
                    result.append(stripped_line)
    
    print(f"Found {len(result)} bags to process (serial processing)")
    
    failed_files = []
    for i, file_path in enumerate(result, 1):
        print(f"\n[{i}/{len(result)}] Processing: {file_path}")
        
        file_dir = Path(file_path).parent
        output_dir = file_dir.parent / (file_dir.name + "_out")
        if not output_dir.exists():
            output_dir.mkdir(parents=True, exist_ok=True)
            
        success = parse_single_bag(file_path, str(output_dir))
        if not success:
            failed_files.append(file_path)
    
    if failed_files:
        with open(fail_log, 'a') as err_f:
            for failed_file in failed_files:
                err_f.write(f"{failed_file}\n")
        print(f"\n Failed to process {len(failed_files)} files, see {fail_log}")
    else:
        print(f"\n All {len(result)} files processed successfully!")

def parse_bag_dir(file_dir_list="modules/perception/tool/bag_process/file_dir"):
    fail_log = Path(file_dir_list).parent / "fail_lst"
    result = []
    
    with open(file_dir_list, 'r') as file:
        for line in file:
            stripped_line = line.strip()
            if not stripped_line.startswith('#'):
                if stripped_line:
                    result.append(stripped_line)
    
    record_files = []
    for dir_path in result:
        record_dir = Path(dir_path)
        record_files.extend(list(record_dir.rglob("*.record")))
    
    print(f"Found {len(record_files)} record files to process (serial processing)")
    
    failed_files = []
    for i, record_file in enumerate(record_files, 1):
        print(f"\n[{i}/{len(record_files)}] Processing: {record_file}")
        
        output_dir = record_file.parent.parent / (record_file.parent.name + "_out")
        if not output_dir.exists():
            output_dir.mkdir(parents=True, exist_ok=True)
            
        success = parse_single_bag(str(record_file), str(output_dir))
        if not success:
            failed_files.append(str(record_file))
    
    if failed_files:
        with open(fail_log, 'a') as err_f:
            for failed_file in failed_files:
                err_f.write(f"{failed_file}\n")
        print(f"\n Failed to process {len(failed_files)} files, see {fail_log}")
    else:
        print(f"\n All {len(record_files)} files processed successfully!")

if __name__ == "__main__":  
    multiprocessing.set_start_method('spawn', force=True)
    Fire(parse_bag_dir)