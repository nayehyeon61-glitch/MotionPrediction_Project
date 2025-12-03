import os.path
import struct
from datetime import datetime
import math

from scipy.spatial.transform import Rotation
from make_120_bvh.hierarchy import optitrack_etri_hierarchy


class OptitrackBVH:
    def __init__(self):
        self.file_name = ""
        
    def make_bvh(self, optitrack_queue, click_timestamp):    
        while True :
            queue_data = optitrack_queue.get()
            
            if queue_data == True:
                self.bvh_frame_alter(self.file_name)
                print("[bvh] optitrack bvh Done : ", datetime.now())
                break
            
            if 'optitrack' in queue_data:
                data = queue_data['optitrack']
                data_ts = queue_data['TS']
                
                timestamp = struct.unpack('!32s', data_ts)[0].decode()
                convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")
                dir_time = convert_timestamp.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
                dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
                os.makedirs(dir_path, exist_ok=True)
                
                timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
                self.file_name = r"{}/Optitrack_{}.bvh".format(dir_path, timestamp_set)
                
                count_id = 0
                etri_hirerarchy_order = [0, "waist", 3, 4, 25, 26, 27, 6, 7, 8, 47, 48, 49, 50, 43, 44, 45, 46]
                
                segment_data = data['skeleton']
                
                pos_x = format(segment_data[0][1]*100, '.6f')
                pos_y = format(segment_data[0][2]*100, '.6f')
                pos_z = format(segment_data[0][3]*100, '.6f')
                
                if not self.file_exist():
                    with open(self.file_name, 'w') as file:
                        file.write(''.join(map(str, optitrack_etri_hierarchy())))
                        file.write(f"Frames:\n")
                        file.write(f"Frame Time: {float(0.008333)}\n")
                else:
                    with open(self.file_name, 'a') as file:
                        file.write(f"{pos_x} {pos_y} {pos_z}")
                        
                        for idx, order in enumerate(etri_hirerarchy_order):
                            
                            if order != "waist":
                                quaternion_data = segment_data[order][4:]
                            else:
                                pass
                            
                            roll_x_deg, pitch_y_deg, yaw_z_deg = self.quaternion_to_euler(quaternion_data[0], quaternion_data[1], quaternion_data[2], quaternion_data[3])
                            rot_x = roll_x_deg
                            rot_y = pitch_y_deg
                            rot_z = yaw_z_deg
            
                            if order == 0:
                                pelvis_data = segment_data[0][4:]
                                pelvis_x, pelvis_y, pelvis_z = self.quaternion_to_euler_ZYX(pelvis_data[0], pelvis_data[1], pelvis_data[2], pelvis_data[3])
                                rot_x = format(pelvis_x, '.6f')
                                rot_y = format(pelvis_y, '.6f')
                                rot_z = format(pelvis_z, '.6f')
                                
                                file.write(f" {rot_x}")
                                file.write(f" {rot_y}")
                                file.write(f" {rot_z}")
                            
                            elif order == "waist":
                                spine_data = segment_data[1][4:]
                                spine_x, spine_y, spine_z = self.quaternion_to_euler(spine_data[0], spine_data[1], spine_data[2], spine_data[3])
                                
                                spine1_data = segment_data[2][4:]
                                spine1_x, spine1_y, spine1_z = self.quaternion_to_euler(spine1_data[0], spine1_data[1], spine1_data[2], spine1_data[3])
                                
                                rot_x = spine_x + (spine1_x *0.5)
                                rot_y = spine_y + (spine1_y *0.5)
                                rot_z = spine_z + (spine1_z *0.5)
                                
                                file.write(f" {format(rot_x, '.6f')}")
                                file.write(f" {format(rot_y, '.6f')}")
                                file.write(f" {format(rot_z, '.6f')}")
                                
                            elif order == 3:
                                spine_data = segment_data[2][4:]
                                spine_x, spine_y, spine_z = self.quaternion_to_euler(spine_data[0], spine_data[1], spine_data[2], spine_data[3])
                                
                                rot_x = float(rot_x) + spine_x
                                rot_y = float(rot_y) + spine_y
                                rot_z = float(rot_z) + spine_z
                                
                                file.write(f" {format(rot_x, '.6f')}")
                                file.write(f" {format(rot_y, '.6f')}")
                                file.write(f" {format(rot_z, '.6f')}")
                                
                            elif order == 25:
                                rshoulder_data = segment_data[24][4:]
                                rshoulder_x, rshoulder_y, rshoulder_z = self.quaternion_to_euler(rshoulder_data[0], rshoulder_data[1], rshoulder_data[2], rshoulder_data[3])
                                spine_data = segment_data[2][4:]
                                spine_x, spine_y, spine_z = self.quaternion_to_euler(spine_data[0], spine_data[1], spine_data[2], spine_data[3])
                                
                                rot_x = float(rot_x) + rshoulder_x + spine_x
                                rot_y = float(rot_y) + rshoulder_y + spine_y
                                rot_z = float(rot_z) + rshoulder_z + spine_z
                                
                                file.write(f" {format(rot_x, '.6f')}")
                                file.write(f" {format(rot_y, '.6f')}")
                                file.write(f" {format(rot_z, '.6f')}")
                                
                            elif order == 6:
                                lshoulder_data = segment_data[5][4:]
                                lshoulder_x, lshoulder_y, lshoulder_z = self.quaternion_to_euler(lshoulder_data[0], lshoulder_data[1], lshoulder_data[2], lshoulder_data[3])
                                spine_data = segment_data[2][4:]
                                spine_x, spine_y, spine_z = self.quaternion_to_euler(spine_data[0], spine_data[1], spine_data[2], spine_data[3])
                                
                                rot_x = float(rot_x) + lshoulder_x + spine_x
                                rot_y = float(rot_y) + lshoulder_y + spine_y
                                rot_z = float(rot_z) + lshoulder_z + spine_z
                                
                                file.write(f" {format(rot_x, '.6f')}")
                                file.write(f" {format(rot_y, '.6f')}")
                                file.write(f" {format(rot_z, '.6f')}")
                                
                            else:
                                file.write(f" {format(rot_x, '.6f')}")
                                file.write(f" {format(rot_y, '.6f')}")
                                file.write(f" {format(rot_z, '.6f')}")
                        file.write("\n")

    def file_exist(self):
        path = self.file_name
        is_exist = os.path.exists(path)

        return is_exist
    
    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        # 주어진 쿼터니언 값
        quaternion = [x, y, z, w]

        # 쿼터니언을 회전 행렬로 변환
        rotation_matrix = Rotation.from_quat(quaternion).as_matrix()

        # 회전 행렬을 사용하여 오일러 각 계산
        euler_angles = Rotation.from_matrix(rotation_matrix).as_euler('ZXY', degrees=True)
        
        return euler_angles[0], euler_angles[1], euler_angles[2]

    @staticmethod
    def quaternion_to_euler_ZYX(x, y, z, w):
        # 주어진 쿼터니언 값
        quaternion = [x, y, z, w]

        # 쿼터니언을 회전 행렬로 변환
        rotation_matrix = Rotation.from_quat(quaternion).as_matrix()

        # 회전 행렬을 사용하여 오일러 각 계산
        euler_angles = Rotation.from_matrix(rotation_matrix).as_euler('ZYX', degrees=True)
        
        return euler_angles[0], euler_angles[1], euler_angles[2]

    @staticmethod
    def radian_to_degree(rad):
        return rad * 180 / math.pi

    @staticmethod
    def bvh_frame_alter(file_path):
        if os.path.exists(file_path):
            with open(file_path, 'r') as bvh_file_read:
                lines = bvh_file_read.readlines()
                # org_hireracrchy
                # lines[309] = f"Frames: {len(lines) - 311}\n"
                # ETRI_hireracrchy
                lines[112] = f"Frames: {len(lines) - 114}\n"
                lines[113] = f"Frame Time: {float(0.008333)}\n"
                with open(file_path, 'w') as bvh_file:
                    bvh_file.writelines(lines)
