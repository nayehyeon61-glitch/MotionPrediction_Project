import os
from datetime import datetime
import math
from scipy.spatial.transform import Rotation

from hierarchy import xsens_etri_hierarchy, optitrack_etri_hierarchy

# Xsens_BVH ===================================

def file_exist(file_path):
    return os.path.isfile(file_path)

def xsens_bvh_etri(xsens_data_list, file_path):
    pos_x = format(xsens_data_list[0][8], '.6f')
    pos_y = format(xsens_data_list[0][9], '.6f')
    pos_z = format(xsens_data_list[0][10], '.6f')
    
    if os.path.exists(file_path):
        with open(file_path, 'a') as file:
            file.write(f"{pos_x} {pos_y} {pos_z}")
            etri_hirerarchy_order = [0, 1, 5, 6, 8, 9, 10, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]
            hirerachy_order_body = [0, 1, 2, 3, 4, 5, 6]
            hirerachy_order_r_shoulder = [0, 1, 2, 3, 4, 7, 8, 9, 10]
            hirerachy_order_l_shoulder = [0, 1, 2, 3, 4, 11, 12, 13, 14]
            hirerachy_order_r_hip = [0, 15, 16, 17, 18]
            hirerachy_order_l_hip = [0, 19, 20, 21, 22]

            for order in etri_hirerarchy_order:
            # for order in range(len(xsens_data_list)):
                xsens_quat = xsens_data_list[order][1:5]
                xsens_euler = xsens_data_list[order][5:8]
                
                data = xsens_quat
                
                if data == xsens_quat:
                    roll_x_deg, pitch_y_deg, yaw_z_deg = quaternion_to_euler_xsens(xsens_quat)
                    
                    if order == 0:
                        rot_x = format(roll_x_deg, '.6f')
                        rot_y = format(pitch_y_deg, '.6f')
                        rot_z = format(yaw_z_deg, '.6f')
                    
                    # neck
                    elif order == 5:
                        before_data = xsens_data_list[4][1:5]
                        before_x, before_y, before_z = quaternion_to_euler_xsens(before_data)
                        
                        chest1_data = xsens_data_list[1][1:5]
                        chest1_x, chest1_y, chest1_z = quaternion_to_euler_xsens(chest1_data)
                        
                        chest2_data = xsens_data_list[2][1:5]
                        chest2_x, chest2_y,chest2_z = quaternion_to_euler_xsens(chest2_data)
                        
                        chest3_data = xsens_data_list[3][1:5]
                        chest3_x, chest3_y,chest3_z = quaternion_to_euler_xsens(chest3_data)
                        
                        chest4_data = xsens_data_list[4][1:5]
                        chest4_x, chest4_y,chest4_z = quaternion_to_euler_xsens(chest4_data)
                        
                        
                        rot_x = roll_x_deg - chest1_x
                        rot_y = pitch_y_deg - chest1_y
                        rot_z = yaw_z_deg - chest1_z

                    # shoulder
                    elif order == 8:
                        before_data = xsens_data_list[7][1:5]
                        before_x, before_y, before_z = quaternion_to_euler_xsens(before_data)
                        
                        r_collar_data = xsens_data_list[7][1:5]
                        r_collar_x, r_collar_y, r_collar_z = quaternion_to_euler_xsens(r_collar_data)
                        
                        chest4_data = xsens_data_list[4][1:5]
                        chest4_x, chest4_y,chest4_z = quaternion_to_euler_xsens(chest4_data)
                        
                        rot_x = roll_x_deg - chest4_x
                        rot_y = chest4_y - pitch_y_deg
                        rot_z = yaw_z_deg - chest4_z
                        
                    elif order == 12:
                        chest4_data = xsens_data_list[4][1:5]
                        chest4_x, chest4_y,chest4_z = quaternion_to_euler_xsens(chest4_data)
                        
                        rot_x = roll_x_deg - chest4_x
                        rot_y = pitch_y_deg - chest4_y
                        rot_z = yaw_z_deg - chest4_z
                                
                    else:
                        if order == 1 or order == 6:
                            list_sort = hirerachy_order_body
                        elif 9 <= order <= 10:
                            list_sort = hirerachy_order_r_shoulder
                        elif 13 <= order <= 14:
                            list_sort = hirerachy_order_l_shoulder
                        elif 15 <= order <= 18:
                            list_sort = hirerachy_order_r_hip
                        elif 19 <= order <= 22:
                            list_sort = hirerachy_order_l_hip

                        for list_order in list_sort:
                            if list_order >= order:
                                break
                            else:
                                before_data = xsens_data_list[list_order][1:5]
                                before_x, before_y, before_z = quaternion_to_euler_xsens(before_data)
                                
                                rot_x = roll_x_deg - before_x
                                rot_y = pitch_y_deg- before_y
                                rot_z = yaw_z_deg - before_z
                                
                if data == xsens_quat:
                    file.write(f" {rot_x}")
                    file.write(f" {rot_z}")
                    file.write(f" {rot_y}")
            file.write("\n")
    else:
        with open(file_path, 'w') as file:
            file.write(''.join(map(str, xsens_etri_hierarchy())))
            file.write(f"Frames:\n")
            file.write(f"Frame Time: {float(0.008333)}\n")
            
def xsens_bvh_etri_v2(xsens_data_list, file_path):
    pos_x = format(xsens_data_list[0][8], '.6f')
    pos_y = format(xsens_data_list[0][9], '.6f')
    pos_z = format(xsens_data_list[0][10], '.6f')
    
    if os.path.exists(file_path):
        with open(file_path, 'a') as file:
            file.write(f"{pos_x} {pos_y} {pos_z}")
            etri_hirerarchy_order = [0,2,5,6,8,9,10,12,13,14,15,16,17,18,19,20,21,22]
            
            hirerachy_order_body = [0,1,2,3,4,5,6]
            hirerachy_order_r_shoulder = [0,1,2,3,4,7,8,9,10]
            hirerachy_order_l_shoulder = [0,1,2,3,4,11,12,13,14]
            hirerachy_order_r_hip = [0,15,16,17,18]
            hirerachy_order_l_hip = [0,19,20,21,22]
            
            for order in etri_hirerarchy_order:
                xsens_quat = xsens_data_list[order][1:5]
                xsens_euler = xsens_data_list[order][5:8]
                data = xsens_quat
                
                if data == xsens_quat:
                    roll_x_deg, pitch_y_deg, yaw_z_deg = quaternion_to_euler_xsens_v2(xsens_quat, 'ZXY')
                    
                    rot_x = roll_x_deg
                    rot_y = pitch_y_deg
                    rot_z = yaw_z_deg
                    
                    if order == 0:                            
                        hroll_x_deg, hpitch_y_deg, hyaw_z_deg = quaternion_to_euler_xsens_v2(xsens_data_list[0][1:5], 'ZYX')
                        x = format(hroll_x_deg, '.6f')
                        y = format(hpitch_y_deg, '.6f')
                        z = format(hyaw_z_deg, '.6f')
                        
                        rot_x = x
                        rot_y = y
                        rot_z = z

                    # neck
                    elif order == 5:
                        chest4_data = xsens_data_list[4][1:5]
                        chest4_x, chest4_y, chest4_z = quaternion_to_euler_xsens_v2(chest4_data, 'xyz')
                        
                        neck_data = xsens_data_list[5][1:5]
                        neck_x, neck_y, neck_z = quaternion_to_euler_xsens_v2(neck_data, 'xyz')
                        
                        rot_x = format(neck_x - chest4_x, '.6f')
                        rot_y = format(neck_y - chest4_y, '.6f')
                        rot_z = format(neck_z - chest4_z, '.6f')
                        
            
                    # R_shoulder
                    elif order == 8:                                                 
                        rshoulder_data = xsens_data_list[8][1:5]
                        rshoulder_x, rshoulder_y, rshoulder_z = quaternion_to_euler_xsens_v2(rshoulder_data, 'xyz')
                        
                        r_collar_data = xsens_data_list[7][1:5]
                        r_collar_x, r_collar_y, r_collar_z = quaternion_to_euler_xsens_v2(r_collar_data, 'xyz')
                        
                        chest1_data = xsens_data_list[1][1:5]
                        chest1_x, chest1_y, chest1_z = quaternion_to_euler_xsens_v2(chest1_data, 'xyz')
                        
                        chest2_data = xsens_data_list[1][1:5]
                        chest2_x, chest2_y, chest2_z = quaternion_to_euler_xsens_v2(chest2_data, 'xyz')
                                                    
                        test_x = rshoulder_x - chest1_x
                        test_y = rshoulder_y - chest1_y
                        test_z = rshoulder_z - chest1_z
                        
                        rot_x = format(test_x, '.6f')
                        rot_y = format(-test_y, '.6f')
                        rot_z = format(-test_z, '.6f')
                        
                    # L_shoulder
                    elif order == 12:                                                        
                        chest1_data = xsens_data_list[1][1:5]
                        chest1_x, chest1_y, chest1_z = quaternion_to_euler_xsens_v2(chest1_data, 'xyz')
                        
                        lshoulder_data = xsens_data_list[12][1:5]
                        lshoulder_x, lshoulder_y, lshoulder_z = quaternion_to_euler_xsens_v2(lshoulder_data, 'xyz')
                    
                        test_x = lshoulder_x - chest1_x
                        test_y = lshoulder_y - chest1_y
                        test_z = lshoulder_z - chest1_z
                        
                        rot_x = format(test_x, '.6f')
                        rot_y = format(test_y, '.6f')
                        rot_z = format(test_z, '.6f')

                    else:
                        if order == 2 or order == 6:
                            list_sort = hirerachy_order_body
                        elif 9 <= order <= 10:
                            list_sort = hirerachy_order_r_shoulder
                        elif 13 <= order <= 14:
                            list_sort = hirerachy_order_l_shoulder
                        elif 15 <= order <= 18:
                            list_sort = hirerachy_order_r_hip
                        elif 19 <= order <= 22:
                            list_sort = hirerachy_order_l_hip
                        
                        count = 0
                        for list_order in list_sort:
                            if list_order == order:
                                
                                if order == 2:
                                    index = list_sort[count-1]
                                    before_data = xsens_data_list[index][1:5]
                                    before_x, before_y, before_z = quaternion_to_euler_xsens_v2(before_data, 'ZXY')
                                    rot_x = format(roll_x_deg - before_x, '.6f')
                                    rot_y = format(pitch_y_deg - before_y, '.6f')
                                    rot_z = format(yaw_z_deg - before_z, '.6f')
                                    
                                elif 1 <= order <= 14:
                                    index = list_sort[count-1]
                                    before_data = xsens_data_list[index][1:5]
                                    before_x, before_y, before_z = quaternion_to_euler_xsens_v2(before_data, 'ZXY')
                                    
                                    order_data = xsens_data_list[index+1][1:5]
                                    order_x, order_y, order_z = quaternion_to_euler_xsens_v2(order_data, 'ZXY')
                                    
                                    # R elbow, R_wrist
                                    if 9 <= order <= 10:
                                        index = list_sort[count-1]
                                        before_data = xsens_data_list[index][1:5]
                                        before_x, before_y, before_z = quaternion_to_euler_xsens_v2(before_data, 'xyz')
                                        
                                        order_data = xsens_data_list[index+1][1:5]
                                        order_x, order_y, order_z = quaternion_to_euler_xsens_v2(order_data, 'xyz')
                                        
                                        test_x = order_x - before_x
                                        test_y = order_y - before_y
                                        test_z = order_z - before_z
                                        
                                        rot_x = format(test_x, '.6f')
                                        rot_y = format(-test_y, '.6f')
                                        rot_z = format(test_z, '.6f')
                                            
                                    elif 13 <= order <= 14:
                                        index = list_sort[count-1]
                                        before_data = xsens_data_list[index][1:5]
                                        before_x, before_y, before_z = quaternion_to_euler_xsens_v2(before_data, 'xyz')
                                        
                                        order_data = xsens_data_list[index+1][1:5]
                                        order_x, order_y, order_z = quaternion_to_euler_xsens_v2(order_data, 'xyz')
                                        test_x = order_x - before_x
                                        test_y = order_y - before_y
                                        test_z = order_z - order_z
                                        
                                        rot_x = format(test_x, '.6f')
                                        rot_y = format(test_y, '.6f')
                                        rot_z = format(test_z, '.6f')
                                        
                                    else:
                                        rot_x = format(order_x - before_x, '.6f')
                                        rot_y = format(order_y - before_y, '.6f')
                                        rot_z = format(order_z - before_z, '.6f')

                                else :    
                                    index = list_sort[count-1]
                                    before_data = xsens_data_list[index][1:5]
                                    before_x, before_y, before_z = quaternion_to_euler_xsens_v2(before_data, 'ZXY')
                                    
                                    rot_x = format(roll_x_deg - before_x, '.6f')
                                    rot_y = format(pitch_y_deg - before_y, '.6f')
                                    rot_z = format(yaw_z_deg - before_z, '.6f')

                                break
                            else :
                                count += 1

                    if data == xsens_quat:
                        d_rot_x = radian_to_degree(float(rot_x))
                        d_rot_y = radian_to_degree(float(rot_y))
                        d_rot_z = radian_to_degree(float(rot_z))
                        
                        d_rot_x = format(d_rot_x, '.6f')
                        d_rot_y = format(d_rot_y, '.6f')
                        d_rot_z = format(d_rot_z, '.6f')
                        
                        if order == 0:
                            file.write(f" {d_rot_z}")  # Z
                            file.write(f" {d_rot_x}")  # X
                            file.write(f" {d_rot_y}")  # Y
                            
                        elif 1<= order <= 14:
                            file.write(f" {d_rot_x}")  # Z
                            file.write(f" {d_rot_z}")  # X
                            file.write(f" {d_rot_y}")  # Y
                            
                        else :                                
                            file.write(f" {d_rot_y}")  # Z
                            file.write(f" {d_rot_z}")  # X
                            file.write(f" {d_rot_x}")  # Y

            file.write("\n")
    else:
        with open(file_path, 'w') as file:
            file.write(''.join(map(str, xsens_etri_hierarchy())))
            file.write(f"Frame Time: {float(0.008333)}\n")
            
def quaternion_to_euler_xsens(q):
    # 쿼터니언 값 구성 요소
    a, b, c, d = q

    # 롤 계산
    roll = math.atan2(2 * (a * b + c * d), 1 - 2 * (b**2 + c**2))

    # 피치 계산
    sin_pitch = 2 * (a * c - d * b)
    if abs(sin_pitch) >= 1:
        pitch = math.pi / 2 if sin_pitch > 0 else -math.pi / 2
    else:
        pitch = math.asin(sin_pitch)

    # 요 계산
    yaw = math.atan2(2 * (a * d + b * c), 1 - 2 * (c**2 + d**2))

    roll = radian_to_degree(roll)
    pitch = radian_to_degree(pitch)
    yaw = radian_to_degree(yaw)

    return roll, pitch, yaw

def quaternion_to_euler_xsens_v2(q, rotation):
    [w, x, y, z] = q
    # 주어진 쿼터니언 값
    quaternion = [x, y, z, w]

    # 쿼터니언을 회전 행렬로 변환
    rotation_matrix = Rotation.from_quat(quaternion).as_matrix()

    # 회전 행렬을 사용하여 오일러 각 계산
    euler_angles = Rotation.from_matrix(rotation_matrix).as_euler(rotation, degrees=False)
    
    return euler_angles[0], euler_angles[1], euler_angles[2]

def radian_to_degree(rad):
    return rad * 180 / math.pi

# Xsens_BVH ====================================

# Optitrack_BVH ================================


def quaternion_to_euler(x, y, z, w):
    # 주어진 쿼터니언 값
    quaternion = [x, y, z, w]

    # 쿼터니언을 회전 행렬로 변환
    rotation_matrix = Rotation.from_quat(quaternion).as_matrix()

    # 회전 행렬을 사용하여 오일러 각 계산
    euler_angles = Rotation.from_matrix(rotation_matrix).as_euler('ZXY', degrees=True)
    
    return euler_angles[0], euler_angles[1], euler_angles[2]

def optitrack_bvh(data_dict, file_path):
    # test_hierarchy = hirerarchy.optitrack_etri_hierarchy()
    root_pos = data_dict[0]

    pos_x = format(root_pos[1]*100, '.6f')
    pos_y = format(root_pos[2]*100, '.6f')
    pos_z = format(root_pos[3]*100, '.6f')

    if os.path.exists(file_path):
        with open(file_path, 'a') as file:
            file.write(f"{pos_x} {pos_y} {pos_z}")
            hierarchy_order = [0, "waist", 3, 4, 25, 26, 27, 6, 7, 8, 47, 48, 49, 50, 43, 44, 45, 46]

            for order in hierarchy_order:
                if order != "waist":
                    quaternion_data = data_dict[order][4:]
                else:
                    pass

                roll_x_deg, pitch_y_deg, yaw_z_deg = quaternion_to_euler(quaternion_data[0], quaternion_data[1], quaternion_data[2], quaternion_data[3])
                rot_x = format(roll_x_deg, '.6f')
                rot_y = format(pitch_y_deg, '.6f')
                rot_z = format(yaw_z_deg, '.6f')

                if order == 0:
                    file.write(f" {rot_x}")
                    file.write(f" {rot_z}")
                    file.write(f" {rot_y}")

                elif order == "waist":
                    spine_data = data_dict[1][4:]
                    spine_x, spine_y, spine_z = quaternion_to_euler(spine_data[0], spine_data[1], spine_data[2], spine_data[3])

                    spine1_data = data_dict[2][4:]
                    spine1_x, spine1_y, spine1_z = quaternion_to_euler(spine1_data[0], spine1_data[1], spine1_data[2], spine1_data[3])

                    rot_x = spine_x + (spine1_x *0.5)
                    rot_y = spine_y + (spine1_y *0.5)
                    rot_z = spine_z + (spine1_z *0.5)

                    file.write(f" {rot_x}")
                    file.write(f" {rot_y}")
                    file.write(f" {rot_z}")

                elif order == 3:
                    spine_data = data_dict[2][4:]
                    spine_x, spine_y, spine_z = quaternion_to_euler(spine_data[0], spine_data[1], spine_data[2], spine_data[3])

                    rot_x = float(rot_x) + spine_x
                    rot_y = float(rot_y) + spine_y
                    rot_z = float(rot_z) + spine_z

                    file.write(f" {rot_x}")
                    file.write(f" {rot_y}")
                    file.write(f" {rot_z}")

                elif order == 25:
                    rshoulder_data = data_dict[24][4:]
                    rshoulder_x, rshoulder_y, rshoulder_z = quaternion_to_euler(rshoulder_data[0], rshoulder_data[1], rshoulder_data[2], rshoulder_data[3])
                    spine_data = data_dict[2][4:]
                    spine_x, spine_y, spine_z = quaternion_to_euler(spine_data[0], spine_data[1], spine_data[2], spine_data[3])

                    rot_x = float(rot_x) + rshoulder_x + spine_x
                    rot_y = float(rot_y) + rshoulder_y + spine_y
                    rot_z = float(rot_z) + rshoulder_z + spine_z

                    file.write(f" {rot_x}")
                    file.write(f" {rot_y}")
                    file.write(f" {rot_z}")

                elif order == 6:
                    lshoulder_data = data_dict[5][4:]
                    lshoulder_x, lshoulder_y, lshoulder_z = quaternion_to_euler(lshoulder_data[0], lshoulder_data[1], lshoulder_data[2], lshoulder_data[3])
                    spine_data = data_dict[2][4:]
                    spine_x, spine_y, spine_z = quaternion_to_euler(spine_data[0], spine_data[1], spine_data[2], spine_data[3])

                    rot_x = float(rot_x) + lshoulder_x + spine_x
                    rot_y = float(rot_y) + lshoulder_y + spine_y
                    rot_z = float(rot_z) + lshoulder_z + spine_z

                    file.write(f" {rot_x}")
                    file.write(f" {rot_y}")
                    file.write(f" {rot_z}")

                else:
                    file.write(f" {rot_x}")
                    file.write(f" {rot_y}")
                    file.write(f" {rot_z}")
            file.write("\n")

    else:
        with open(file_path, 'w') as file:
            file.write(''.join(map(str, optitrack_etri_hierarchy())))
            # file.write(''.join(map(str, hirerarchy.optitrack_hierarchy())))
            file.write(f"Frames:\n")
            file.write(f"Frame Time: {float(0.008333)}\n")

# Optitrack_BVH ================================

def make_file_path(click_timestamp, sensor):
    convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S.%f")
    timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
    dir_time = convert_timestamp.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
    # dir_path = r"C:\Users\ADMIN\Desktop\soo\git\HIL-code\Examples\dataset\ETRI SoftSuit dataset\{}".format(dir_time)
    dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
    os.makedirs(dir_path, exist_ok=True)
    
    file_path = r"{}\{}_BVH_{}.bvh".format(dir_path, sensor, timestamp_set)
    
    return file_path

    