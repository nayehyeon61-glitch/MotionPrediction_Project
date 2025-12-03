import os.path
import struct
from datetime import datetime
import math
from scipy.spatial.transform import Rotation

from make_120_bvh.hierarchy import xsens_etri_hierarchy
import pdb

import luamodule as lua  # see luamodule.py
import numpy as np
m=lua.taesooLib()

class XsensBVH:
    def __init__(self):
        self.file_name = ""

    # returns taesooLib format (새로 구현)
    def make_posedof(self, xsens_queue, loader):
        queue_data = xsens_queue.get()
        
        if 'xsens' in queue_data:
            data = queue_data['xsens']
            
            etri_hirerarchy_order = [0,2,5,6,8,9,10,12,13,14,15,16,17,18,19,20,21,22]
            
            segment_data = data['segment']
                    
            for idx, order in enumerate(etri_hirerarchy_order):
                segment_quaternion = segment_data[order][1:5]

                q=m.quater()
                q.ref()[:]=segment_quaternion
                q.x,q.y,q.z=q.y, q.z, q.x # I don't know why this is necessary-.- there is definitely a bug in the segment_quaternion
                # 정상적이라면, 위 줄은 좌표계를 바꾸는거라 위치도 동일하게 바꿔야 일관성이 있어야하는데 ...

                T= loader.getBoneByRotJointIndex(idx).getFrame()
                T.rotation.assign(q)
                T.translation.ref()[:]=segment_data[order][8:11]
            pose=m.vectorn()
            loader.fkSolver().getPoseDOFfromGlobal(pose)
            return pose

    # 이 버젼은 원래 있던 코드를 재사용했는데 동작은 하지만 포즈가 약간 이상함.
    def make_posedof_old(self, xsens_queue, loader):
        queue_data = xsens_queue.get()
        
        if 'xsens' in queue_data:
            data = queue_data['xsens']
            data_ts = queue_data['TS']
            
            xsens_data = []
            count_id = 0
            etri_hirerarchy_order = [0,2,5,6,8,9,10,12,13,14,15,16,17,18,19,20,21,22]
            
            hirerachy_order_body = [0,1,2,3,4,5,6]
            hirerachy_order_r_shoulder = [0,1,2,3,4,7,8,9,10]
            hirerachy_order_l_shoulder = [0,1,2,3,4,11,12,13,14]
            hirerachy_order_r_hip = [0,15,16,17,18]
            hirerachy_order_l_hip = [0,19,20,21,22]
            
            segment_data = data['segment']
            
            pos_x = segment_data[0][8]
            pos_y = segment_data[0][9]
            pos_z = segment_data[0][10]
            
            out=m.vectorn(loader.dofInfo.numDOF())
            out.ref()[0:3]=segment_data[0][8:11]
            channel=3

                    
            for idx, order in enumerate(etri_hirerarchy_order):
                segment_quaternion = segment_data[order][1:5]

                roll_x, pitch_y, yaw_z = self.quaternion_to_euler_xsens(segment_quaternion, 'ZXY')
                
                if order == 0:
                    rot_x, rot_y, rot_z = self.quaternion_to_euler_xsens(segment_data[0][1:5], 'ZYX')
                
                elif order == 5:
                    chest4_data = segment_data[4][1:5]
                    chest4_x, chest4_y, chest4_z = self.quaternion_to_euler_xsens(chest4_data, 'xyz')
                    
                    neck_data = segment_data[5][1:5]
                    neck_x, neck_y, neck_z = self.quaternion_to_euler_xsens(neck_data, 'xyz')
                    
                    rot_x = neck_x - chest4_x
                    rot_y = neck_y - chest4_y
                    rot_z = neck_z - chest4_z
                
                elif order == 8:
                    rshoulder_data = segment_data[8][1:5]
                    rshoulder_x, rshoulder_y, rshoulder_z = self.quaternion_to_euler_xsens(rshoulder_data, 'xyz')

                    chest1_data = segment_data[1][1:5]
                    chest1_x, chest1_y, chest1_z = self.quaternion_to_euler_xsens(chest1_data, 'xyz')
                    
                    rot_x = rshoulder_x - chest1_x
                    rot_y = -(rshoulder_y - chest1_y)
                    rot_z = -(rshoulder_z - chest1_z)
                
                elif order == 12:
                    
                    lshoulder_data = segment_data[12][1:5]
                    lshoulder_x, lshoulder_y, lshoulder_z = self.quaternion_to_euler_xsens(lshoulder_data, 'xyz')
                    
                    chest1_data = segment_data[1][1:5]
                    chest1_x, chest1_y, chest1_z = self.quaternion_to_euler_xsens(chest1_data, 'xyz')
                    
                    rot_x = lshoulder_x - chest1_x
                    rot_y = lshoulder_y - chest1_y
                    rot_z = lshoulder_z - chest1_z
                
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
                                before_data = segment_data[index][1:5]
                                before_x, before_y, before_z = self.quaternion_to_euler_xsens(before_data, 'ZXY')
                                
                                rot_x = format(roll_x - before_x,'.6f')
                                rot_y = format(pitch_y- before_y,'.6f')
                                rot_z = format(yaw_z - before_z,'.6f')
                                
                            elif 1 <= order <= 14:
                                index = list_sort[count-1]
                                before_data = segment_data[index][1:5]
                                before_x, before_y, before_z = self.quaternion_to_euler_xsens(before_data, 'ZXY')
                                
                                order_data = segment_data[index+1][1:5]
                                order_x, order_y, order_z = self.quaternion_to_euler_xsens(order_data, 'ZXY')
                            
                                # R elbow, R_wrist
                                if 9 <= order <= 10:
                                    index = list_sort[count-1]
                                    before_data = segment_data[index][1:5]
                                    before_x, before_y, before_z = self.quaternion_to_euler_xsens(before_data, 'xyz')
                                    
                                    order_data = segment_data[index+1][1:5]
                                    order_x, order_y, order_z = self.quaternion_to_euler_xsens(order_data, 'xyz')
                                    
                                    rot_x = order_x - before_x
                                    rot_y = -(order_y - before_y)
                                    rot_z = order_z - before_z
                                        
                                elif 13 <= order <= 14:
                                    index = list_sort[count-1]
                                    before_data = segment_data[index][1:5]
                                    before_x, before_y, before_z = self.quaternion_to_euler_xsens(before_data, 'xyz')
                                    
                                    order_data = segment_data[index+1][1:5]
                                    order_x, order_y, order_z = self.quaternion_to_euler_xsens(order_data, 'xyz')
                                    
                                    rot_x = order_x - before_x
                                    rot_y = order_y - before_y
                                    rot_z = order_z - before_z
                                    
                                else:
                                    rot_x = order_x - before_x
                                    rot_y = order_y - before_y
                                    rot_z = order_z - before_z

                            else:
                                index = list_sort[count-1]
                                before_data = segment_data[index][1:5]
                                before_x, before_y, before_z = self.quaternion_to_euler_xsens(before_data, 'ZXY')
                                
                                rot_x = roll_x - before_x
                                rot_y = pitch_y - before_y
                                rot_z = yaw_z - before_z
                        
                            break
                        else:
                            count += 1
                
                f_rot_x = self.radian_to_degree(float(rot_x))
                f_rot_y = self.radian_to_degree(float(rot_y))
                f_rot_z = self.radian_to_degree(float(rot_z))
                
                
                if order == 0:
                    out.set(channel, f_rot_z) 
                    channel+=1
                    out.set(channel, f_rot_x)
                    channel+=1
                    out.set(channel, f_rot_y)
                    channel+=1
                
                elif 1 <= order <= 14:
                    out.set(channel, f_rot_x)
                    channel+=1
                    out.set(channel, f_rot_z)
                    channel+=1
                    out.set(channel, f_rot_y)
                    channel+=1
                
                else:
                    out.set(channel, f_rot_y)
                    channel+=1
                    out.set(channel, f_rot_z)
                    channel+=1
                    out.set(channel, f_rot_x)
                    channel+=1

            assert(channel== out.size()-1)
            # bvhpose to taesoolib posedof
            out.ref()[4:]=out.ref()[3:-1]*(math.pi/180) # to radian
            q=m.quater()
            q.setRotation(loader.bone(1).getRotationalChannels(), out.toVector3(4))
            #print(out.toVector3(4), q)
            out.setQuater(3, q)  # euler to quaternion for the root joint
            # again, the above code has definitely a bug.  the output quaternion is different from the input quaternion. This xsens_120_bvh.py has too many bugs. -.- 

            return out
                           
    def make_bvh(self, xsens_queue, click_timestamp):
        queue_data = xsens_queue.get()
        
        if False and queue_data:
            self.bvh_frame_alter(self.file_name)
            print("[bvh] xsens bvh Done : ", datetime.now())
        
        if 'xsens' in queue_data:
            data = queue_data['xsens']
            data_ts = queue_data['TS']
            
            timestamp = struct.unpack('!32s', data_ts)[0].decode()
            convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")
            dir_time = convert_timestamp.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
            dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
            os.makedirs(dir_path, exist_ok=True)
            
            timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
            self.file_name = r"{}/MVNLink_{}.bvh".format(dir_path, timestamp_set)
            
            xsens_data = []
            count_id = 0
            etri_hirerarchy_order = [0,2,5,6,8,9,10,12,13,14,15,16,17,18,19,20,21,22]
            
            hirerachy_order_body = [0,1,2,3,4,5,6]
            hirerachy_order_r_shoulder = [0,1,2,3,4,7,8,9,10]
            hirerachy_order_l_shoulder = [0,1,2,3,4,11,12,13,14]
            hirerachy_order_r_hip = [0,15,16,17,18]
            hirerachy_order_l_hip = [0,19,20,21,22]
            
            segment_data = data['segment']
            
            pos_x = format(segment_data[0][8], '.6f')
            pos_y = format(segment_data[0][9], '.6f')
            pos_z = format(segment_data[0][10], '.6f')
            
            if not self.file_exist():
                with open(self.file_name, 'w') as f:
                    f.write(''.join(map(str, xsens_etri_hierarchy())))
                    f.write(f"Frames:\n")
                    f.write(f"Frame Time: {float(0.008333)}\n")
            else:
                with open(self.file_name, 'a') as f:
                    f.write(f"{pos_x} {pos_y} {pos_z}")
                    
                    for idx, order in enumerate(etri_hirerarchy_order):
                        segment_quaternion = segment_data[order][1:5]

                        roll_x, pitch_y, yaw_z = self.quaternion_to_euler_xsens(segment_quaternion, 'ZXY')
                        
                        if order == 0:
                            rot_x, rot_y, rot_z = self.quaternion_to_euler_xsens(segment_data[0][1:5], 'ZYX')
                        
                        elif order == 5:
                            chest4_data = segment_data[4][1:5]
                            chest4_x, chest4_y, chest4_z = self.quaternion_to_euler_xsens(chest4_data, 'xyz')
                            
                            neck_data = segment_data[5][1:5]
                            neck_x, neck_y, neck_z = self.quaternion_to_euler_xsens(neck_data, 'xyz')
                            
                            rot_x = neck_x - chest4_x
                            rot_y = neck_y - chest4_y
                            rot_z = neck_z - chest4_z
                        
                        elif order == 8:
                            rshoulder_data = segment_data[8][1:5]
                            rshoulder_x, rshoulder_y, rshoulder_z = self.quaternion_to_euler_xsens(rshoulder_data, 'xyz')

                            chest1_data = segment_data[1][1:5]
                            chest1_x, chest1_y, chest1_z = self.quaternion_to_euler_xsens(chest1_data, 'xyz')
                            
                            rot_x = rshoulder_x - chest1_x
                            rot_y = -(rshoulder_y - chest1_y)
                            rot_z = -(rshoulder_z - chest1_z)
                        
                        elif order == 12:
                            
                            lshoulder_data = segment_data[12][1:5]
                            lshoulder_x, lshoulder_y, lshoulder_z = self.quaternion_to_euler_xsens(lshoulder_data, 'xyz')
                            
                            chest1_data = segment_data[1][1:5]
                            chest1_x, chest1_y, chest1_z = self.quaternion_to_euler_xsens(chest1_data, 'xyz')
                            
                            rot_x = lshoulder_x - chest1_x
                            rot_y = lshoulder_y - chest1_y
                            rot_z = lshoulder_z - chest1_z
                        
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
                                        before_data = segment_data[index][1:5]
                                        before_x, before_y, before_z = self.quaternion_to_euler_xsens(before_data, 'ZXY')
                                        
                                        rot_x = format(roll_x - before_x,'.6f')
                                        rot_y = format(pitch_y- before_y,'.6f')
                                        rot_z = format(yaw_z - before_z,'.6f')
                                        
                                    elif 1 <= order <= 14:
                                        index = list_sort[count-1]
                                        before_data = segment_data[index][1:5]
                                        before_x, before_y, before_z = self.quaternion_to_euler_xsens(before_data, 'ZXY')
                                        
                                        order_data = segment_data[index+1][1:5]
                                        order_x, order_y, order_z = self.quaternion_to_euler_xsens(order_data, 'ZXY')
                                    
                                        # R elbow, R_wrist
                                        if 9 <= order <= 10:
                                            index = list_sort[count-1]
                                            before_data = segment_data[index][1:5]
                                            before_x, before_y, before_z = self.quaternion_to_euler_xsens(before_data, 'xyz')
                                            
                                            order_data = segment_data[index+1][1:5]
                                            order_x, order_y, order_z = self.quaternion_to_euler_xsens(order_data, 'xyz')
                                            
                                            rot_x = order_x - before_x
                                            rot_y = -(order_y - before_y)
                                            rot_z = order_z - before_z
                                                
                                        elif 13 <= order <= 14:
                                            index = list_sort[count-1]
                                            before_data = segment_data[index][1:5]
                                            before_x, before_y, before_z = self.quaternion_to_euler_xsens(before_data, 'xyz')
                                            
                                            order_data = segment_data[index+1][1:5]
                                            order_x, order_y, order_z = self.quaternion_to_euler_xsens(order_data, 'xyz')
                                            
                                            rot_x = order_x - before_x
                                            rot_y = order_y - before_y
                                            rot_z = order_z - before_z
                                            
                                        else:
                                            rot_x = order_x - before_x
                                            rot_y = order_y - before_y
                                            rot_z = order_z - before_z

                                    else:
                                        index = list_sort[count-1]
                                        before_data = segment_data[index][1:5]
                                        before_x, before_y, before_z = self.quaternion_to_euler_xsens(before_data, 'ZXY')
                                        
                                        rot_x = roll_x - before_x
                                        rot_y = pitch_y - before_y
                                        rot_z = yaw_z - before_z
                                
                                    break
                                else:
                                    count += 1
                        
                        f_rot_x = self.radian_to_degree(float(rot_x))
                        f_rot_y = self.radian_to_degree(float(rot_y))
                        f_rot_z = self.radian_to_degree(float(rot_z))
                        
                        d_rot_x = format(f_rot_x, '.6f')
                        d_rot_y = format(f_rot_y, '.6f')
                        d_rot_z = format(f_rot_z, '.6f')
                        
                        if order == 0:
                            f.write(f" {d_rot_z}")
                            f.write(f" {d_rot_x}")
                            f.write(f" {d_rot_y}")
                        
                        elif 1 <= order <= 14:
                            f.write(f" {d_rot_x}")
                            f.write(f" {d_rot_z}")
                            f.write(f" {d_rot_y}")
                        
                        else:
                            f.write(f" {d_rot_y}")
                            f.write(f" {d_rot_z}")
                            f.write(f" {d_rot_x}")
                    
                    f.write("\n")
                           
    def file_exist(self):
        path = self.file_name
        is_exist = os.path.exists(path)

        return is_exist
    
    @staticmethod
    def quaternion_to_euler_xsens(q, rotation):
        [w,x,y,z] = q
        # 주어진 쿼터니언 값
        quaternion = [x, y, z, w]

        # 쿼터니언을 회전 행렬로 변환
        rotation_matrix = Rotation.from_quat(quaternion).as_matrix()

        # 회전 행렬을 사용하여 오일러 각 계산
        euler_angles = Rotation.from_matrix(rotation_matrix).as_euler(rotation, degrees=False)
        
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
                #lines[309] = f"Frames: {len(lines) - 311}\n"
                # ETRI_hireracrchy
                lines[112] = f"Frames: {len(lines) - 114}\n"
                lines[113] = f"Frame Time: {float(0.008333)}\n"
                with open(file_path, 'w') as bvh_file:
                    bvh_file.writelines(lines)
