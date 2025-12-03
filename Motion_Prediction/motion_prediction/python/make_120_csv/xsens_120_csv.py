import os.path
import struct
import numpy as np
import socket
import pandas as pd
from datetime import datetime

import queue
import csv


Xsens_column = np.dtype([
    ('segment_id', 'U2'),
    ('sensor_free_acceleration_x', 'f4'),
    ('sensor_free_acceleration_y', 'f4'),
    ('sensor_free_acceleration_z', 'f4'),
    ('sensor_magnetic_field_x', 'f4'),
    ('sensor_magnetic_field_y', 'f4'),
    ('sensor_magnetic_field_z', 'f4'),
    ('sensor_position_q0_quaternion', 'f4'),
    ('sensor_position_q1_quaternion', 'f4'),
    ('sensor_position_q2_quaternion', 'f4'),
    ('sensor_position_q3_quaternion', 'f4'),
    ('segment_position_q0_quaternion', 'f4'),
    ('segment_position_q1_quaternion', 'f4'),
    ('segment_position_q2_quaternion', 'f4'),
    ('segment_position_q3_quaternion', 'f4'),
    ('segment_position_x_euler', 'f4'),
    ('segment_position_y_euler', 'f4'),
    ('segment_position_z_euler', 'f4'),
    ('segment_position_x', 'f4'),
    ('segment_position_y', 'f4'),
    ('segment_position_z', 'f4'),
    ('segment_velocity_x', 'f4'),
    ('segment_velocity_y', 'f4'),
    ('segment_velocity_z', 'f4'),
    ('segment_acceleration_x', 'f4'),
    ('segment_acceleration_y', 'f4'),
    ('segment_acceleration_z', 'f4'),
    ('segment_angular_velocity_x', 'f4'),
    ('segment_angular_velocity_y', 'f4'),
    ('segment_angular_velocity_z', 'f4'),
    ('segment_angular_acceleration_x', 'f4'),
    ('segment_angular_acceleration_y', 'f4'),
    ('segment_angular_acceleration_z', 'f4'),
    ('segment_scale_x', 'f4'),
    ('segment_scale_y', 'f4'),
    ('segment_scale_z', 'f4'),
])
        
class xsens_120_csv:    
    def make_csv(self, data, timestamp, click_timestamp):
        sensor_data = data['sensor']
        segment_data = data['segment']
        convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")
        xsens_data = []
        count_id = 0
        for i in range(len(sensor_data)):
            sensor_name=self.sensor_name_parser(count_id)
            xsens_data.append(sensor_name)
            for s in range(1, len(sensor_data[i])):
                xsens_data.append(sensor_data[i][s])
                
            for g in range(1, len(segment_data[i])):
                xsens_data.append(segment_data[i][g])
            count_id += 1
        
        df = pd.DataFrame(xsens_data)
        df_flat = pd.DataFrame([df.values.flatten()])
        df_flat.columns = Xsens_column.names * count_id
         
        dir_time = convert_timestamp.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
        dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
        os.makedirs(dir_path, exist_ok=True)
        
        timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
        df_timestamp = timestamp
        ts_df = pd.DataFrame([df_timestamp], columns=["Timestamp"])
        output_df = pd.concat([ts_df, df_flat], axis=1)
        
        self.file_name = r"{}/MVNLink_{}.csv".format(dir_path, timestamp_set)
        #self.file_name = r"C:\Users\ADMIN\Desktop\soo\git\HIL\Examples\test.csv"
        if not self.file_exist():
            output_df.to_csv(self.file_name, mode='a', index=False, header=True)
        else:
            output_df.to_csv(self.file_name, mode='a', index=False, header=False)
            
    def make_csv_v2(self, data, timestamp, click_timestamp):
        sensor_data = data['sensor']
        segment_data = data['segment']
        convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")
        xsens_data = []
        count_id = 0
        for i in range(len(sensor_data)):
            sensor_name = self.sensor_name_parser(count_id)
            xsens_data.append(sensor_name)
            for s in range(1, len(sensor_data[i])):
                xsens_data.append(sensor_data[i][s])
                    
            for g in range(1, len(segment_data[i])):
                xsens_data.append(segment_data[i][g])
            count_id += 1

        xsens_array = np.array(xsens_data).reshape((1, -1))
        
        dir_time = convert_timestamp.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
        dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
        os.makedirs(dir_path, exist_ok=True)
            
        timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
        self.file_name = r"{}/MVNLink_{}.csv".format(dir_path, timestamp_set)
        
        if not self.file_exist():
            with open(self.file_name, 'w') as f:
                # 컬럼 이름을 파일에 작성
                column_names = ['Timestamp'] + list(Xsens_column.names * count_id)
                np.savetxt(f, [column_names], delimiter=',', fmt='%s')
                # 데이터와 타임스탬프를 파일에 작성
                data_row = [timestamp] + xsens_data
                np.savetxt(f, [data_row], delimiter=',', fmt='%s')
        else:
            with open(self.file_name, 'ab') as f:
                data_row = [timestamp] + xsens_data
                np.savetxt(f, [data_row], delimiter=',', fmt='%s')
    
    def make_csv_v3(self, result_list, click_timestamp):
        
        while len(result_list) > 0:
            data = result_list[0]['xsens']
            timestamp = struct.unpack('!32s', result_list[0]["TS"])[0].decode()
            result_list.pop(0)
            
            sensor_data = data['sensor']
            segment_data = data['segment']
            convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")
            xsens_data = []
            count_id = 0
            for i in range(len(sensor_data)):
                sensor_name = self.sensor_name_parser(count_id)
                xsens_data.append(sensor_name)
                for s in range(1, len(sensor_data[i])):
                    xsens_data.append(sensor_data[i][s])
                        
                for g in range(1, len(segment_data[i])):
                    xsens_data.append(segment_data[i][g])
                count_id += 1

            xsens_array = np.array(xsens_data).reshape((1, -1))
            
            dir_time = convert_timestamp.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
            dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
            os.makedirs(dir_path, exist_ok=True)
                
            timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
            self.file_name = r"{}/MVNLink_{}.csv".format(dir_path, timestamp_set)
            
            if not self.file_exist():
                with open(self.file_name, 'w') as f:
                    # 컬럼 이름을 파일에 작성
                    column_names = ['Timestamp'] + list(Xsens_column.names * count_id)
                    np.savetxt(f, [column_names], delimiter=',', fmt='%s')
                    # 데이터와 타임스탬프를 파일에 작성
                    data_row = [timestamp] + xsens_data
                    np.savetxt(f, [data_row], delimiter=',', fmt='%s')
            else:
                with open(self.file_name, 'ab') as f:
                    data_row = [timestamp] + xsens_data
                    np.savetxt(f, [data_row], delimiter=',', fmt='%s')

        print("[CSV] Xsens_csv Done : ", datetime.now())
        
    def make_csv_v4(self, xsens_queue, click_timestamp):
        while True :
            queue_data = xsens_queue.get()
            
            if queue_data == True:
                break
            
            if 'xsens' in queue_data:
                data = queue_data['xsens']
                data_TS = queue_data['TS']
                
                timestamp = struct.unpack('!32s', data_TS)[0].decode()
                
                sensor_data = data['sensor']
                segment_data = data['segment']
                if len(data['scale']) != 0 and len(data['scale'][0]) != 0:
                    scale_data = data['scale']
                else:
                    scale_arry = []
                    for i in range(23):
                        scale_arry.append(["","","",""])
                    scale_data = scale_arry
                convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")
                xsens_data = []
                count_id = 0
                for i in range(len(sensor_data)):
                    sensor_name = self.sensor_name_parser(count_id)
                    xsens_data.append(sensor_name)
                    #sensor
                    for s in range(1, len(sensor_data[i])):
                        xsens_data.append(sensor_data[i][s])
                        
                    #segment 
                    for g in range(1, len(segment_data[i])):
                        xsens_data.append(segment_data[i][g])
                        
                    #scale
                    for si in range(1, len(scale_data[i])):
                        xsens_data.append(scale_data[i][si])
                    count_id += 1

                xsens_array = np.array(xsens_data).reshape((1, -1))
                
                dir_time = convert_timestamp.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
                dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
                os.makedirs(dir_path, exist_ok=True)
                    
                timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
                self.file_name = r"{}/MVNLink_{}.csv".format(dir_path, timestamp_set)
                
                if not self.file_exist():
                    with open(self.file_name, 'w') as f:
                        # 컬럼 이름을 파일에 작성
                        column_names = ['Timestamp'] + list(Xsens_column.names * count_id)
                        np.savetxt(f, [column_names], delimiter=',', fmt='%s')
                        # 데이터와 타임스탬프를 파일에 작성
                        data_row = [timestamp] + xsens_data
                        np.savetxt(f, [data_row], delimiter=',', fmt='%s')
                else:
                    with open(self.file_name, 'ab') as f:
                        data_row = [timestamp] + xsens_data
                        np.savetxt(f, [data_row], delimiter=',', fmt='%s')

        print("[CSV] xsens CSV Done : ", datetime.now())
        
    def sensor_name_parser(self, num):
        
        segment = ["Pelvis", "L5", "L3", "T12", "T8", "Neck", "Head", "RightShoulder", "RightupperArm", "RightFoream",
                   "RightHand", "LeftShoulder", "LeftUpper Arm", "LeftForearm", "LeftHand", "RightUpperLeg",
                   "RightLowerLeg", "RightFoot", "RightToe","LeftUpperLeg",
                   "LeftLowerLeg", "LeftFoot", "LeftToe"]

        return segment[num]
    
    def file_exist(self):
        # path = r"C:\Users\ADMIN\Desktop\soo\git\HIL-code\Examples\dataset\ETRI SoftSuit dataset\%s" % self.file_name
        path = self.file_name
        self.isExist = os.path.exists(path)

        return self.isExist