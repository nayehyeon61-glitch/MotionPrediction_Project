import os.path
import struct
import numpy as np
import socket
import pandas as pd
from datetime import datetime

import queue
import csv


futek_column = np.dtype([
    ('sensor1', 'U2')
])
        
class futek_120_csv:    
    def make_csv(self, data, timestamp, click_timestamp):
        convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")
        futek_arry = []
        sensor_data = data[0][1]
        
        futek_arry.append(sensor_data)
        df = pd.DataFrame(futek_arry)
        df_flat = pd.DataFrame([df.values.flatten()])
        df_flat.columns = futek_column.names
        
        dir_time = convert_timestamp.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
        dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
        os.makedirs(dir_path, exist_ok=True)
        
        timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
        self.file_name = r"{}/FutekLSB205_{}.csv".format(dir_path, timestamp_set)
        ts_df = pd.DataFrame([timestamp], columns=["Timestamp"])
        output_df = pd.concat([ts_df, df_flat], axis=1)
        if not self.file_exist():
            output_df.to_csv(self.file_name, mode='a', index=False, header=True)
        else:
            output_df.to_csv(self.file_name, mode='a', index=False, header=False)
            
    def make_csv_v2(self, result_list, click_timestamp):
        while len(result_list) > 0:
            data = result_list[0]['futek']
            timestamp = struct.unpack('!32s', result_list[0]["TS"])[0].decode()
            result_list.pop(0)
            
            convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")
            futek_arry = []
            sensor_data = data[0][1]
            
            futek_arry.append(sensor_data)
            df = pd.DataFrame(futek_arry)
            df_flat = pd.DataFrame([df.values.flatten()])
            df_flat.columns = futek_column.names
            
            dir_time = convert_timestamp.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
            dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
            os.makedirs(dir_path, exist_ok=True)
            
            timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
            self.file_name = r"{}/FutekLSB205_{}.csv".format(dir_path, timestamp_set)
            ts_df = pd.DataFrame([timestamp], columns=["Timestamp"])
            output_df = pd.concat([ts_df, df_flat], axis=1)
            if not self.file_exist():
                output_df.to_csv(self.file_name, mode='a', index=False, header=True)
            else:
                output_df.to_csv(self.file_name, mode='a', index=False, header=False)
        print("futek CSV Done : ", datetime.now()) 
        
    def make_csv_v3(self, futek_queue, click_timestamp):
        while True:
            queue_data = futek_queue.get()
            
            if queue_data == True:
                break
            
            if 'futek' in queue_data:
                data = queue_data['futek']
                data_TS = queue_data['TS']
                
            
                timestamp = struct.unpack('!32s', data_TS)[0].decode()
                
                
                convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")
                futek_arry = []
                sensor_data = data[0][1]
                
                futek_arry.append(sensor_data)
                df = pd.DataFrame(futek_arry)
                df_flat = pd.DataFrame([df.values.flatten()])
                df_flat.columns = futek_column.names
                
                dir_time = convert_timestamp.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
                dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
                os.makedirs(dir_path, exist_ok=True)
                
                timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
                self.file_name = r"{}/FutekLSB205_{}.csv".format(dir_path, timestamp_set)
                ts_df = pd.DataFrame([timestamp], columns=["Timestamp"])
                output_df = pd.concat([ts_df, df_flat], axis=1)
                if not self.file_exist():
                    output_df.to_csv(self.file_name, mode='a', index=False, header=True)
                else:
                    output_df.to_csv(self.file_name, mode='a', index=False, header=False)
        print("[CSV] futek CSV Done : ", datetime.now()) 
        
    def file_exist(self):
        # path = r"C:\Users\ADMIN\Desktop\soo\git\HIL-code\Examples\dataset\ETRI SoftSuit dataset\%s" % self.file_name

        path = self.file_name
        self.isExist = os.path.exists(path)

        return self.isExist
    
    def test_make_csv(self, data, timestamp, click_timestamp):
        
        bulk_insert_data = {
            'TS': [],
            'x': []
            #'y': []
        }
    
        for i, TS in enumerate(timestamp):
            value = data[i][0][1]
            if value is not None:
                bulk_insert_data["TS"].append(TS)
                bulk_insert_data["x"].append(value)
            #    elif i == 1:
            #        initial_data["y"].append(value)
                   
        new_df = pd.DataFrame(bulk_insert_data)

        # 파일을 열린 상태로 유지한 채로 데이터 추가
        new_df.to_csv(r"C:\Users\ADMIN\Desktop\soo\test.csv", mode='w', index=False, header=True)