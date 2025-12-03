import os.path
import struct
import numpy as np
import socket
import pandas as pd
from datetime import datetime

import queue
import csv


FMGMMG_data_struct = np.dtype([
    #("sensor_data_ID", 'i'),
    ('sensor_location_ID', 'i'),
    ('Ch1', 'f4'),
    ('Ch2', 'f4'),
    ('Ch3', 'f4'),
    ('Ch4', 'f4'),
    ('Ch5', 'f4'),
    ('Ch6', 'f4'),
    ('Ch7', 'f4'),
    ('Ch8', 'f4'),
    ('Ch9', 'f4'),
    ('Ch10', 'f4'),
    ('Ch11', 'f4'),
    ('Ch12', 'f4'),
    ('Ch13', 'f4'),
    ('Ch14', 'f4'),
    ('Ch15', 'f4'),
    ('Ch16', 'f4')
])
        
class FMGMMG_120_csv:    
    def make_csv(self, data, timestamp, click_timestamp):
        convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")
        key_list = []
        value_list = []
        
        for key, value in data.items():
            for i in range(len(value)):
                value_list.append(value[i])
        
        df = pd.DataFrame(value_list)
        df_flat = pd.DataFrame([df.values.flatten()])
        df_flat.columns = FMGMMG_data_struct.names
        
        dir_time = convert_timestamp.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
        dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
        os.makedirs(dir_path, exist_ok=True)
        
        timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
        self.file_name = r"{}/FMGMMG_{}.csv".format(dir_path, timestamp_set)
        ts_df = pd.DataFrame([timestamp], columns=["Timestamp"])
        output_df = pd.concat([ts_df, df_flat], axis=1)
        if not self.file_exist():
            output_df.to_csv(self.file_name, mode='a', index=False, header=True)
        else:
            output_df.to_csv(self.file_name, mode='a', index=False, header=False)
            
    def make_csv_v2(self, result_list, click_timestamp):
        while len(result_list) > 0:
            data = result_list[0]['fmgmmg']
            timestamp = struct.unpack('!32s', result_list[0]["TS"])[0].decode()
            result_list.pop(0)
            
            convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")
            key_list = []
            value_list = []
            
            for key, value in data.items():
                for i in range(len(value)):
                    value_list.append(value[i])
            
            df = pd.DataFrame(value_list)
            df_flat = pd.DataFrame([df.values.flatten()])
            df_flat.columns = FMGMMG_data_struct.names
            
            dir_time = convert_timestamp.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
            dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
            os.makedirs(dir_path, exist_ok=True)
            
            timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
            self.file_name = r"{}/ETRI_FMGMMG_{}.csv".format(dir_path, timestamp_set)
            ts_df = pd.DataFrame([timestamp], columns=["Timestamp"])
            output_df = pd.concat([ts_df, df_flat], axis=1)
            if not self.file_exist():
                output_df.to_csv(self.file_name, mode='a', index=False, header=True)
            else:
                output_df.to_csv(self.file_name, mode='a', index=False, header=False)
        print("fmgmmg CSV Done : ", datetime.now())
        
    def make_csv_v3(self, FMGMMG_queue, click_timestamp):
        while True :
            queue_data = FMGMMG_queue.get()
            
            if queue_data == True:
                break
            
            if 'fmgmmg' in queue_data:
                data = queue_data['fmgmmg']
                data_TS = queue_data['TS']
                timestamp = struct.unpack('!32s', data_TS)[0].decode()
                
                
                convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")
                key_list = []
                value_list = []
                
                for key, value in data.items():
                    for i in range(len(value)):
                        value_list.append(value[i])
                
                df = pd.DataFrame(value_list)
                df_flat = pd.DataFrame([df.values.flatten()])
                df_flat.columns = FMGMMG_data_struct.names
                
                dir_time = convert_timestamp.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
                dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
                os.makedirs(dir_path, exist_ok=True)
                
                timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
                self.file_name = r"{}/ETRI_FMGMMG_{}.csv".format(dir_path, timestamp_set)
                ts_df = pd.DataFrame([timestamp], columns=["Timestamp"])
                output_df = pd.concat([ts_df, df_flat], axis=1)
                if not self.file_exist():
                    output_df.to_csv(self.file_name, mode='a', index=False, header=True)
                else:
                    output_df.to_csv(self.file_name, mode='a', index=False, header=False)
        print("[CSV] fmgmmg CSV Done : ", datetime.now())
        
    def file_exist(self):
        # path = r"C:\Users\ADMIN\Desktop\soo\git\HIL-code\Examples\dataset\ETRI SoftSuit dataset\%s" % self.file_name

        path = self.file_name
        self.isExist = os.path.exists(path)

        return self.isExist