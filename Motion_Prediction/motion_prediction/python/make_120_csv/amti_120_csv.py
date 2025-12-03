import os.path
import struct
import numpy as np
import socket
import pandas as pd
from datetime import datetime

import queue
import csv


amti_column = np.dtype([
    ('Fx', 'f4'),
    ('Fy', 'f4'),
    ('Fz', 'f4'),
    ('Mx', 'f4'),
    ('My', 'f4'),
    ('Mz', 'f4')
])
        
class amti_120_csv:    
    def make_csv(self, data, timestamp, click_timestamp):
        convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")
        amti_arry = []
        
        for i in range(1, len(data[0])):
            amti_arry.append(data[0][i])
        
        df = pd.DataFrame(amti_arry)
        df_flat = pd.DataFrame([df.values.flatten()])
        df_flat.columns = amti_column.names
        
        dir_time = convert_timestamp.strftime("ETRI_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
        dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
        os.makedirs(dir_path, exist_ok=True)
        
        timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
        self.file_name = r"{}/AMTI_{}.csv".format(dir_path, timestamp_set)
        
        ts_df = pd.DataFrame([timestamp], columns=["Timestamp"])
        output_df = pd.concat([ts_df, df_flat], axis=1)
        if not self.file_exist():
           output_df.to_csv(self.file_name, mode='a', index=False, header=True)
        else:
           output_df.to_csv(self.file_name, mode='a', index=False, header=False)
           
    def make_csv_v2(self, result_list, click_timestamp):
        while len(result_list) > 0:
            data = result_list[0]['amti']
            timestamp = struct.unpack('!32s', result_list[0]["TS"])[0].decode()
            result_list.pop(0)
            
            convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")
            amti_arry = []
            
            for i in range(1, len(data[0])):
                amti_arry.append(data[0][i])
            
            df = pd.DataFrame(amti_arry)
            df_flat = pd.DataFrame([df.values.flatten()])
            df_flat.columns = amti_column.names
            
            dir_time = convert_timestamp.strftime("ETRI_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
            dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
            os.makedirs(dir_path, exist_ok=True)
            
            timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
            self.file_name = r"{}/AMTI_{}.csv".format(dir_path, timestamp_set)
            
            ts_df = pd.DataFrame([timestamp], columns=["Timestamp"])
            output_df = pd.concat([ts_df, df_flat], axis=1)
            if not self.file_exist():
                output_df.to_csv(self.file_name, mode='a', index=False, header=True)
            else:
                output_df.to_csv(self.file_name, mode='a', index=False, header=False)
                
    def make_csv_v3(self, amti_queue, click_timestamp):
        while True :
            queue_data = amti_queue.get()
            
            if queue_data == True:
                break
            
            if 'amti' in queue_data:
                data = queue_data['amti']
                data_TS = queue_data['TS']
                timestamp = struct.unpack('!32s', data_TS)[0].decode()
                
                convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")
                amti_arry = []
                
                for i in range(1, len(data[0])):
                    amti_arry.append(data[0][i])
                
                df = pd.DataFrame(amti_arry)
                df_flat = pd.DataFrame([df.values.flatten()])
                df_flat.columns = amti_column.names
                
                dir_time = convert_timestamp.strftime("ETRI_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
                dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
                os.makedirs(dir_path, exist_ok=True)
                
                timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
                self.file_name = r"{}/AMTI_{}.csv".format(dir_path, timestamp_set)
                
                ts_df = pd.DataFrame([timestamp], columns=["Timestamp"])
                output_df = pd.concat([ts_df, df_flat], axis=1)
                if not self.file_exist():
                    output_df.to_csv(self.file_name, mode='a', index=False, header=True)
                else:
                    output_df.to_csv(self.file_name, mode='a', index=False, header=False)
        print("[CSV] amti CSV Done :", datetime.now())
        
    def file_exist(self):
        # path = r"C:\Users\ADMIN\Desktop\soo\git\HIL-code\Examples\dataset\ETRI SoftSuit dataset\%s" % self.file_name
        path = self.file_name
        self.isExist = os.path.exists(path)

        return self.isExist