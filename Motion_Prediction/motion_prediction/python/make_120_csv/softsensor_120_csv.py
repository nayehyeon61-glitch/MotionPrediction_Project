import os.path
import struct
import numpy as np
import socket
import pandas as pd
from datetime import datetime

import queue
import csv
        
class softsensor_120_csv:    
    def make_csv(self, data, timestamp ,click_timestamp):
        convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")
        
        ADS_list = []
        BNO_list = []
        value_list = []
        
        for key, value in data.items():
            if "ADS" in key:
                ADS_list.append(key)
            elif "BNO" in key:
                BNO_list.append(key)
                
            for i in range(len(value)):
                if len(value) > 2 and i != 0:
                    value_list.append(value[i])
                elif len(value) == 2:
                    value_list.append(value[i])
        
        csv_header = self.create_csv_header(ADS_list, BNO_list)
        df = pd.DataFrame(value_list)
        df_flat = pd.DataFrame([df.values.flatten()])
        df_flat.columns = csv_header.tolist()
        
        dir_time = convert_timestamp.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
        dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
        os.makedirs(dir_path, exist_ok=True)
        
        timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
        self.file_name = r"{}/ETRI_softsensor_{}.csv".format(dir_path, timestamp_set)
        ts_df = pd.DataFrame([timestamp], columns=["Timestamp"])
        output_df = pd.concat([ts_df, df_flat], axis=1)
        if not self.file_exist():
            output_df.to_csv(self.file_name, mode='a', index=False, header=True)
        else:
            output_df.to_csv(self.file_name, mode='a', index=False, header=False)
            
    def make_csv_v2(self, result_list, click_timestamp):
        while len(result_list) > 0:
            data = result_list[0]['softsensor']
            timestamp = struct.unpack('!32s', result_list[0]["TS"])[0].decode()
            result_list.pop(0)
            
            convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")
            
            ADS_list = []
            BNO_list = []
            value_list = []
            
            for key, value in data.items():
                if "ADS" in key:
                    ADS_list.append(key)
                elif "BNO" in key:
                    BNO_list.append(key)
                    
                for i in range(len(value)):
                    if len(value) > 2 and i != 0:
                        value_list.append(value[i])
                    elif len(value) == 2:
                        value_list.append(value[i])
            
            csv_header = self.create_csv_header(ADS_list, BNO_list)
            df = pd.DataFrame(value_list)
            df_flat = pd.DataFrame([df.values.flatten()])
            df_flat.columns = csv_header.tolist()
            
            dir_time = convert_timestamp.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
            dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
            os.makedirs(dir_path, exist_ok=True)
            
            timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
            self.file_name = r"{}/ETRI_softsensor_{}.csv".format(dir_path, timestamp_set)
            ts_df = pd.DataFrame([timestamp], columns=["Timestamp"])
            output_df = pd.concat([ts_df, df_flat], axis=1)
            if not self.file_exist():
                output_df.to_csv(self.file_name, mode='a', index=False, header=True)
            else:
                output_df.to_csv(self.file_name, mode='a', index=False, header=False)            
        print("Softsensor CSV Done : ", datetime.now())
        
    def make_csv_v3(self, softsensor_queue, click_timestamp):
        while True :
            queue_data = softsensor_queue.get()
            
            if queue_data == True:
                break
            
            if 'softsensor' in queue_data:
                data = queue_data['softsensor']
                data_TS = queue_data['TS']
                timestamp = struct.unpack('!32s',data_TS)[0].decode()
                
                
                convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")
                
                ADS_list = []
                BNO_list = []
                value_list = []
                
                for key, value in data.items():
                    if "ADS" in key:
                        ADS_list.append(key)
                    elif "BNO" in key:
                        BNO_list.append(key)
                        
                    for i in range(len(value)):
                        if len(value) > 2 and i != 0:
                            value_list.append(value[i])
                        elif len(value) == 2:
                            value_list.append(value[i])
                
                csv_header = self.create_csv_header(ADS_list, BNO_list)
                df = pd.DataFrame(value_list)
                df_flat = pd.DataFrame([df.values.flatten()])
                df_flat.columns = csv_header.tolist()
                
                dir_time = convert_timestamp.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
                dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
                os.makedirs(dir_path, exist_ok=True)
                
                timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
                self.file_name = r"{}/ETRI_softsensor_{}.csv".format(dir_path, timestamp_set)
                ts_df = pd.DataFrame([timestamp], columns=["Timestamp"])
                output_df = pd.concat([ts_df, df_flat], axis=1)
                if not self.file_exist():
                    output_df.to_csv(self.file_name, mode='a', index=False, header=True)
                else:
                    output_df.to_csv(self.file_name, mode='a', index=False, header=False)            
        print("[CSV] Softsensor CSV Done : ", datetime.now())
        
    def file_exist(self):
        # path = r"C:\Users\ADMIN\Desktop\soo\git\HIL-code\Examples\dataset\ETRI SoftSuit dataset\%s" % self.file_name

        path = self.file_name
        self.isExist = os.path.exists(path)

        return self.isExist
    
    def create_csv_header(self, ADS_ID_list, BNO_ID_list):
        header = []
        
        #header.append('Seq.num')
        for i in range(len(ADS_ID_list)):
            header.append('ADS_'+str(ADS_ID_list[i])+'.x')
            header.append('ADS_'+str(ADS_ID_list[i])+'.y')

        for j in range(len(BNO_ID_list)):
            header.append('BNO_' + str(BNO_ID_list[j]) + '.ax')
            header.append('BNO_' + str(BNO_ID_list[j]) + '.ay')
            header.append('BNO_' + str(BNO_ID_list[j]) + '.az')
            header.append('BNO_' + str(BNO_ID_list[j]) + '.gx')
            header.append('BNO_' + str(BNO_ID_list[j]) + '.gy')
            header.append('BNO_' + str(BNO_ID_list[j]) + '.gz')
            header.append('BNO_' + str(BNO_ID_list[j]) + '.mx')
            header.append('BNO_' + str(BNO_ID_list[j]) + '.my')
            header.append('BNO_' + str(BNO_ID_list[j]) + '.mz')
            header.append('BNO_' + str(BNO_ID_list[j]) + '.qw')
            header.append('BNO_' + str(BNO_ID_list[j]) + '.qx')
            header.append('BNO_' + str(BNO_ID_list[j]) + '.qy')
            header.append('BNO_' + str(BNO_ID_list[j]) + '.qz')
            
        CSV_Header = np.array(header)
        
        return CSV_Header