import os.path
import struct
import numpy as np
import socket
import pandas as pd
from datetime import datetime

import queue
import csv


class Etri_struct():
    def csv_struct(self, body):
        return np.dtype([
            ("MTi_{}.Gqw".format(body), 'f4'),
            ("MTi_{}.Gqx".format(body), 'f4'),
            ("MTi_{}.Gqy".format(body), 'f4'),
            ("MTi_{}.Gqz".format(body), 'f4'),

            ("MTi_{}.ax".format(body), 'f4'),
            ("MTi_{}.ay".format(body), 'f4'),
            ("MTi_{}.az".format(body), 'f4'),

            ("MTi_{}.gx".format(body), 'f4'),
            ("MTi_{}.gy".format(body), 'f4'),
            ("MTi_{}.gz".format(body), 'f4'),

            ("MTi_{}.mx".format(body), 'f4'),
            ("MTi_{}.my".format(body), 'f4'),
            ("MTi_{}.mz".format(body), 'f4'),

            ("MTi_{}.tmep".format(body), 'f4'),
            ("MTi_{}.dt".format(body), 'f4'),
            ("MTi_{}.timestamp".format(body), 'f4'),

            ("JE_LeftKnee.turns", 'f4'),
            ("JE_LeftKnee.angle", 'f4'),
            ("JE_RightKnee.turns", 'f4'),
            ("JE_RightKnee.angle", 'f4'),

            ("Insole_LeftFoot.fsrT", 'f4'),
            ("Insole_LeftFoot.fsrB", 'f4'),
            ("Insole_LeftFoot.fsrM", 'f4'),
            ("Insole_LeftFoot.fsrH", 'f4'),

            ("Insole_LeftFoot.ax", 'f4'),
            ("Insole_LeftFoot.ay", 'f4'),
            ("Insole_LeftFoot.az", 'f4'),

            ("Insole_LeftFoot.gx", 'f4'),
            ("Insole_LeftFoot.gy", 'f4'),
            ("Insole_LeftFoot.gz", 'f4'),

            ("Insole_LeftFoot.qw", 'f4'),
            ("Insole_LeftFoot.qx", 'f4'),
            ("Insole_LeftFoot.qy", 'f4'),
            ("Insole_LeftFoot.qz", 'f4'),

            ("Insole_LeftFoot.temp", 'f4'),
            ("Insole_LeftFoot.dt", 'f4'),
            ("Insole_LeftFoot.SeqNum", 'f4'),

            ("Insole_RightFoot.fsrT", 'f4'),
            ("Insole_RightFoot.fsrB", 'f4'),
            ("Insole_RightFoot.fsrM", 'f4'),
            ("Insole_RightFoot.fsrH", 'f4'),

            ("Insole_RightFoot.ax", 'f4'),
            ("Insole_RightFoot.ay", 'f4'),
            ("Insole_RightFoot.az", 'f4'),

            ("Insole_RightFoot.gx", 'f4'),
            ("Insole_RightFoot.gy", 'f4'),
            ("Insole_RightFoot.gz", 'f4'),

            ("Insole_RightFoot.qw", 'f4'),
            ("Insole_RightFoot.qx", 'f4'),
            ("Insole_RightFoot.qy", 'f4'),
            ("Insole_RightFoot.qz", 'f4'),

            ("Insole_RightFoot.temp", 'f4'),
            ("Insole_RightFoot.dt", 'f4'),
            ("Insole_RightFoot.SeqNum", 'f4'),

            ("TAG", 'u2')
        ])
        
class etri_120_csv:    
    def make_csv(self, data, timestamp, click_timestamp):
        convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")
        etri_arry = []
        IMU_key = data['IMU_key']
        IMU_quater = data['IMU_quater']
        IMU_euler = data['IMU_euler']
        IMU_acc = data['IMU_acc']
        IMU_gyro = data['IMU_gyro']
        IMU_mag = data['IMU_mag']
        joint_R = data['joint_R']
        joint_L = data['joint_L']
        insole_R = data['insole_R']
        insole_L = data['insole_L']
        
        for q in range(len(IMU_quater[0])):
            etri_arry.append(IMU_quater[0][q])
        
        for a in range(len(IMU_acc[0])):
            etri_arry.append(IMU_acc[0][a])
            
        for g in range(len(IMU_gyro[0])):
            etri_arry.append(IMU_gyro[0][g])
            
        for m in range(len(IMU_mag[0])):
            etri_arry.append(IMU_mag[0][m])
            
        #"MTi_{}.tmep" , "MTi_{}.dt", "MTi_{}.timestamp"
        for test in range(3):
            etri_arry.append(0.0)
            
        # for jl in range(1, len(joint_L[0])):
        #     etri_arry.append(joint_L[0][jl])
        etri_arry.append(0.0)
        etri_arry.append(0.0)
        
        # for jr in range(1, len(joint_R[0])):
        #     etri_arry.append(joint_R[0][jr])
        etri_arry.append(0.0)
        etri_arry.append(0.0)
            
        for il in range(1,len(insole_L[0])):
            etri_arry.append(insole_L[0][il])
            
        #"Insole_LeftFoot.temp", "Insole_LeftFoot.dt", "Insole_LeftFoot.SeqNum"
        etri_arry.append(0.0)
        etri_arry.append(0.0)
        etri_arry.append(insole_L[0][0])
            
        for ir in range(1,len(insole_R[0])):
            etri_arry.append(insole_R[0][ir])
            
        #"Insole_RightFoot.temp", "Insole_RightFoot.dt" ,"Insole_RightFoot.SeqNum"
        etri_arry.append(0.0)
        etri_arry.append(0.0)
        etri_arry.append(insole_R[0][0])
        
        etri_arry.append("")
        
        
        csv_strcut = Etri_struct().csv_struct(self.IMU_key_parser(IMU_key[0]))
        df = pd.DataFrame(etri_arry)
        df_flat = pd.DataFrame([df.values.flatten()])
        df_flat.columns = csv_strcut.names
        
        dir_time = convert_timestamp.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
        dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
        os.makedirs(dir_path, exist_ok=True)
        
        timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
        self.file_name = r"{}/ETRI_SoftSuit_{}.csv".format(dir_path, timestamp_set)
        ts_df = pd.DataFrame([timestamp], columns=["Timestamp"])
        output_df = pd.concat([ts_df, df_flat], axis=1)
        if not self.file_exist():
            output_df.to_csv(self.file_name, mode='a', index=False, header=True)
        else:
            output_df.to_csv(self.file_name, mode='a', index=False, header=False)
            
    def make_csv_v2(self, result_list, click_timestamp):
        while len(result_list) > 0:
            data = result_list[0]['etri_sensor']
            timestamp = struct.unpack('!32s', result_list[0]["TS"])[0].decode()
            result_list.pop(0)
            
            convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")
            etri_arry = []
            IMU_key = data['IMU_key']
            IMU_quater = data['IMU_quater']
            IMU_euler = data['IMU_euler']
            IMU_acc = data['IMU_acc']
            IMU_gyro = data['IMU_gyro']
            IMU_mag = data['IMU_mag']
            joint_R = data['joint_R']
            joint_L = data['joint_L']
            insole_R = data['insole_R']
            insole_L = data['insole_L']
            
            for q in range(len(IMU_quater[0])):
                etri_arry.append(IMU_quater[0][q])
            
            for a in range(len(IMU_acc[0])):
                etri_arry.append(IMU_acc[0][a])
                
            for g in range(len(IMU_gyro[0])):
                etri_arry.append(IMU_gyro[0][g])
                
            for m in range(len(IMU_mag[0])):
                etri_arry.append(IMU_mag[0][m])
                
            #"MTi_{}.tmep" , "MTi_{}.dt", "MTi_{}.timestamp"
            for test in range(3):
                etri_arry.append(0.0)
                
            # for jl in range(1, len(joint_L[0])):
            #     etri_arry.append(joint_L[0][jl])
            etri_arry.append(0.0)
            etri_arry.append(0.0)
            
            # for jr in range(1, len(joint_R[0])):
            #     etri_arry.append(joint_R[0][jr])
            etri_arry.append(0.0)
            etri_arry.append(0.0)
                
            for il in range(1,len(insole_L[0])):
                etri_arry.append(insole_L[0][il])
                
            #"Insole_LeftFoot.temp", "Insole_LeftFoot.dt", "Insole_LeftFoot.SeqNum"
            etri_arry.append(0.0)
            etri_arry.append(0.0)
            etri_arry.append(insole_L[0][0])
                
            for ir in range(1,len(insole_R[0])):
                etri_arry.append(insole_R[0][ir])
                
            #"Insole_RightFoot.temp", "Insole_RightFoot.dt" ,"Insole_RightFoot.SeqNum"
            etri_arry.append(0.0)
            etri_arry.append(0.0)
            etri_arry.append(insole_R[0][0])
            
            etri_arry.append("")
            
            
            csv_strcut = Etri_struct().csv_struct(self.IMU_key_parser(IMU_key[0]))
            df = pd.DataFrame(etri_arry)
            df_flat = pd.DataFrame([df.values.flatten()])
            df_flat.columns = csv_strcut.names
            
            dir_time = convert_timestamp.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
            dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
            os.makedirs(dir_path, exist_ok=True)
            
            timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
            self.file_name = r"{}/ETRI_SoftSuit_{}.csv".format(dir_path, timestamp_set)
            ts_df = pd.DataFrame([timestamp], columns=["Timestamp"])
            output_df = pd.concat([ts_df, df_flat], axis=1)
            if not self.file_exist():
                output_df.to_csv(self.file_name, mode='a', index=False, header=True)
            else:
                output_df.to_csv(self.file_name, mode='a', index=False, header=False)
        print("ETRI softsuit CSV Done : ", datetime.now())
        
    def make_csv_v3(self, etri_queue, click_timestamp):
        while True:
            queue_data = etri_queue.get()
            
            if queue_data == True:
                break
            
            if 'etri' in queue_data:
                data = queue_data['etri']
                data_TS = queue_data['TS']
                
                timestamp = struct.unpack('!32s', data_TS)[0].decode()
                
                convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")
                etri_arry = []
                IMU_key = data['IMU_key']
                IMU_quater = data['IMU_quater']
                IMU_euler = data['IMU_euler']
                IMU_acc = data['IMU_acc']
                IMU_gyro = data['IMU_gyro']
                IMU_mag = data['IMU_mag']
                joint_R = data['joint_R']
                joint_L = data['joint_L']
                insole_R = data['insole_R']
                insole_L = data['insole_L']
                
                for q in range(len(IMU_quater[0])):
                    etri_arry.append(IMU_quater[0][q])
                
                for a in range(len(IMU_acc[0])):
                    etri_arry.append(IMU_acc[0][a])
                    
                for g in range(len(IMU_gyro[0])):
                    etri_arry.append(IMU_gyro[0][g])
                    
                for m in range(len(IMU_mag[0])):
                    etri_arry.append(IMU_mag[0][m])
                    
                #"MTi_{}.tmep" , "MTi_{}.dt", "MTi_{}.timestamp"
                for test in range(3):
                    etri_arry.append(0.0)
                    
                # for jl in range(1, len(joint_L[0])):
                #     etri_arry.append(joint_L[0][jl])
                etri_arry.append(0.0)
                etri_arry.append(0.0)
                
                # for jr in range(1, len(joint_R[0])):
                #     etri_arry.append(joint_R[0][jr])
                etri_arry.append(0.0)
                etri_arry.append(0.0)
                    
                for il in range(1,len(insole_L[0])):
                    etri_arry.append(insole_L[0][il])
                    
                #"Insole_LeftFoot.temp", "Insole_LeftFoot.dt", "Insole_LeftFoot.SeqNum"
                etri_arry.append(0.0)
                etri_arry.append(0.0)
                etri_arry.append(insole_L[0][0])
                    
                for ir in range(1,len(insole_R[0])):
                    etri_arry.append(insole_R[0][ir])
                    
                #"Insole_RightFoot.temp", "Insole_RightFoot.dt" ,"Insole_RightFoot.SeqNum"
                etri_arry.append(0.0)
                etri_arry.append(0.0)
                etri_arry.append(insole_R[0][0])
                
                etri_arry.append("")
                
                
                csv_strcut = Etri_struct().csv_struct(self.IMU_key_parser(IMU_key[0]))
                df = pd.DataFrame(etri_arry)
                df_flat = pd.DataFrame([df.values.flatten()])
                df_flat.columns = csv_strcut.names
                
                dir_time = convert_timestamp.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
                dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
                os.makedirs(dir_path, exist_ok=True)
                
                timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
                self.file_name = r"{}/ETRI_SoftSuit_{}.csv".format(dir_path, timestamp_set)
                ts_df = pd.DataFrame([timestamp], columns=["Timestamp"])
                output_df = pd.concat([ts_df, df_flat], axis=1)
                if not self.file_exist():
                    output_df.to_csv(self.file_name, mode='a', index=False, header=True)
                else:
                    output_df.to_csv(self.file_name, mode='a', index=False, header=False)
        print("[CSV] ETRI softsuit CSV Done : ", datetime.now())
        
    def file_exist(self):
        # path = r"C:\Users\ADMIN\Desktop\soo\git\HIL-code\Examples\dataset\ETRI SoftSuit dataset\%s" % self.file_name
        path = self.file_name
        self.isExist = os.path.exists(path)

        return self.isExist
    
    def IMU_key_parser(self, key):
            if key == 5:
                body = "HIPS"
            elif key == 28:
                body = "R_THIGH"
            elif key == 15:
                body = "L_THIGH"
            elif key == 26:
                body = "R_LLEG"
            elif key == 13:
                body = "L_LLEG"
            elif key == 3:
                body = "HEAD"
            elif key == 2:
                body = "Chest"
            else:
                print("IMU key error , now key : ", key)
                body = ""

            return body
