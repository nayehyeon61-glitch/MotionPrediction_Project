
import os.path
import struct
import numpy as np
import socket
import pandas as pd
from datetime import datetime, timedelta

import queue
import csv

class mono_camera_120_csv:
    def save_image_to_file(self, data, timestamp_data, click_timestamp):
        image_data = data
        recv_time = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")

        now_time = struct.unpack('!26s', timestamp_data[0:26])[0].decode()
        now_time = str(datetime.strptime(now_time, "%Y-%m-%d %H:%M:%S.%f") + timedelta(hours=9))
        date_part, time_part = now_time.split()
        year, month, day = date_part.split("-")
        hour, minute, second = time_part.split(":")
        current_time = f"ETRI_SoftSuit_{year}_{month}_{day}_{int(hour):02d}_{minute}_{second}"
        #current_time = now_time.strftime("ETRI_SoftSuit_%Y_%m_%d_%H_%M_%S.%f")
        dir_time = recv_time.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")

        #dir_path = r"C:\Users\ADMIN\Desktop\soo\git\HIL-code\Examples\dataset\ETRI SoftSuit dataset\{}".format(dir_time)
        dir_path = "./dataset/Etri_SoftSuit_Dataset/{}/mono_camera".format(dir_time)
        os.makedirs(dir_path, exist_ok=True)

        self.file_name = r"{}\{}_img".format(dir_path, current_time)
        file_exist = self.file_exist()

        while True:
            if not file_exist:

                self.file_name = r"{}\{}_img.jpg".format(dir_path, current_time)
                with open(self.file_name, 'wb') as file:
                    file.write(image_data)
                break
            else:
                self.file_name = r"{}\{}_img.jpg".format(dir_path, current_time)
                
    def save_image_to_file_v2(self, result_list, click_timestamp):
        while len(result_list) > 0:
            data = result_list[0]['mono_camera']
            now_time = struct.unpack('!26s', result_list[0]["TS"][0:26])[0].decode()
            result_list.pop(0)
            
            image_data = data
            recv_time = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")

            now_time = str(datetime.strptime(now_time, "%Y-%m-%d %H:%M:%S.%f") + timedelta(hours=9))
            date_part, time_part = now_time.split()
            year, month, day = date_part.split("-")
            hour, minute, second = time_part.split(":")
            current_time = f"ETRI_SoftSuit_{year}_{month}_{day}_{int(hour):02d}_{minute}_{second}"
            #current_time = now_time.strftime("ETRI_SoftSuit_%Y_%m_%d_%H_%M_%S.%f")
            dir_time = recv_time.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")

            #dir_path = r"C:\Users\ADMIN\Desktop\soo\git\HIL-code\Examples\dataset\ETRI SoftSuit dataset\{}".format(dir_time)
            dir_path = "./dataset/Etri_SoftSuit_Dataset/{}/mono_camera".format(dir_time)
            os.makedirs(dir_path, exist_ok=True)

            self.file_name = r"{}\{}_img".format(dir_path, current_time)
            file_exist = self.file_exist()

            while True:
                if not file_exist:

                    self.file_name = r"{}\{}_img.jpg".format(dir_path, current_time)
                    with open(self.file_name, 'wb') as file:
                        file.write(image_data)
                    break
                else:
                    self.file_name = r"{}\{}_img.jpg".format(dir_path, current_time)
        print("[JPG] MONO Camera JPG Done : ", datetime.now())
        
    def save_image_to_file_v3(self, camera_queue, click_timestamp):
        while True :
            queue_data = camera_queue.get()
            
            if queue_data == True:
                break
            
            if 'mono_camera' in queue_data:
                data = queue_data['mono_camera']
                data_TS = queue_data['TS']
                now_time = struct.unpack('!26s',data_TS[0:26])[0].decode()
                
                image_data = data
                recv_time = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")

                now_time = str(datetime.strptime(now_time, "%Y-%m-%d %H:%M:%S.%f") + timedelta(hours=9))
                date_part, time_part = now_time.split()
                year, month, day = date_part.split("-")
                hour, minute, second = time_part.split(":")
                current_time = f"ETRI_SoftSuit_{year}_{month}_{day}_{int(hour):02d}_{minute}_{second}"
                #current_time = now_time.strftime("ETRI_SoftSuit_%Y_%m_%d_%H_%M_%S.%f")
                dir_time = recv_time.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")

                #dir_path = r"C:\Users\ADMIN\Desktop\soo\git\HIL-code\Examples\dataset\ETRI SoftSuit dataset\{}".format(dir_time)
                dir_path = "./dataset/Etri_SoftSuit_Dataset/{}/mono_camera".format(dir_time)
                os.makedirs(dir_path, exist_ok=True)

                self.file_name = r"{}\{}_img".format(dir_path, current_time)
                file_exist = self.file_exist()

                while True:
                    if not file_exist:

                        self.file_name = r"{}\{}_img.jpg".format(dir_path, current_time)
                        with open(self.file_name, 'wb') as file:
                            file.write(image_data)
                        break
                    else:
                        self.file_name = r"{}\{}_img.jpg".format(dir_path, current_time)
        print("MONO Camera JPG Done : ", datetime.now())

    def file_exist(self):
        #i += 1
        #path = r"C:\Users\ADMIN\Desktop\soo\git\HIL-code\Examples\dataset\Mono camera"
        path = self.file_name
        isExist = os.path.exists(path)

        return isExist