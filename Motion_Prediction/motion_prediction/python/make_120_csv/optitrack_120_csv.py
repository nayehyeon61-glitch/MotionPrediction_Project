import os.path
import struct
import numpy as np
import socket
import pandas as pd
from datetime import datetime

import queue
import csv


optitrack_mark_column = np.dtype([
    ('maker_id', 'U2'),
    ('maker_position_x', 'f4'),
    ('maker_position_y', 'f4'),
    ('maker_position_z', 'f4')
])

optitrack_sk_column = np.dtype([
    ('skeleton_id', 'U2'),
    ('skeleton_position_x', 'f4'),
    ('skeleton_position_y', 'f4'),
    ('skeleton_position_z', 'f4'),
    ('skeleton_oriental_w', 'f4'),
    ('skeleton_oriental_x', 'f4'),
    ('skeleton_oriental_y', 'f4'),
    ('skeleton_oriental_z', 'f4')
])
        
class optitrack_120_csv:    
    def make_csv(self, data, timestamp, click_timestamp):
        convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")
        marker_data = data['marker']
        skeleton_data = data['skeleton']
        mark_count_id = 0
        sk_count_id = 0
        mark_arry = []
        sk_arry = []
        for m in range(1, len(marker_data)):
            mark_num_id = marker_data[m][0]
            unparsed_mark_id = mark_num_id.to_bytes(2, byteorder='big')
            m_id = struct.unpack('2c', unparsed_mark_id)
            mark_string = struct.unpack('s', m_id[0])[0].decode()
            mark_num = struct.unpack('B' , m_id[1])[0]
            mark_id = mark_string+str(mark_num)
            mark_count_id += 1
            mark_arry.append(mark_id)
            
            for r in range(1, len(marker_data[m])):
                mark_arry.append(marker_data[m][r])
                
        marker_df = pd.DataFrame(mark_arry)
        marker_df_flat = pd.DataFrame([marker_df.values.flatten()])
        marker_df_flat.columns = optitrack_mark_column.names * mark_count_id
         
        for s in range(0, len(skeleton_data)):
            skeleton_num_id = skeleton_data[s][0]
            unparsed_skeleton_id = skeleton_num_id.to_bytes(2, byteorder='big')
            sid = struct.unpack('2c', unparsed_skeleton_id)
            sk_string = struct.unpack('s', sid[0])[0].decode()
            sk_num = struct.unpack('B' , sid[1])[0]
            
            skeleton_id = sk_string+str(sk_num)
            sk_count_id += 1
            sk_arry.append(skeleton_id)
            
            for r in range(1, len(skeleton_data[s])):
                sk_arry.append(skeleton_data[s][r])
                
                    
        skeleton_df = pd.DataFrame(sk_arry)
        skeleton_df_flat = pd.DataFrame([skeleton_df.values.flatten()])
        skeleton_df_flat.columns = optitrack_sk_column.names * sk_count_id
        
        ts_df = pd.DataFrame([timestamp], columns=["Timestamp"])
        output_df = pd.concat([ts_df, marker_df_flat, skeleton_df_flat], axis=1)

        dir_time = convert_timestamp.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
        dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
        os.makedirs(dir_path, exist_ok=True)

        self.file_name = r"{}\OptiTrack_{}.csv".format(dir_path, dir_time)
        if not self.file_exist():
            output_df.to_csv(self.file_name, mode='a', index=False, header=True)
        else:
            output_df.to_csv(self.file_name, mode='a', index=False, header=False)

    def make_csv_v2(self, data, timestamp, click_timestamp):
        convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")
        marker_data = data['marker']
        skeleton_data = data['skeleton']
        mark_count_id = 0
        sk_count_id = 0
        mark_arry = []
        sk_arry = []
        for m in range(0, len(marker_data)):
            mark_num_id = marker_data[m][0]
            unparsed_mark_id = mark_num_id.to_bytes(2, byteorder='big')
            m_id = struct.unpack('2c', unparsed_mark_id)
            mark_string = struct.unpack('s', m_id[0])[0].decode()
            mark_num = struct.unpack('B' , m_id[1])[0]
            mark_id = mark_string + str(mark_num)
            mark_count_id += 1
            mark_arry.append(mark_id)

            for r in range(1, len(marker_data[m])):
                mark_arry.append(marker_data[m][r])
                
        marker_array = np.array(mark_arry)
        marker_df_flat = np.ravel(marker_array)
        marker_df_flat = np.expand_dims(marker_df_flat, axis=0)
        
        marker_columns = [field_name for field_name in optitrack_mark_column.fields]
        marker_columns *= mark_count_id

        for s in range(0, len(skeleton_data)):
            skeleton_num_id = skeleton_data[s][0]
            unparsed_skeleton_id = skeleton_num_id.to_bytes(2, byteorder='big')
            sid = struct.unpack('2c', unparsed_skeleton_id)
            sk_string = struct.unpack('s', sid[0])[0].decode()
            sk_num = struct.unpack('B' , sid[1])[0]
            skeleton_id = sk_string + str(sk_num)
            sk_count_id += 1
            sk_arry.append(skeleton_id)

            for r in range(1, len(skeleton_data[s])):
                sk_arry.append(skeleton_data[s][r])

        skeleton_array = np.array(sk_arry)
        skeleton_df_flat = np.ravel(skeleton_array)
        skeleton_df_flat = np.expand_dims(skeleton_df_flat, axis=0)

        skeleton_columns = [field_name for field_name in optitrack_sk_column.fields]
        skeleton_columns *= sk_count_id
        
                                
        ts_df = np.array([[timestamp]])

        output_df = np.concatenate((ts_df, marker_df_flat, skeleton_df_flat), axis=1)
        timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
        
        dir_time = convert_timestamp.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
        dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
        os.makedirs(dir_path, exist_ok=True)

        self.file_name = r"{}/OptiTrack_{}.csv".format(dir_path, timestamp_set)

        if not self.file_exist():
            if mark_count_id == 0:
                np.savetxt(self.file_name, output_df, delimiter=",", fmt="%s", header="Timestamp," + ",".join(skeleton_columns), comments='')
            else:    
                np.savetxt(self.file_name, output_df, delimiter=",", fmt="%s", header="Timestamp," + ",".join(marker_columns) + ",".join(skeleton_columns), comments='')
        else:
            with open(self.file_name, 'ab') as f:
                np.savetxt(f, output_df, delimiter=",", fmt="%s")
                
    def make_csv_v3(self, result_list, click_timestamp):    
        marker_data = None
        skeleton_data = None
        while len(result_list) > 0:
            data = result_list[0]['optitrack']
            timestamp = struct.unpack('!32s', result_list[0]["TS"])[0].decode()
            result_list.pop(0)
            convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")
            
            if 'marker' in data :
                marker_data = data['marker']
            if 'skeleton' in data:
                skeleton_data = data['skeleton']
                
            mark_count_id = 0
            sk_count_id = 0
            mark_arry = []
            sk_arry = []
            
            if marker_data is not None:
                for m in range(0, len(marker_data)):
                    mark_num_id = marker_data[m][0]
                    unparsed_mark_id = mark_num_id.to_bytes(2, byteorder='big')
                    m_id = struct.unpack('2c', unparsed_mark_id)
                    mark_string = struct.unpack('s', m_id[0])[0].decode()
                    mark_num = struct.unpack('B' , m_id[1])[0]
                    mark_id = mark_string + str(mark_num)
                    mark_count_id += 1
                    mark_arry.append(mark_id)

                    for r in range(1, len(marker_data[m])):
                        mark_arry.append(marker_data[m][r])
                        
            marker_array = np.array(mark_arry)
            marker_df_flat = np.ravel(marker_array)
            marker_df_flat = np.expand_dims(marker_df_flat, axis=0)
            
            marker_columns = [field_name for field_name in optitrack_mark_column.fields]
            marker_columns *= mark_count_id

            if skeleton_data is not None :
                for s in range(0, len(skeleton_data)):
                    skeleton_num_id = skeleton_data[s][0]
                    unparsed_skeleton_id = skeleton_num_id.to_bytes(2, byteorder='big')
                    sid = struct.unpack('2c', unparsed_skeleton_id)
                    sk_string = struct.unpack('s', sid[0])[0].decode()
                    sk_num = struct.unpack('B' , sid[1])[0]
                    skeleton_id = sk_string + str(sk_num)
                    sk_count_id += 1
                    sk_arry.append(skeleton_id)

                    for r in range(1, len(skeleton_data[s])):
                        sk_arry.append(skeleton_data[s][r])

            skeleton_array = np.array(sk_arry)
            skeleton_df_flat = np.ravel(skeleton_array)
            skeleton_df_flat = np.expand_dims(skeleton_df_flat, axis=0)

            skeleton_columns = [field_name for field_name in optitrack_sk_column.fields]
            skeleton_columns *= sk_count_id
                
            ts_df = np.array([[timestamp]])

            output_df = np.concatenate((ts_df, marker_df_flat, skeleton_df_flat), axis=1)
                            
            timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
            
            dir_time = convert_timestamp.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
            dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
            os.makedirs(dir_path, exist_ok=True)

            self.file_name = r"{}/OptiTrack_{}.csv".format(dir_path, timestamp_set)

            if not self.file_exist():
                if mark_count_id == 0:
                    np.savetxt(self.file_name, output_df, delimiter=",", fmt="%s", header="Timestamp," + ",".join(skeleton_columns), comments='')
                else:    
                    np.savetxt(self.file_name, output_df, delimiter=",", fmt="%s", header="Timestamp," + ",".join(marker_columns) + ",".join(skeleton_columns), comments='')
            else:
                with open(self.file_name, 'ab') as f:
                    np.savetxt(f, output_df, delimiter=",", fmt="%s")
        
        print("optitrack CSV Done : ", datetime.now())
        
    def make_csv_v4(self, optitrack_queue, click_timestamp):    
        marker_data = None
        skeleton_data = None
        while True:
            queue_data = optitrack_queue.get()
            
            if queue_data == True:
                break
            
            if 'optitrack' in queue_data:
                data = queue_data['optitrack']
                data_TS = queue_data['TS']
                timestamp = struct.unpack('!32s', data_TS)[0].decode()
                convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S")
                
                if 'marker' in data :
                    marker_data = data['marker']
                if 'skeleton' in data:
                    skeleton_data = data['skeleton']
                    
                mark_count_id = 0
                sk_count_id = 0
                mark_arry = []
                sk_arry = []
                
                if marker_data is not None:
                    for m in range(0, len(marker_data)):
                        mark_num_id = marker_data[m][0]
                        unparsed_mark_id = mark_num_id.to_bytes(2, byteorder='big')
                        m_id = struct.unpack('2c', unparsed_mark_id)
                        mark_string = struct.unpack('s', m_id[0])[0].decode()
                        mark_num = struct.unpack('B' , m_id[1])[0]
                        mark_id = mark_string + str(mark_num)
                        mark_count_id += 1
                        mark_arry.append(mark_id)

                        for r in range(1, len(marker_data[m])):
                            mark_arry.append(marker_data[m][r])
                            
                marker_array = np.array(mark_arry)
                marker_df_flat = np.ravel(marker_array)
                marker_df_flat = np.expand_dims(marker_df_flat, axis=0)
                
                marker_columns = [field_name for field_name in optitrack_mark_column.fields]
                marker_columns *= mark_count_id

                if skeleton_data is not None :
                    for s in range(0, len(skeleton_data)):
                        skeleton_num_id = skeleton_data[s][0]
                        unparsed_skeleton_id = skeleton_num_id.to_bytes(2, byteorder='big')
                        sid = struct.unpack('2c', unparsed_skeleton_id)
                        sk_string = struct.unpack('s', sid[0])[0].decode()
                        sk_num = struct.unpack('B' , sid[1])[0]
                        skeleton_id = sk_string + str(sk_num)
                        sk_count_id += 1
                        sk_arry.append(skeleton_id)

                        for r in range(1, len(skeleton_data[s])):
                            sk_arry.append(skeleton_data[s][r])

                skeleton_array = np.array(sk_arry)
                skeleton_df_flat = np.ravel(skeleton_array)
                skeleton_df_flat = np.expand_dims(skeleton_df_flat, axis=0)

                skeleton_columns = [field_name for field_name in optitrack_sk_column.fields]
                skeleton_columns *= sk_count_id
                    
                ts_df = np.array([[timestamp]])

                output_df = np.concatenate((ts_df, marker_df_flat, skeleton_df_flat), axis=1)
                                
                timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
                
                dir_time = convert_timestamp.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
                dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
                os.makedirs(dir_path, exist_ok=True)

                self.file_name = r"{}/OptiTrack_{}.csv".format(dir_path, timestamp_set)

                if not self.file_exist():
                    if mark_count_id == 0:
                        np.savetxt(self.file_name, output_df, delimiter=",", fmt="%s", header="Timestamp," + ",".join(skeleton_columns), comments='')
                    else:    
                        np.savetxt(self.file_name, output_df, delimiter=",", fmt="%s", header="Timestamp," + ",".join(marker_columns) + ",".join(skeleton_columns), comments='')
                else:
                    with open(self.file_name, 'ab') as f:
                        np.savetxt(f, output_df, delimiter=",", fmt="%s")
        
        print("[CSV] optitrack CSV Done : ", datetime.now())
        
    def file_exist(self):
        #path = r"C:\Users\ADMIN\Desktop\soo\git\HIL\Examples\test.csv"
        
        path = self.file_name
        self.isExist = os.path.exists(path)

        return self.isExist