import struct
import sys
from multiprocessing import Process

sys.path.append('..')
from make_120_csv import amti_120_csv, delsys_120_csv, etri_120_csv, futek_120_csv, mono_camera_120_csv, optitrack_120_csv, xsens_120_csv, softsensor_120_csv, fmgmmg_120_csv


class csv_120_frame:
    def __init__(self):
        self.test = 0
        self.i = 0
        
    def run(self, result, click_timestamp):
        timestamp = struct.unpack('!32s', result["TS"])[0].decode()
        
        if 'xsens' in result:
            #xsens_120_csv.xsens_120_csv().make_csv(result['xsens'], timestamp, click_timestamp)
            xsens_120_csv.xsens_120_csv().make_csv_v2(result['xsens'], timestamp, click_timestamp)
        if 'futek' in result:
            #futek_120_csv.futek_120_csv().test_make_csv(result['futek'], result["TS"], click_timestamp)
            futek_120_csv.futek_120_csv().make_csv(result['futek'], timestamp, click_timestamp)
        if 'delsys' in result:
            delsys_120_csv.delsys_120_csv().make_csv(result['delsys'], timestamp, click_timestamp)
        if 'etri_sensor' in result:
            etri_120_csv.etri_120_csv().make_csv(result['etri_sensor'], timestamp, click_timestamp)
        if 'amti' in result:
            amti_120_csv.amti_120_csv().make_csv(result['amti'], timestamp, click_timestamp)
        if 'optitrack' in result:
            #optitrack_120_csv.optitrack_120_csv().make_csv(result["optitrack"], timestamp, click_timestamp)
            optitrack_120_csv.optitrack_120_csv().make_csv_v2(result["optitrack"], timestamp, click_timestamp)
        if 'mono_camera' in result:
            mono_camera_120_csv.mono_camera_120_csv().save_image_to_file(result["mono_camera"], result['TS'], click_timestamp)
        if 'fmgmmg' in result:
            FMGMMG_120_csv.FMGMMG_120_csv().make_csv(result['fmgmmg'], result['TS'], click_timestamp)
        if 'softsensor' in result:
            softsensor_120_csv.softsensor_120_csv().make_csv(result['softsensor'], result['TS'], click_timestamp)

# Process version
class Csv120Frame:
    def __init__(self):
        self.test = 0
        self.i = 0
        
    def run(self, test_queue, result, click_timestamp):    
        result_list = []
        
        while test_queue.qsize() > 0:
            result_list.append(test_queue.get())
        
        if 'xsens' in result_list[0]:
            xsens_process = Process(target=xsens_120_csv.xsens_120_csv().make_csv_v3, args = (result_list, click_timestamp))
            xsens_process.start()
        if 'optitrack' in result_list[0]:
            optitrack_process = Process(target=optitrack_120_csv.optitrack_120_csv().make_csv_v3, args = (result_list, click_timestamp))
            optitrack_process.start()
        if 'etri_sensor' in result_list[0]:
            etri_process = Process(target = etri_120_csv.etri_120_csv().make_csv_v2, args = (result_list, click_timestamp))
            etri_process.start()
        if 'futek' in result_list[0]:
            futek_process = Process(target=futek_120_csv.futek_120_csv().make_csv_v2, args = (result_list, click_timestamp))
            futek_process.start()
        if 'delsys' in result_list[0]:
            delsys_process = Process(target = delsys_120_csv.delsys_120_csv().make_csv_v2, args = (result_list, click_timestamp))
            delsys_process.start()
        if 'amti' in result_list[0]:
            amti_process = Process(target = amti_120_csv.amti_120_csv().make_csv_v2, args = (result_list, click_timestamp))
            amti_process.start()
        if 'mono_camera' in result_list[0]:
            camera_process = Process(target = mono_camera_120_csv.mono_camera_120_csv().save_image_to_file_v2, args = (result_list, click_timestamp))
            camera_process.start()
        if 'fmgmmg' in result_list[0]:
            FMGMMG_process = Process(target = FMGMMG_120_csv.FMGMMG_120_csv().make_csv_v2, args=(result_list, click_timestamp))
            FMGMMG_process.start()
        if 'softsensor' in result_list[0]:
            softsensor_process = Process(target = softsensor_120_csv.softsensor_120_csv().make_csv_v2, args = (result_list, click_timestamp))
            softsensor_process.start()

    def run_v2(self, result, xsens_queue, optitrack_queue, futek_queue, etri_queue, amti_queue, delsys_queue, mono_camera_queue, FMGMMG_queue, softsensor_queue, click_timestamp):    
        if 'xsens' in result:
            xsens_process = Process(target=xsens_120_csv.xsens_120_csv().make_csv_v4, args = (xsens_queue, click_timestamp))
            xsens_process.start()
        if 'optitrack' in result:
            optitrack_process = Process(target=optitrack_120_csv.optitrack_120_csv().make_csv_v4, args = (optitrack_queue, click_timestamp))
            optitrack_process.start()
        if 'etri' in result:
            etri_process = Process(target = etri_120_csv.etri_120_csv().make_csv_v3, args = (etri_queue, click_timestamp))
            etri_process.start()
        if 'futek' in result:
            futek_process = Process(target=futek_120_csv.futek_120_csv().make_csv_v3, args = (futek_queue, click_timestamp))
            futek_process.start()
        if 'delsys' in result:
            delsys_process = Process(target = delsys_120_csv.delsys_120_csv().make_csv_v3, args = (delsys_queue, click_timestamp))
            delsys_process.start()
        if 'amti' in result:
            amti_process = Process(target = amti_120_csv.amti_120_csv().make_csv_v3, args = (amti_queue, click_timestamp))
            amti_process.start()
        if 'mono_camera' in result:
            camera_process = Process(target = mono_camera_120_csv.mono_camera_120_csv().save_image_to_file_v3, args = (mono_camera_queue, click_timestamp))
            camera_process.start()
        if 'fmgmmg' in result:
            FMGMMG_process = Process(target = FMGMMG_120_csv.FMGMMG_120_csv().make_csv_v3, args=(FMGMMG_queue, click_timestamp))
            FMGMMG_process.start()
        if 'softsensor' in result:
            softsensor_process = Process(target = softsensor_120_csv.softsensor_120_csv().make_csv_v3, args = (softsensor_queue, click_timestamp))
            softsensor_process.start()