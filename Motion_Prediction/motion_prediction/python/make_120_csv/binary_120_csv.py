from datetime import datetime
import os


class binary_120_bin:
    def __init__(self):
        self.test = 0

    def make_binary(self, binary_data, click_timestamp):
        convert_timestamp = datetime.strptime(click_timestamp, "%Y-%m-%d %H:%M:%S.%f")
        timestamp_set = convert_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
        dir_time = convert_timestamp.strftime("Etri_SoftSuit_Dataset_%Y_%m_%d_%H_%M_%S")
        #dir_path = r"C:\Users\ADMIN\Desktop\soo\git\HIL-code\Examples\dataset\ETRI SoftSuit dataset\{}".format(dir_time)
        dir_path = "./dataset/Etri_SoftSuit_Dataset/{}".format(dir_time)
        os.makedirs(dir_path, exist_ok=True)

        file_name = r"{}\SoftSuit_LE_{}.bin".format(dir_path, timestamp_set)
        with open(file_name, 'ab') as f:
            f.write(binary_data)

    def file_exist(self, file_name):
        path = self.file_name
        isExist = os.path.exists(path)

        return isExist