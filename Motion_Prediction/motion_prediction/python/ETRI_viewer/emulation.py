
from ETRI_viewer.sensor_parser import BinaryParser
from make_120_bvh.bvh_120 import BVH120frame
from make_120_csv.csv_120 import Csv120Frame
from ETRI_viewer.utils import split_packets, get_current_time
import pdb
from datetime import datetime

class EmulationModeNoThread:
    def __init__(self,file_path):
        self.file_path = file_path

    def get_ts_str_to_long(self, binary_ts_str):
        cleaned_date_string = str(binary_ts_str).strip("b'")

        # 문자열을 datetime 객체로 파싱합니다.
        date_format = '%Y-%m-%d %H:%M:%S.%f%z'
        parsed_date = datetime.strptime(cleaned_date_string, date_format)

        # 파싱된 datetime 객체를 Unix 타임스탬프로 변환합니다.
        timestamp_unix = int(parsed_date.timestamp() * 1000)

        return timestamp_unix

    def ts_unix_to_str(self, ts_unix):
        # 밀리초를 초로 변환
        timestamp_in_seconds = ts_unix / 1000

        # 초를 datetime 객체로 변환
        datetime_obj = datetime.fromtimestamp(timestamp_in_seconds)

        # 원하는 출력 형식을 설정
        # desired_format = '%Y-%m-%d %H:%M:%S.%f%z'
        desired_format = '%H:%M:%S.%f%z'

        # datetime 객체를 원하는 형식으로 문자열로 변환
        formatted_date = datetime_obj.strftime(desired_format)

        return formatted_date

    def milliseconds_to_str(self, milliseconds):
        # 밀리초를 초, 분, 시간으로 변환
        seconds = milliseconds // 1000
        minutes, seconds = divmod(seconds, 60)
        hours, minutes = divmod(minutes, 60)

        # 시, 분, 초, 밀리초를 문자열로 변환하고 포맷팅
        formatted_time = "{:02d}:{:02d}:{:02d}.{:03d}".format(hours, minutes, seconds, milliseconds % 1000)

        return formatted_time

    def readBin(self):
        try:
            self.mode="rb"
            # .bin 파일을 읽음
            file_reader = FileReader(self.file_path, mode=self.mode)
            file_data = file_reader.get_file_data()

            # 파일에서 전체 패킷의 갯수를 구함
            line_data_list = split_packets(file_data)

            total_idx = len(line_data_list)

            data_list = []

            end_time = None
            start_time = None
            start_index=self.file_path.find("SoftSuit")
            XSENS_CONTAINS_SCALE_="2024" in self.file_path[start_index:]

            # 패킷 별로 파싱 처리
            for i in range(total_idx):
                data = dict()
                result, xsens_raw_data = BinaryParser().parser(line_data_list[i],XSENS_CONTAINS_SCALE_)

                for key, item in result.items():
                    data[key] = item

                    if key == "TS":
                        data["time_milis"] = self.get_ts_str_to_long(str(item))

                    # 첫번째 패킷일 경우에는 전체 패킷의 시작 시간을 저장
                    if i == 0 and key == "TS":
                        start_time = str(item)  # f"{i}" # self.parseBinaryTs(item)

                    # 마지막 패킷일 경우에는 전체 패킷의 마지막 시간을 저장
                    if i == total_idx-1 and key == "TS":
                        end_time = str(item)  # f"{i}" #self.parseBinaryTs(item)

                # 패킷 1개를 읽고, append 처리
                data["xsens_raw_data"] = xsens_raw_data
                data["binary_data"] = line_data_list[i]
                data_list.append(data)

            e = self.get_ts_str_to_long(end_time)
            s = self.get_ts_str_to_long(start_time)

            data_list_len = len(data_list)

            if data_list_len == 0:
                print("data_list len is 0")
                return

            # print(data_list[0])
            self.current_idx=0
            self.data_list=data_list
        except Exception as e:
            print("Error", str(e))

        finally:
            print(get_current_time(), "Loading finished")

    def getOneData(self):
        try:
            # 바이너리 데이터 패킷 첫번째 부터 끝까지
            data_list_len=len(self.data_list)
            current_idx=self.current_idx
            data_list=self.data_list
            if self.current_idx < data_list_len:
                data = data_list[current_idx]
                move_s = self.get_ts_str_to_long(data["TS"])
                message = {
                    "recv_data": data,
                    "slider_value": [
                        current_idx,  # 현재 시뮬레이션 list index
                        data_list_len,  # 전체 인덱스 (total row count)
                        ],
                    "delay_milis" : data_list[current_idx + 1]["time_milis"] - data_list[current_idx]["time_milis"]
                }

                self.current_idx += 1
                return data, message

        except Exception as e:
            print("Error", str(e))

        finally:
            print(get_current_time(), "getOneData ended")

class FileWriter:
    def __init__(self, filename):
        self.filename = filename
        self.file = open(self.filename, mode='a+t', encoding='utf-8')
        self.file.seek(0)
        if self.file.read().strip() == '': # 파일에 아무데이터도 없는 경우 헤더 추가
            self.file.write('header1,header2,header3\n')

    def append_data(self, data):
        for row in data:
            self.file.write(','.join(row) + '\n')

    def close(self):
        self.file.close()
class FileReader:
    def __init__(self, filename, mode='rt'):
        self.filename = filename
        self.mode = mode
        self.file = None

    def open(self):
        if self.mode == "rb":
            self.file = open(self.filename, self.mode)
        else:
            self.file = open(self.filename, self.mode, encoding='utf-8')

    def get_lines(self):
        self.open()
        lines = []
        for line in self.file:
            lines.append(line.strip())
        self.close()
        return lines

    def get_file_data(self):
        self.open()
        result = self.file.read()
        self.close()
        return result

    def close(self):
        self.file.close()
