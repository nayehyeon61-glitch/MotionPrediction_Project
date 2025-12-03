# 2024.05.19 yjkim 파일 생성
# viewer 관련 소스코드에서 중복으로 사용되는 함수 저장함
# split packets()
# binary_datetime_to_milliseconds()

import struct
from datetime import datetime, timezone


# .bin 파일을 읽고, 1 프레임별로 나눠주는 코드
def split_packets(file_data):
    read_len = 0
    line_data = []
    while len(file_data) > read_len:
        read_len += 3
        msg_len = struct.unpack('!I', file_data[read_len:read_len + 4])[0]
        read_len += 4
        msg = file_data[read_len:read_len + msg_len + 1]
        read_len += msg_len + 1
        line_data.append(msg)

    return line_data


# 바이너리 데이터 -> 문자열 -> datetime -> 밀리초 변환
def binary_datetime_to_milliseconds(binary_datetime):
    # 바이너리 데이터를 문자열로 변환
    datetime_str = binary_datetime.decode('utf-8')

    # 문자열을 datetime 객체로 파싱
    # dt = datetime.strptime(datetime_str, "%Y-%m-%d %H:%M:%S.%f%z")
    dt = datetime.fromisoformat(datetime_str)

    # UTC 시간으로 변환 후 밀리초로 변환
    # utc_dt = dt.astimezone(timezone.utc)
    # milliseconds = int(utc_dt.timestamp() * 1000)
    milliseconds = int(dt.timestamp() * 1000)

    return milliseconds


def get_current_time():
    current_time = datetime.now()
    return current_time.strftime("%Y-%m-%d %H:%M:%S")
