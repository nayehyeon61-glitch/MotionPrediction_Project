import struct
import numpy as np
import socket
import sys

# struct==============================================
Packet_Header_type = np.dtype([
    ('Msg_Len', 'u4'),
    ('Sensor_Count', 'B')
])

Sensor_Header_type = np.dtype([
    ('SensorID', 'u2'),  # 원래 xsens MTi packet에는 없지만, ID 구분하기 위하여 넣었음
    ('Premble', 'B'),
    ('BID', 'B'),
    ('MID', 'B'),
    ('LEN', 'B'),
])

InsoleSensor_Header_type = np.dtype([
    ('SensorID', 'u2'),  # 원래 xsens MTi packet에는 없지만, ID 구분하기 위하여 넣었음
    ('Premble', 'u2'),
    ('DID', 'B'),
    ('MID', 'B'),
    ('LEN', 'B'),
    ('ToD', 'B'),
])

Data_Header_type = np.dtype([
    ('XDI', 'u2'),
    ('Len', 'B'),
])

UTC_Data_type = np.dtype([
    ('Data_NS', 'u4'),
    ('Data_YY', 'u2'),
    ('Data_MM', 'u1'),
    ('Data_DD', 'u1'),
    ('Data_hh', 'u1'),
    ('Data_mm', 'u1'),
    ('Data_ss', 'u1'),
    ('Data_F', 'u1'),
])

Quat_Data_type = np.dtype([
    ('Data_Qw', 'f4'),  # float 32
    ('Data_Qx', 'f4'),
    ('Data_Qy', 'f4'),
    ('Data_Qz', 'f4'),
])

Euler_Data_type = np.dtype([
    ('Data_roll', 'f4'),  # float 32
    ('Data_pitch', 'f4'),
    ('Data_yaw', 'f4'),
])

Acc_Data_type = np.dtype([
    ('Data_Ax', 'f4'),  # float 32
    ('Data_Ay', 'f4'),
    ('Data_Az', 'f4'),
])

Gyro_Data_type = np.dtype([
    ('Data_Gx', 'f4'),  # float 32
    ('Data_Gy', 'f4'),
    ('Data_Gz', 'f4'),
])

Mag_Data_type = np.dtype([
    ('Data_Mx', 'f4'),  # float 32
    ('Data_My', 'f4'),
    ('Data_Mz', 'f4'),
])

Status_Data_type = np.dtype([
    ('Status', 'u4'),
    ('Checksum', 'B')
])

Tension_Data_type = np.dtype([
    ('LID', 'u2'),
    ('Data', 'f4'),
])

Joint_Data_type = np.dtype([
    ('LID', 'u2'),
    ('Turn', 'i4'),
    ('Angle', 'i4'),
])

Insole_Data_type = np.dtype([
    ('PC', 'h'),
    ('FSR_T', 'f4'),
    ('FSR_B', 'f4'),
    ('FSR_M', 'f4'),
    ('FSR_H', 'f4'),
    ('Data_Ax', 'f4'),
    ('Data_Ay', 'f4'),
    ('Data_Az', 'f4'),
    ('Data_Gx', 'f4'),
    ('Data_Gy', 'f4'),
    ('Data_Gz', 'f4'),
    ('Data_Qw', 'f4'),
    ('Data_Qx', 'f4'),
    ('Data_Qy', 'f4'),
    ('Data_Qz', 'f4'),
])


# struct==============================================

class ETRI_client():
    def __init__(self):
        self.parsedLen = 0
        self.stop_thread_flag = False
        self.SENSOR_HEADER_LEN = 6
        self.DATA_HEADER_LEN = 3
        self.SOP_LEN = 3  # 3 bytes
        self.sync_code = [0xFA, 0xFF, 0xFE]
        self.CHK_Len = 1  # 1 byte
        self.XsensMTi_Header_len = 106  # 106 byte (include CHK)
        # self.sensor = {'Joint': {}, 'Insole': {}, 'Tension': {}}
        # self.IMU = {}

        self.SENSOR_OFFSET = 7

    # print(sensor[0][1])
    def recv_binary(self, client_socket):
        self.client_socket = client_socket
        while not self.stop_thread_flag:
            # print("stop flag: ", self.stop_thread_flag)
            self.parsedLen = 0

            # print(self.stop_thread_flag)
            # Sync Code(0xFA, 0xFF, 0xFE) 확인
            self.find_sync_code(self.client_socket)

            recv_data = self.client_socket.recv(4)
            # print(recv_data)
            while len(recv_data) != 4:  # 패킷 길이 만큼 수집 안될 경우 추가 수집
                recv_data = recv_data + self.client_socket.recv(4 - len(recv_data))
            MSG_LEN = int.from_bytes(recv_data, byteorder='big')
            # print("MSG_LEN = {}".format(MSG_LEN))
            self.msg_len = MSG_LEN
            self.binary_data = self.client_socket.recv(MSG_LEN + 1)
            while len(self.binary_data) != (MSG_LEN + 1):  # 패킷 길이 만큼 수집 안될 경우 추가 수집
                self.binary_data = self.binary_data + self.client_socket.recv(MSG_LEN + 1 - len(self.binary_data))
            
            return (bytearray([0xFA, 0xFF, 0xFE]) + recv_data + self.binary_data)
            # return (self.binary_data[:-1])
        self.client_socket.close()
        sys.exit(0)

    def parse(self, bin_data):
        bin_len = len(bin_data)  # in checksum(1byte)
        total_parsed_len = 0
        sensor = dict()
        IMU = {}
        sensor = {'Joint': {}, 'Insole': {}, 'Tension': {}}
        while total_parsed_len < bin_len:
            SensorID = struct.unpack("> H", bin_data[total_parsed_len:total_parsed_len+2])
                                
            # print("SensorID : 0x%0X" % SensorID)
            total_parsed_len += 2
            parsed_len = 0
            # SensorID 확인 후 각 센서별 파싱 함수로 파싱
            if (SensorID[0] == 0x4958):  # IMU Sensor
                # print("IMU Sensor : 0x%0X" % SensorID)
                parsed_len, IMU = self.XsensParser(bin_data[total_parsed_len:])
                

            elif (SensorID[0] == 0x4a45):  # Joint Encoder
                # print("Joint Encoder : 0x%0X" % SensorID)
                parsed_len, sensor['Joint'] = self.JointParser(bin_data[total_parsed_len:])
                

            elif (SensorID[0] == 0x6949):  # Insole
                # print("Insole : 0x%0X" % SensorID)
                parsed_len, sensor['Insole'] = self.InsoleParser(bin_data[total_parsed_len:])
                

            elif (SensorID[0] == 0x5445):  # Tension
                # print("Tension : 0x%0X" % SensorID)
                parsed_len, sensor['Tension'] = self.TensionParser(bin_data[total_parsed_len:])
                
            else:
                print(" Unknown SensorID : ", hex(SensorID[0]))
                break
            
            total_parsed_len += parsed_len
        # print("parsed_len: ", total_parsed_len, "\r\nMSG_LEN(NoCheckSum): ", bin_len)
        # print("======================")

        return IMU, sensor

    def JointParser(self, bin_data):
        parsed_len = 0
        # l = len(bin_data)
        # str_val = bin2hex(bin_data)
        # print("JointParser : bin_len = {}, {}".format(l, str_val))

        recv_data = bin_data[parsed_len: parsed_len + 1]
        parsed_len += 1

        # print("JointParser : NumberOfSensor : {}".format(recv_data[0]))
        sen = {}
        for CNT in range(0, recv_data[0]):
            recv_data = bin_data[parsed_len: parsed_len + 10]
            parsed_len += 10

            # print(recv_data)

            unpacked_recv_data = struct.unpack("> H i i", recv_data)
            data = np.asarray(unpacked_recv_data, dtype=Joint_Data_type)
            sen[int(data['LID'])] = data
        return parsed_len, sen

    def TensionParser(self, bin_data):
        parsed_len = 0
        # l = len(bin_data)
        # str_val = bin2hex(bin_data)
        # print("TensionParser : bin_len = {}, {}".format(l, str_val))

        recv_data = bin_data[parsed_len: parsed_len + 1]
        parsed_len += 1

        # print("TensionParser : NumberOfSensor : {}".format(recv_data[0]))
        sen = {}
        for CNT in range(0, recv_data[0]):
            recv_data = bin_data[parsed_len: parsed_len + 6]
            parsed_len += 6

            # print(recv_data)

            unpacked_recv_data = struct.unpack("> h f", recv_data)
            data = np.asarray(unpacked_recv_data, dtype=Tension_Data_type)

            # print("LID : {:X}".format(data['LID']))
            # print("Data : {:f}".format(data['Data']))
            sen[int(data['LID'])] = data
        return parsed_len, sen

    def XsensParser(self, bin_data):
        parsed_len = 0
        # l = len(bin_data)
        # str_val = bin2hex(bin_data[0:32])
        # print("XsensParser : bin_len = {}, {}".format(l, str_val))
        recv_data = bin_data[parsed_len:parsed_len + 1]
        parsed_len += 1
        
        imu = {}
        # print("XsensParser : NumberOfSensor : {}".format(recv_data[0]))
        for CNT in range(0, recv_data[0]):
            recv_data = bin_data[parsed_len:parsed_len + self.SENSOR_HEADER_LEN]
            parsed_len += self.SENSOR_HEADER_LEN

            unpacked_recv_data = struct.unpack("> h B B B B", recv_data)
            sensor_header = np.asarray(unpacked_recv_data, dtype=Sensor_Header_type)

            # print("SensorLocationID : {:X}".format(sensor_header['SensorID']))
            # print("Premble : {:X}".format(sensor_header['Premble']))
            # print("BID : {:X}".format(sensor_header['BID']))
            # print("MID : {:d}".format(sensor_header['MID']))
            # print("LEN : {:d}".format(sensor_header['LEN']))

            # 센서 헤더의 시작이 FA FF가 아니면 패스한다
            if (sensor_header['Premble'] != 0xFA) and (sensor_header['BID'] != 0xFF):
                return

            reading_len = 0
            while reading_len < sensor_header['LEN']:
                recv_data = bin_data[parsed_len + reading_len:parsed_len + reading_len + self.DATA_HEADER_LEN]
                reading_len += self.DATA_HEADER_LEN
                unpacked_recv_data = struct.unpack("> H B ", recv_data)
                data_header = np.asarray(unpacked_recv_data, dtype=Data_Header_type)
                # print("XDI : {:X}".format(data_header['XDI']))
                # print("Sensor Len : {:d}".format(data_header['Len']))
                
                recv_data = bin_data[parsed_len + reading_len : parsed_len + reading_len + data_header['Len']]

                # reading_len += data_header['Len'].astype(np.int32)
                reading_len += data_header['Len']

                if data_header['XDI'] == 0x1010:
                    unpacked_recv_data = struct.unpack("> I HBB BBB B ", recv_data)
                    # print(unpacked_recv_data)
                    utc = np.asarray(unpacked_recv_data, dtype=UTC_Data_type)
                # print('utc: ', utc['Data_YY'], '/', utc['Data_MM'], '/', utc['Data_DD'], ' ', utc['Data_hh'], ':', utc['Data_mm'], ':', utc['Data_ss'], '.', utc['Data_NS'])
                # utc = datetime.datetime(utc['Data_YY'], utc['Data_MM'], utc['Data_DD'], utc['Data_hh'], utc['Data_mm'], utc['Data_ss'], int(utc['Data_NS']*0.001)).timestamp()
                elif data_header['XDI'] == 0x2010:
                    unpacked_recv_data = struct.unpack("> ffff ", recv_data)
                    quat = np.asarray(unpacked_recv_data, dtype=Quat_Data_type)
                    # print("Quaternion: {:f}\t{:f}\t{:f}\t{:f}".format(quat['Data_Qw'], quat['Data_Qx'], quat['Data_Qy'], quat['Data_Qz']))
                elif data_header['XDI'] == 0x2030:
                    unpacked_recv_data = struct.unpack("> fff ", recv_data)
                    euler = np.asarray(unpacked_recv_data, dtype=Euler_Data_type)
                # print("Euler: {:f}\t{:f}\t{:f}".format(euler['Data_roll'], euler['Data_pitch'], euler['Data_yaw']))
                elif data_header['XDI'] == 0x4020:
                    unpacked_recv_data = struct.unpack("> fff ", recv_data)
                    acc = np.asarray(unpacked_recv_data, dtype=Acc_Data_type)
                # print("Acc: {:f}\t{:f}\t{:f}".format(acc['Data_Ax'], acc['Data_Ay'], acc['Data_Az']))
                elif data_header['XDI'] == 0x8020:
                    unpacked_recv_data = struct.unpack("> fff ", recv_data)
                    gyro = np.asarray(unpacked_recv_data, dtype=Gyro_Data_type)
                # print("Gyro: {:f}\t{:f}\t{:f}".format(gyro['Data_Gx'], gyro['Data_Gy'], gyro['Data_Gz']))
                elif data_header['XDI'] == 0xC020:
                    unpacked_recv_data = struct.unpack("> fff ", recv_data)
                    mag = np.asarray(unpacked_recv_data, dtype=Mag_Data_type)
                # print("Mag: {:f}\t{:f}\t{:f}".format(mag['Data_Mx'], mag['Data_My'], mag['Data_Mz']))
                elif data_header['XDI'] == 0xE020:
                    # print("Status:", recv_data)

                    # Checksum
                    recv_data = bin_data[parsed_len + reading_len: parsed_len + reading_len + 1]
                    reading_len += 1
                    # print('Checksum: 0x%02X'% recv_data[0])
            # print()
            parsed_len += reading_len

            imu[int(sensor_header['SensorID'])] = [utc, quat, euler, acc, gyro, mag]
            # print(imu)
        return parsed_len, imu

    def InsoleParser(self, bin_data):
        parsed_len = 0
        # l = len(bin_data)
        # str_val = bin2hex(bin_data[0:32])
        # print("InsoleParser : bin_len = {}, {}".format(l, str_val))
        recv_data = bin_data[parsed_len:parsed_len + 1]
        parsed_len += 1
        sen = {}
        # IMU = {}
        # print("InsoleParser NumberOfSensor : {}".format(recv_data[0]))
        for cnt in range(0, recv_data[0]):
            recv_data = bin_data[parsed_len:parsed_len + 8]
            parsed_len += 8

            unpacked_recv_data = struct.unpack("> H H B B B B", recv_data)
            sensor_header = np.asarray(unpacked_recv_data, dtype=InsoleSensor_Header_type)

            # print("SensorID : {:X}".format(sensor_header['SensorID']))
            # print("Premble : {:X}".format(sensor_header['Premble']))
            # print("DID : {:X}".format(sensor_header['DID']))
            # print("MID : {:X}".format(sensor_header['MID']))
            # print("LEN : {:X}".format(sensor_header['LEN']))

            # 센서 헤더의 시작이 FA FF가 아니면 패스한다
            if (sensor_header['Premble'] != 0xFFFA):
                return

            recv_data = bin_data[parsed_len:parsed_len + 2]
            parsed_len += 2

            # print(recv_data)

            recv_data = bin_data[parsed_len:parsed_len + 58]
            parsed_len += 58

            unpacked_recv_data = struct.unpack("< h ffff fff fff ffff ", recv_data)
            data = np.asarray(unpacked_recv_data, dtype=Insole_Data_type)

            # print("FSR: {:f}\t{:f}\t{:f}\t{:f}".format(data['FSR_T'], data['FSR_B'], data['FSR_M'], data['FSR_H']))
            # print("Acc: {:f}\t{:f}\t{:f}".format(data['Data_Ax'], data['Data_Ay'], data['Data_Az']))
            # print("Gyro: {:f}\t{:f}\t{:f}".format(data['Data_Gx'], data['Data_Gy'], data['Data_Gz']))
            # print("Quaternion: {:f}\t{:f}\t{:f}\t{:f}".format(data['Data_Qw'], data['Data_Qx'], data['Data_Qy'], data['Data_Qz']))
            # print()

            recv_data = bin_data[parsed_len:parsed_len + 1]
            parsed_len += 1
            # print("CKSUM: ", recv_data)

            recv_data = bin_data[parsed_len:parsed_len + 1]
            parsed_len += 1
            # print("EOM: ", recv_data)
            sen[int(sensor_header['SensorID'])] = data

        return parsed_len, sen

    def find_sync_code(self, p_socket_obj):
        # sync_code= [0xFA, 0xFF, 0xFE]
        check_code = [0x00, 0x00, 0x00]
        count = 0
        while not (np.array_equal(self.sync_code, check_code)):
            check_code = np.roll(check_code, -1)
            read_data = []
            read_data = p_socket_obj.recv(1)
            # print("{:x} ".format(read_data[0]))
            # print(".", end =" ")
            check_code[2] = read_data[0]
            # print(check_code)
            count = count + 1
        # print("CHECK_CODE = {:X} {:X} {:X}".format(check_code[0], check_code[1], check_code[2]))
        return count

def bin2hex(b, split = 32):
    str_val = ""
    for i in range(0, len(b)):
        if (i % split) == 0 and i != 0:
            str_val += '\n'
        str_val += "".join(' 0x%02x' % b[i])
    return str_val


if __name__ == '__main__':
    try:
        # host = '192.168.1.125'
        host = '192.168.2.75'
        port = 7793
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.settimeout(5)
        client_socket.connect((host, port))
        client_socket.settimeout(None)
        print("Connect to host({}:{}) \r\n".format(host, port))
    except socket.error as e:
        print("error")

    collector = ETRI_client()

    for x in range(0,10):

        recv_data = collector.recv_binary(client_socket)
        l = len(recv_data)
        str_val = bin2hex(recv_data[0:])
        # print("bin_len = {}\n{}".format(l, str_val))
        
        IMU, Sensor = collector.parse(recv_data[collector.SENSOR_OFFSET:-1])
        # print(IMU)
        print("\nIMU Data ================================================ ")
        for key, val in IMU.items():
            print('SensorLocationID = 0x%02X'%key)
            UTC = val[0]
            QUAT = val[1]
            EULER = val[2]
            ACC = val[3]
            GYRO = val[4]
            MAG = val[5]
            print('  UTC: ', UTC['Data_YY'], '/', UTC['Data_MM'], '/', UTC['Data_DD'], ' ', UTC['Data_hh'], ':', UTC['Data_mm'], ':', UTC['Data_ss'], '.', UTC['Data_NS'])            
            print("  Quaternion: {:f}\t{:f}\t{:f}\t{:f}".format(QUAT['Data_Qw'], QUAT['Data_Qx'], QUAT['Data_Qy'], QUAT['Data_Qz']))
            print("  Euler: {:f}\t{:f}\t{:f}".format(EULER['Data_roll'], EULER['Data_pitch'], EULER['Data_yaw']))
            print("  Acc: {:f}\t{:f}\t{:f}".format(ACC['Data_Ax'], ACC['Data_Ay'], ACC['Data_Az']))
            print("  Gyro: {:f}\t{:f}\t{:f}".format(GYRO['Data_Gx'], GYRO['Data_Gy'], GYRO['Data_Gz']))
            print("  Mag: {:f}\t{:f}\t{:f}".format(MAG['Data_Mx'], MAG['Data_My'], MAG['Data_Mz']))

        # print(Sensor)
        print("Sensor Data ================================================ ")
        print('  SensorName = Joint(cnt=%d)' % len(Sensor['Joint']))
        for key, val in Sensor['Joint'].items():
                print('\tLID= %c%c[0x%02X],' % (key >> 8, key & 0xFF, key), '\tTurn= {}, \tAngle= {}'.format(val['Turn'], val['Angle']))
                
        print('  SensorName = Tension(cnt=%d)' % len(Sensor['Tension']))
        for key, val in Sensor['Tension'].items():
                print('\tLID= %c%c[0x%02X],' % (key >> 8, key & 0xFF, key), '\Value= {}'.format(val['Data']))

        print('  SensorName = Insole(cnt=%d)'%len(Sensor['Insole']))
        # print('\tLocation', '\tPC','\tFSR_T','\tFSR_B','\tFSR_M', '\tFSR_H', '\tData_Ax','\tData_Ay','\tData_Az','\tData_Gx','\tData_Gy','\tData_Gz','\tData_Qw','\tData_Qx','\tData_Qy','\tData_Qz')
        print('\tLocation', '\tPC', '\tFSR_T', '\tFSR_B', '\tFSR_M', '\tFSR_H', '\tAx', '\tAy', '\tAz', '\tGx', '\tGy', '\tGz', '\tQw', '\tQx', '\tQy', '\tQz')
        for key, val in Sensor['Insole'].items():
            str = ''
            str += '\t%c%c[0x%02X]' % (key >> 8, key & 0xFF, key)
            str += '\t%d'%val['PC']
            str += '\t%f'%val['FSR_T']
            str += '\t%f'%val['FSR_B']
            str += '\t%f'%val['FSR_M']
            str += '\t%f'%val['FSR_H']
            str += '\t%f'%val['Data_Ax']
            str += '\t%f'%val['Data_Ay']
            str += '\t%f'%val['Data_Az']
            str += '\t%f'%val['Data_Gx']
            str += '\t%f'%val['Data_Gy']
            str += '\t%f'%val['Data_Gz']
            str += '\t%f'%val['Data_Qw']
            str += '\t%f'%val['Data_Qx']
            str += '\t%f'%val['Data_Qy']
            str += '\t%f'%val['Data_Qz']
            print(str)
            
       # print(val[]'PC','FSR_T','FSR_B','FSR_M', 'FSR_H', 'Data_Ax','Data_Ay','Data_Az','Data_Gx','Data_Gy','Data_Gz','Data_Qw','Data_Qx','Data_Qy','Data_Qz')

    sys.exit(1)
