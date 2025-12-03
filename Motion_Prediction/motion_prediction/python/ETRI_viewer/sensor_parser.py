# 2024.05.19 yjkim 파일 생성
# viewer 관련 소스코드에서 파싱 관련 부분을 별도 관리
# BinaryParser: 바이너리 파일을 텍스트로 파싱
# SensorParser: 각 센서의 raw 데이터를 파싱

import struct
import json

from ETRI_viewer.etri_parser import ETRI_client

import ETRI_viewer.evconfig as evconfig

class BinaryParser:
    def __init__(self):
        self.file_path = evconfig.config_file_path
        self.parsed_len = 0
        self.binary_data = b''
        self.config = self.read_config()
        self.contains_scale = self.config['XSENS_CONTAINS_SCALE']
    def read_config(self):
        with open(self.file_path,encoding='utf-8') as f:
            config = json.load(f)
        return config

    def parser(self, binary_data, XSENS_CONTAINS_SCALE_):
        self.binary_data = binary_data
        self.parsed_len = 32
        set_num = self.parsed_len
        xsens_raw_data = b''
        result = {}
        self.contains_scale=XSENS_CONTAINS_SCALE_
        while len(self.binary_data) > set_num:
            sensor_id = self.binary_data[set_num:set_num + 2]
            set_num += 2
            timestamp = struct.unpack('!32s', self.binary_data[0:32])[0]
            result["TS"] = timestamp

            if sensor_id == b'XS':
                result["xsens"], set_num, xsens_raw_data = SensorParser().xsens_parser(self.binary_data, set_num, self.contains_scale)
            elif sensor_id == b'OT':
                result["optitrack"], set_num = SensorParser().optitrack_parser(self.binary_data, set_num)
            elif sensor_id == b'FT':
                result["futek"], set_num = SensorParser().futek_parser(self.binary_data, set_num)
            elif sensor_id == b'DS':
                result["delsys"], set_num = SensorParser().delsys_parser(self.binary_data, set_num)
            elif sensor_id == b'FP':
                result["amti"], set_num = SensorParser().amti_parser(self.binary_data, set_num)
            elif sensor_id == b'CA':
                result["mono_camera"], set_num = SensorParser().mono_camera(self.binary_data, set_num)
            elif sensor_id == b'ES':
                result["etri_sensor"], set_num = SensorParser().etri_parser(self.binary_data, set_num)
            elif sensor_id == b'mm':
                result["fmgmmg"], set_num = SensorParser().fmgmmg_parser2(self.binary_data, set_num)
            elif sensor_id == b'SS':
                set_num -= 2
                result["softsensor"], set_num = SensorParser().softsensor_parser(self.binary_data, set_num)

        return result, xsens_raw_data


class SensorParser:
    def __init__(self):
        self.parsed_len = 0
        self.binary_data = b''
        self.config = self.read_config()
        self.softsensor_config = self.read_softsensor_config()

    def xsens_parser(self, binary_data, set_num, constrains_scale):
        # sensor, segment, scale 데이터 구조체 정의
        if constrains_scale:
            xsens_dict = {
                "sensor": [],
                "segment": [],
                "scale": []
            }
        else:
            xsens_dict = {
                "sensor": [],
                "segment": []
            }

        self.parsed_len = set_num
        self.binary_data = binary_data
        sensor_num = struct.unpack('!B', self.binary_data[self.parsed_len: self.parsed_len + 1])[0]
        self.parsed_len += 1

        # xsens 23개 센서 데이터를 순차적으로 읽음
        for i in range(23):
            xsens_data_list = []
            sensor_location_id = struct.unpack('2s', self.binary_data[self.parsed_len:self.parsed_len + 2])[0]
            self.parsed_len += 2
            xsens_data_list.append(int.from_bytes(sensor_location_id, byteorder='big'))
            num_xdi = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len + 1])[0]
            self.parsed_len += 1

            for x in range(num_xdi):
                xdi = struct.unpack('2s', self.binary_data[self.parsed_len:self.parsed_len + 2])[0]
                self.parsed_len += 2

                if xdi != b'OQ':
                    xdi_len_x = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len + 1])[0]
                    self.parsed_len += 1
                    xdi_value_x = struct.unpack('f', self.binary_data[self.parsed_len:self.parsed_len + xdi_len_x])[0]
                    self.parsed_len += xdi_len_x

                    xdi_len_y = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len + 1])[0]
                    self.parsed_len += 1
                    xdi_value_y = struct.unpack('f', self.binary_data[self.parsed_len:self.parsed_len + xdi_len_y])[0]
                    self.parsed_len += xdi_len_y

                    xdi_len_z = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len + 1])[0]
                    self.parsed_len += 1
                    xdi_value_z = struct.unpack('f', self.binary_data[self.parsed_len:self.parsed_len + xdi_len_z])[0]
                    self.parsed_len += xdi_len_z

                    xsens_data_list.append(xdi_value_x)
                    xsens_data_list.append(xdi_value_y)
                    xsens_data_list.append(xdi_value_z)
                else:
                    xdi_len_q0 = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len + 1])[0]
                    self.parsed_len += 1
                    xdi_value_q0 = struct.unpack('f', self.binary_data[self.parsed_len:self.parsed_len + xdi_len_q0])[0]
                    self.parsed_len += xdi_len_q0

                    xdi_len_q1 = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len + 1])[0]
                    self.parsed_len += 1
                    xdi_value_q1 = struct.unpack('f', self.binary_data[self.parsed_len:self.parsed_len + xdi_len_q1])[0]
                    self.parsed_len += xdi_len_q1

                    xdi_len_q2 = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len + 1])[0]
                    self.parsed_len += 1
                    xdi_value_q2 = struct.unpack('f', self.binary_data[self.parsed_len:self.parsed_len + xdi_len_q2])[0]
                    self.parsed_len += xdi_len_q2

                    xdi_len_q3 = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len + 1])[0]
                    self.parsed_len += 1
                    xdi_value_q3 = struct.unpack('f', self.binary_data[self.parsed_len:self.parsed_len + xdi_len_q3])[0]
                    self.parsed_len += xdi_len_q3

                    xsens_data_list.append(xdi_value_q0)
                    xsens_data_list.append(xdi_value_q1)
                    xsens_data_list.append(xdi_value_q2)
                    xsens_data_list.append(xdi_value_q3)

            xsens_dict['sensor'].append(xsens_data_list)
            # xsens_sensor_data_list.append(xsens_data_list)

        segment_num = struct.unpack('!B', self.binary_data[self.parsed_len: self.parsed_len + 1])[0]
        self.parsed_len += 1

        # xsens 세그먼트를 순차적으로 읽음
        for segment in range(segment_num):
            xsens_segment_data = []
            xsens_scale_data = []
            seg_location_id = struct.unpack('2s', self.binary_data[self.parsed_len:self.parsed_len + 2])[0]
            self.parsed_len += 2
            xsens_segment_data.append(int.from_bytes(seg_location_id, byteorder='big'))
            num_seg_xdi = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len + 1])[0]
            self.parsed_len += 1

            for seg_value in range(num_seg_xdi):
                seg_xdi = struct.unpack('2s', self.binary_data[self.parsed_len:self.parsed_len + 2])[0]
                self.parsed_len += 2

                # if seg_xdi != b'OQ':
                #     seg_value_x_len = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len+1])[0]
                #     self.parsed_len += 1
                #     seg_value_x = struct.unpack('f', self.binary_data[self.parsed_len:self.parsed_len+seg_value_x_len])[0]
                #     self.parsed_len += seg_value_x_len
                #
                #     seg_value_y_len = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len+1])[0]
                #     self.parsed_len += 1
                #     seg_value_y = struct.unpack('f', self.binary_data[self.parsed_len:self.parsed_len+seg_value_y_len])[0]
                #     self.parsed_len += seg_value_y_len
                #
                #     seg_value_z_len = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len+1])[0]
                #     self.parsed_len += 1
                #     seg_value_z = struct.unpack('f', self.binary_data[self.parsed_len:self.parsed_len+seg_value_z_len])[0]
                #     self.parsed_len += seg_value_z_len
                #
                #     xsens_segment_data.append(seg_value_x)
                #     xsens_segment_data.append(seg_value_y)
                #     xsens_segment_data.append(seg_value_z)
                #
                # else:
                #     seg_value_q0_len = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len+1])[0]
                #     self.parsed_len += 1
                #     seg_value_q0 = struct.unpack('f', self.binary_data[self.parsed_len:self.parsed_len+seg_value_q0_len])[0]
                #     self.parsed_len += seg_value_q0_len
                #
                #     seg_value_q1_len = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len+1])[0]
                #     self.parsed_len += 1
                #     seg_value_q1 = struct.unpack('f', self.binary_data[self.parsed_len:self.parsed_len+seg_value_q1_len])[0]
                #     self.parsed_len += seg_value_q1_len
                #
                #     seg_value_q2_len = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len+1])[0]
                #     self.parsed_len += 1
                #     seg_value_q2 = struct.unpack('f', self.binary_data[self.parsed_len:self.parsed_len+seg_value_q2_len])[0]
                #     self.parsed_len += seg_value_q2_len
                #
                #     seg_value_q3_len = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len+1])[0]
                #     self.parsed_len += 1
                #     seg_value_q3 = struct.unpack('f', self.binary_data[self.parsed_len:self.parsed_len+seg_value_q3_len])[0]
                #     self.parsed_len += seg_value_q3_len
                #
                #     xsens_segment_data.append(seg_value_q0)
                #     xsens_segment_data.append(seg_value_q1)
                #     xsens_segment_data.append(seg_value_q2)
                #     xsens_segment_data.append(seg_value_q3)

                if seg_xdi == b'OQ':
                    seg_value_q0_len = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len + 1])[0]
                    self.parsed_len += 1
                    seg_value_q0 = \
                    struct.unpack('f', self.binary_data[self.parsed_len:self.parsed_len + seg_value_q0_len])[0]
                    self.parsed_len += seg_value_q0_len

                    seg_value_q1_len = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len + 1])[0]
                    self.parsed_len += 1
                    seg_value_q1 = \
                    struct.unpack('f', self.binary_data[self.parsed_len:self.parsed_len + seg_value_q1_len])[0]
                    self.parsed_len += seg_value_q1_len

                    seg_value_q2_len = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len + 1])[0]
                    self.parsed_len += 1
                    seg_value_q2 = \
                    struct.unpack('f', self.binary_data[self.parsed_len:self.parsed_len + seg_value_q2_len])[0]
                    self.parsed_len += seg_value_q2_len

                    seg_value_q3_len = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len + 1])[0]
                    self.parsed_len += 1
                    seg_value_q3 = \
                    struct.unpack('f', self.binary_data[self.parsed_len:self.parsed_len + seg_value_q3_len])[0]
                    self.parsed_len += seg_value_q3_len

                    xsens_segment_data.append(seg_value_q0)
                    xsens_segment_data.append(seg_value_q1)
                    xsens_segment_data.append(seg_value_q2)
                    xsens_segment_data.append(seg_value_q3)

                # null_pose 관련 스케일 데이터 처리 부분
                # elif seg_xdi == b'SI':
                elif seg_xdi == b'SI' and constrains_scale is True:
                    seg_scale_name_len = struct.unpack('B', self.binary_data[self.parsed_len: self.parsed_len + 1])[0]
                    self.parsed_len += 1
                    seg_scale_name = struct.unpack('{}s'.format(seg_scale_name_len), self.binary_data[
                                                                                     self.parsed_len: self.parsed_len + seg_scale_name_len])[
                        0]
                    self.parsed_len += seg_scale_name_len

                    seg_scale_x_len = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len + 1])[0]
                    self.parsed_len += 1
                    seg_scale_x = struct.unpack('f', self.binary_data[self.parsed_len:self.parsed_len + 4])[0]
                    self.parsed_len += 4

                    seg_scale_y_len = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len + 1])[0]
                    self.parsed_len += 1
                    seg_scale_y = struct.unpack('f', self.binary_data[self.parsed_len:self.parsed_len + 4])[0]
                    self.parsed_len += 4

                    seg_scale_z_len = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len + 1])[0]
                    self.parsed_len += 1
                    seg_scale_z = struct.unpack('f', self.binary_data[self.parsed_len:self.parsed_len + 4])[0]
                    self.parsed_len += 4

                    xsens_scale_data.append(seg_scale_name)
                    xsens_scale_data.append(seg_scale_x)
                    xsens_scale_data.append(seg_scale_y)
                    xsens_scale_data.append(seg_scale_z)

                else:
                    seg_value_x_len = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len + 1])[0]
                    self.parsed_len += 1
                    seg_value_x = \
                    struct.unpack('f', self.binary_data[self.parsed_len:self.parsed_len + seg_value_x_len])[0]
                    self.parsed_len += seg_value_x_len

                    seg_value_y_len = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len + 1])[0]
                    self.parsed_len += 1
                    seg_value_y = \
                    struct.unpack('f', self.binary_data[self.parsed_len:self.parsed_len + seg_value_y_len])[0]
                    self.parsed_len += seg_value_y_len

                    seg_value_z_len = struct.unpack('B', self.binary_data[self.parsed_len:self.parsed_len + 1])[0]
                    self.parsed_len += 1
                    seg_value_z = \
                    struct.unpack('f', self.binary_data[self.parsed_len:self.parsed_len + seg_value_z_len])[0]
                    self.parsed_len += seg_value_z_len

                    xsens_segment_data.append(seg_value_x)
                    xsens_segment_data.append(seg_value_y)
                    xsens_segment_data.append(seg_value_z)

            xsens_dict['segment'].append(xsens_segment_data)
            if constrains_scale:  # yjkim, scale 처리 부분
                xsens_dict['scale'].append(xsens_scale_data)

        raw_num = struct.unpack('!B', self.binary_data[self.parsed_len:self.parsed_len + 1])[0]
        self.parsed_len += 1

        for r in range(raw_num):
            raw_id = struct.unpack('!6s', self.binary_data[self.parsed_len:self.parsed_len + 6])[0]
            self.parsed_len += 6
            raw_len = struct.unpack('!H', self.binary_data[self.parsed_len:self.parsed_len + 2])[0]
            self.parsed_len += 2 + raw_len

        raw_data = self.binary_data[self.parsed_len - raw_len * raw_num:self.parsed_len]

        return xsens_dict, self.parsed_len, raw_data

    def optitrack_parser(self, binary_data, ot_num):
        optitrack_dict = {
            "marker": [],
            "skeleton": []
        }
        marker_list = []
        skeleton_list = []
        sensor_num = struct.unpack('B', binary_data[ot_num:ot_num + 1])[0]
        ot_num += 1

        for i in range(sensor_num):
            # for i in range(self.config['OPTITRACK_MARKER_NUM']):
            marker_data = []
            marker_id = struct.unpack('2s', binary_data[ot_num:ot_num + 2])[0]
            ot_num += 2
            marker_data.append(int.from_bytes(marker_id, byteorder='big'))
            num_xdi = struct.unpack('B', binary_data[ot_num:ot_num + 1])[0]
            ot_num += 1

            xdi_x = struct.unpack('2s', binary_data[ot_num:ot_num + 2])[0]
            ot_num += 2
            xdi_x_len = struct.unpack('B', binary_data[ot_num:ot_num + 1])[0]
            ot_num += 1
            xdi_x_value = struct.unpack('f', binary_data[ot_num:ot_num + xdi_x_len])[0]
            ot_num += xdi_x_len

            xdi_y = struct.unpack('2s', binary_data[ot_num:ot_num + 2])[0]
            ot_num += 2
            xdi_y_len = struct.unpack('B', binary_data[ot_num:ot_num + 1])[0]
            ot_num += 1
            xdi_y_value = struct.unpack('f', binary_data[ot_num:ot_num + xdi_y_len])[0]
            ot_num += xdi_y_len

            xdi_z = struct.unpack('2s', binary_data[ot_num:ot_num + 2])[0]
            ot_num += 2
            xdi_z_len = struct.unpack('B', binary_data[ot_num:ot_num + 1])[0]
            ot_num += 1
            xdi_z_value = struct.unpack('f', binary_data[ot_num:ot_num + xdi_z_len])[0]
            ot_num += xdi_z_len

            marker_data.append(xdi_x_value)
            marker_data.append(xdi_y_value)
            marker_data.append(xdi_z_value)

            # marker_list.append(marker_data)
            optitrack_dict['marker'].append(marker_data)

        skeleton_num = struct.unpack('B', binary_data[ot_num:ot_num + 1])[0]
        ot_num += 1
        for x in range(skeleton_num):
            # for x in range (self.config['skeleton_num']):
            sk_data = []
            sk_id = struct.unpack('2s', binary_data[ot_num:ot_num + 2])[0]
            ot_num += 2
            sk_data.append(int.from_bytes(sk_id, byteorder='big'))
            num_sk_xdi = struct.unpack('B', binary_data[ot_num:ot_num + 1])[0]
            ot_num += 1

            px_xdi = struct.unpack('2s', binary_data[ot_num:ot_num + 2])[0]
            ot_num += 2
            px_len = struct.unpack('B', binary_data[ot_num:ot_num + 1])[0]
            ot_num += 1
            px_value = struct.unpack('f', binary_data[ot_num:ot_num + px_len])[0]
            ot_num += px_len

            py_xdi = struct.unpack('2s', binary_data[ot_num:ot_num + 2])[0]
            ot_num += 2
            py_len = struct.unpack('B', binary_data[ot_num:ot_num + 1])[0]
            ot_num += 1
            py_value = struct.unpack('f', binary_data[ot_num:ot_num + py_len])[0]
            ot_num += py_len

            pz_xdi = struct.unpack('2s', binary_data[ot_num:ot_num + 2])[0]
            ot_num += 2
            pz_len = struct.unpack('B', binary_data[ot_num:ot_num + 1])[0]
            ot_num += 1
            pz_value = struct.unpack('f', binary_data[ot_num:ot_num + pz_len])[0]
            ot_num += pz_len

            qw_xdi = struct.unpack('2s', binary_data[ot_num:ot_num + 2])[0]
            ot_num += 2
            qw_len = struct.unpack('B', binary_data[ot_num:ot_num + 1])[0]
            ot_num += 1
            qw_value = struct.unpack('f', binary_data[ot_num:ot_num + qw_len])[0]
            ot_num += qw_len

            qx_xdi = struct.unpack('2s', binary_data[ot_num:ot_num + 2])[0]
            ot_num += 2
            qx_len = struct.unpack('B', binary_data[ot_num:ot_num + 1])[0]
            ot_num += 1
            qx_value = struct.unpack('f', binary_data[ot_num:ot_num + qx_len])[0]
            ot_num += qx_len

            qy_xdi = struct.unpack('2s', binary_data[ot_num:ot_num + 2])[0]
            ot_num += 2
            qy_len = struct.unpack('B', binary_data[ot_num:ot_num + 1])[0]
            ot_num += 1
            qy_value = struct.unpack('f', binary_data[ot_num:ot_num + qy_len])[0]
            ot_num += qy_len

            qz_xdi = struct.unpack('2s', binary_data[ot_num:ot_num + 2])[0]
            ot_num += 2
            qz_len = struct.unpack('B', binary_data[ot_num:ot_num + 1])[0]
            ot_num += 1
            qz_value = struct.unpack('f', binary_data[ot_num:ot_num + qz_len])[0]
            ot_num += qz_len

            sk_data.append(px_value)
            sk_data.append(py_value)
            sk_data.append(pz_value)
            sk_data.append(qw_value)
            sk_data.append(qx_value)
            sk_data.append(qy_value)
            sk_data.append(qz_value)

            # skeleton_list.append(sk_data)
            optitrack_dict['skeleton'].append(sk_data)
        return optitrack_dict, ot_num

    def delsys_parser(self, binary_data, delsys_num):
        delsys_list = []
        sensor_num = struct.unpack('B', binary_data[delsys_num:delsys_num + 1])[0]
        delsys_num += 1

        for i in range(sensor_num):
            delsys_data = []
            sensor_location_id = struct.unpack('!2s', binary_data[delsys_num:delsys_num + 2])[0]
            delsys_num += 2
            delsys_value_len = struct.unpack('!B', binary_data[delsys_num:delsys_num + 1])[0]
            delsys_num += 1
            delsys_value = struct.unpack('!f', binary_data[delsys_num:delsys_num + delsys_value_len])[0]
            delsys_num += delsys_value_len

            delsys_data.append(int.from_bytes(sensor_location_id, byteorder='big'))
            delsys_data.append(delsys_value)
            delsys_list.append(delsys_data)

        return delsys_list, delsys_num

    def delsys_to_graph_parser(self, binary_data, delsys_num):
        delsys_list = []
        sensor_num = struct.unpack('B', binary_data[delsys_num:delsys_num + 1])[0]
        delsys_num += 1

        for i in range(sensor_num):
            delsys_data = []
            sensor_location_id = struct.unpack('!2s', binary_data[delsys_num:delsys_num + 2])[0]
            delsys_num += 2
            delsys_value_len = struct.unpack('!B', binary_data[delsys_num:delsys_num + 1])[0]
            delsys_num += 1
            delsys_value = struct.unpack('!f', binary_data[delsys_num:delsys_num + delsys_value_len])[0]
            delsys_num += delsys_value_len

            delsys_data.append(int.from_bytes(sensor_location_id, byteorder='big'))
            delsys_data.append(delsys_value * 10000)
            delsys_list.append(delsys_data)

        return delsys_list, delsys_num

    def futek_parser(self, binary_data, futek_num):
        futek_list = []
        sensor_num = struct.unpack('B', binary_data[futek_num:futek_num + 1])[0]
        futek_num += 1

        # for i in range(sensor_num):
        for i in range(self.config['FUTEK_SENSOR_NUM']):
            futek_data = []
            sensor_location_id = struct.unpack('!2s', binary_data[futek_num:futek_num + 2])[0]
            futek_num += 2
            sensor_len = struct.unpack('!B', binary_data[futek_num:futek_num + 1])[0]
            futek_num += 1
            sensor_value = struct.unpack('!f', binary_data[futek_num:futek_num + sensor_len])[0]
            futek_num += sensor_len

            futek_data.append(int.from_bytes(sensor_location_id, byteorder='big'))
            futek_data.append(sensor_value)
        futek_list.append(futek_data)

        return futek_list, futek_num

    def mono_camera(self, binary_data, mono_num):
        mono_camera_num = binary_data[mono_num:mono_num + 1]
        mono_num += 1
        sensor_id = struct.unpack('2s', binary_data[mono_num:mono_num + 2])
        mono_num += 2
        value_len = struct.unpack('I', binary_data[mono_num:mono_num + 4])[0]
        mono_num += 4

        mono_camera_data = binary_data[mono_num:mono_num + value_len]
        mono_num += value_len
        return mono_camera_data, mono_num

    def amti_parser(self, binary_data, amti_num):
        print(binary_data[amti_num:amti_num + 100])
        amti_list = []
        sensor_num = struct.unpack('!B', binary_data[amti_num:amti_num + 1])[0]
        amti_num += 1
        sensor_location_id = struct.unpack('!2s', binary_data[amti_num:amti_num + 2])[0]
        amti_num += 2

        num_xdi = struct.unpack('!B', binary_data[amti_num:amti_num + 1])[0]
        amti_num += 1

        for i in range(1):
            amti_data = []
            xdi_Fx = struct.unpack('!2s', binary_data[amti_num:amti_num + 2])[0]
            amti_num += 2
            xdi_Fx_len = struct.unpack('!B', binary_data[amti_num:amti_num + 1])[0]
            amti_num += 1
            xdi_Fx_value = struct.unpack('!f', binary_data[amti_num:amti_num + xdi_Fx_len])[0]
            amti_num += xdi_Fx_len

            xdi_Fy = struct.unpack('!2s', binary_data[amti_num:amti_num + 2])[0]
            amti_num += 2
            xdi_Fy_len = struct.unpack('!B', binary_data[amti_num:amti_num + 1])[0]
            amti_num += 1
            xdi_Fy_value = struct.unpack('!f', binary_data[amti_num:amti_num + xdi_Fy_len])[0]
            amti_num += xdi_Fy_len

            xdi_Fz = struct.unpack('!2s', binary_data[amti_num:amti_num + 2])[0]
            amti_num += 2
            xdi_Fz_len = struct.unpack('!B', binary_data[amti_num:amti_num + 1])[0]
            amti_num += 1
            xdi_Fz_value = struct.unpack('!f', binary_data[amti_num:amti_num + xdi_Fz_len])[0]
            amti_num += xdi_Fz_len

            xdi_mx = struct.unpack('!2s', binary_data[amti_num:amti_num + 2])[0]
            amti_num += 2
            xdi_mx_len = struct.unpack('!B', binary_data[amti_num:amti_num + 1])[0]
            amti_num += 1
            xdi_mx_value = struct.unpack('!f', binary_data[amti_num:amti_num + xdi_mx_len])[0]
            amti_num += xdi_mx_len

            xdi_my = struct.unpack('!2s', binary_data[amti_num:amti_num + 2])[0]
            amti_num += 2
            xdi_my_len = struct.unpack('!B', binary_data[amti_num:amti_num + 1])[0]
            amti_num += 1
            xdi_my_value = struct.unpack('!f', binary_data[amti_num:amti_num + xdi_my_len])[0]
            amti_num += xdi_my_len

            xdi_mz = struct.unpack('!2s', binary_data[amti_num:amti_num + 2])[0]
            amti_num += 2
            xdi_mz_len = struct.unpack('!B', binary_data[amti_num:amti_num + 1])[0]
            amti_num += 1
            xdi_mz_value = struct.unpack('!f', binary_data[amti_num:amti_num + xdi_mz_len])[0]
            amti_num += xdi_mz_len

            amti_data.append(int.from_bytes(xdi_Fx, byteorder='big'))
            amti_data.append(xdi_Fx_value)
            amti_data.append(xdi_Fy_value)
            amti_data.append(xdi_Fz_value)
            amti_data.append(xdi_mx_value)
            amti_data.append(xdi_my_value)
            amti_data.append(xdi_mz_value)

        amti_list.append(amti_data)

        return amti_list, amti_num

    def etri_parser(self, binary_data, etri_num):
        etri_dict = {
            "IMU_key": [],
            "IMU_datatime": [],
            "IMU_quater": [],
            "IMU_euler": [],
            "IMU_acc": [],
            "IMU_gyro": [],
            "IMU_mag": [],
            "joint_R": [],
            "joint_L": [],
            "insole_R_FSR": [],
            "insole_R_Acc": [],
            "insole_L_FSR": [],
            "insole_L_Acc": [],
            "insole_R": [],
            "insole_L": []
        }
        sensor_num = struct.unpack('B', binary_data[etri_num:etri_num + 1])
        etri_num += 1
        sensor_id = struct.unpack('2s', binary_data[etri_num:etri_num + 2])
        etri_num += 2
        sensor_value_len = struct.unpack('I', binary_data[etri_num:etri_num + 4])[0]
        etri_num += 4
        etri_binary = binary_data[etri_num:etri_num + sensor_value_len]
        etri_num += sensor_value_len

        IMU, sensor = ETRI_client().parse(etri_binary[7:-1])
        # print(sensor)

        for key in IMU:
            IMU_key = key
            IMU_datatime = IMU[key][0].item()
            IMU_quater = IMU[key][1].item()
            IMU_euler = IMU[key][2].item()
            IMU_acc = IMU[key][3].item()
            IMU_gyro = IMU[key][4].item()
            IMU_mag = IMU[key][5].item()

            etri_dict["IMU_key"].append(IMU_key)
            etri_dict["IMU_datatime"].append(IMU_datatime)
            etri_dict["IMU_quater"].append(IMU_quater)
            etri_dict["IMU_euler"].append(IMU_euler)
            etri_dict["IMU_acc"].append(IMU_acc)
            etri_dict["IMU_gyro"].append(IMU_gyro)
            etri_dict["IMU_mag"].append(IMU_mag)

        for sen_key in sensor:
            for sen_key2 in sensor[sen_key]:
                # Insole_R
                if hex(sen_key2) == "0x5246":
                    in_r = sensor[sen_key][sen_key2].item()
                    etri_dict["insole_R"].append(in_r)
                    etri_dict["insole_R_FSR"].append(in_r[1:5])
                    etri_dict["insole_R_Acc"].append(in_r[5:8])
                # Insole_L
                elif hex(sen_key2) == "0x4c46":
                    in_l = sensor[sen_key][sen_key2].item()
                    etri_dict["insole_L"].append(in_l)
                    etri_dict["insole_L_FSR"].append(in_l[1:5])
                    etri_dict["insole_L_Acc"].append(in_l[5:8])
                # joint_R
                elif hex(sen_key2) == "0x4c4e":
                    je_r = sensor[sen_key][sen_key2].item()
                    etri_dict["joint_R"].append(je_r)
                # joint_L
                elif hex(sen_key2) == "0x524e":
                    je_l = sensor[sen_key][sen_key2].item()
                    etri_dict["joint_L"].append(je_l)

        return etri_dict, etri_num

    def fmgmmg_parser(self, binary_data, FMG_num):
        FMG_dict = {
            "sensor_location_ID": [],
            "Ch1": [],
            "Ch2": [],
            "Ch3": [],
            "Ch4": [],
            "Ch5": [],
            "Ch6": [],
            "Ch7": [],
            "Ch8": [],
            "Ch9": [],
            "Ch10": [],
            "Ch11": [],
            "Ch12": [],
            "Ch13": [],
            "Ch14": [],
            "Ch15": [],
            "Ch16": [],
        }

        sensor_num = struct.unpack('< B', binary_data[FMG_num:FMG_num + 1])[0]
        FMG_num += 1
        sensor_id = struct.unpack('< H', binary_data[FMG_num:FMG_num + 2])
        FMG_num += 2

        if sensor_num == 1:
            sensor_location_id = struct.unpack('< H', binary_data[FMG_num:FMG_num + 2])[0]
            FMG_num += 2
            ch_data = struct.unpack('< 16f', binary_data[FMG_num:FMG_num + 64])

            FMG_num += 64

            FMG_dict["sensor_location_ID"].append(sensor_location_id)
            FMG_dict['Ch1'].append(ch_data[0])
            FMG_dict['Ch2'].append(ch_data[1])
            FMG_dict['Ch3'].append(ch_data[2])
            FMG_dict['Ch4'].append(ch_data[3])
            FMG_dict['Ch5'].append(ch_data[4])
            FMG_dict['Ch6'].append(ch_data[5])
            FMG_dict['Ch7'].append(ch_data[6])
            FMG_dict['Ch8'].append(ch_data[7])
            FMG_dict['Ch9'].append(ch_data[8])
            FMG_dict['Ch10'].append(ch_data[9])
            FMG_dict['Ch11'].append(ch_data[10])
            FMG_dict['Ch12'].append(ch_data[11])
            FMG_dict['Ch13'].append(ch_data[12])
            FMG_dict['Ch14'].append(ch_data[13])
            FMG_dict['Ch15'].append(ch_data[14])
            FMG_dict['Ch16'].append(ch_data[15])

        else:
            for _ in range(sensor_num):
                sensor_location_id = struct.unpack('< H', binary_data[FMG_num:FMG_num + 2])[0]
                FMG_num += 2
                ch_data = struct.unpack('< 16f', binary_data[FMG_num:FMG_num + 64])
                FMG_num += 64

                FMG_dict["sensor_location_ID"].append(sensor_location_id)
                FMG_dict['Ch1'].append(ch_data[0])
                FMG_dict['Ch2'].append(ch_data[1])
                FMG_dict['Ch3'].append(ch_data[2])
                FMG_dict['Ch4'].append(ch_data[3])
                FMG_dict['Ch5'].append(ch_data[4])
                FMG_dict['Ch6'].append(ch_data[5])
                FMG_dict['Ch7'].append(ch_data[6])
                FMG_dict['Ch8'].append(ch_data[7])
                FMG_dict['Ch9'].append(ch_data[8])
                FMG_dict['Ch10'].append(ch_data[9])
                FMG_dict['Ch11'].append(ch_data[10])
                FMG_dict['Ch12'].append(ch_data[11])
                FMG_dict['Ch13'].append(ch_data[12])
                FMG_dict['Ch14'].append(ch_data[13])
                FMG_dict['Ch15'].append(ch_data[14])
                FMG_dict['Ch16'].append(ch_data[15])

        return FMG_dict, FMG_num

    def fmgmmg_parser2(self, binary_data, FMG_num):
        FMG_list = []

        sensor_num = struct.unpack('< B', binary_data[FMG_num:FMG_num + 1])[0]
        FMG_num += 1

        if sensor_num == 1:
            FMG_mul_list = []
            sensor_data_id = struct.unpack('< H', binary_data[FMG_num:FMG_num + 2])[0]
            FMG_num += 2
            sensor_location_id = struct.unpack('< H', binary_data[FMG_num:FMG_num + 2])[0]
            FMG_num += 2
            ch_data = struct.unpack('< 16f', binary_data[FMG_num:FMG_num + 64])
            FMG_num += 64

            FMG_mul_list.append(sensor_location_id)
            for i in range(len(ch_data)):
                FMG_mul_list.append(ch_data[i])
            FMG_list.append(FMG_mul_list)

        else:
            for i in range(sensor_num):
                FMG_mul_list = []
                if i == 0:
                    sensor_data_id = struct.unpack('< H', binary_data[FMG_num:FMG_num + 2])[0]
                    FMG_num += 2
                sensor_location_id = struct.unpack('< H', binary_data[FMG_num:FMG_num + 2])[0]
                FMG_num += 2
                ch_data = struct.unpack('< 16f', binary_data[FMG_num:FMG_num + 64])
                FMG_num += 64

                FMG_mul_list.append(sensor_location_id)
                for i in range(len(ch_data)):
                    FMG_mul_list.append(ch_data[i])

                FMG_list.append(FMG_mul_list)

        return FMG_list, FMG_num

    def softsensor_parser(self, binary_data, softsensor_num):
        _ADS_ID_list = []
        _BNO_ID_list = []
        ADS_data_list = []
        BNO_data_list = []

        softsensor_data_list = []

        softsensor_dict = {}
        # binary_data = binary_data[:-1]
        test = softsensor_num

        sensor_port_num = 0
        group_num = 0

        while True:
            # print("binary sensor ID data : ", sensor_data[0:2])
            sensor_ID = binary_data[softsensor_num:softsensor_num + 2]
            softsensor_num += 2
            sensor_ID = struct.unpack('< H', sensor_ID)[0]

            if sensor_ID == 0x5353:  # ADS Soft Sensor
                attached_ads_num = struct.unpack('< B', binary_data[softsensor_num:softsensor_num + 1])[0] + 4
                softsensor_num += 1
                _ADS_num = attached_ads_num
                # print('ADS Num:', attached_ads_num)
                SOFT_PACKET_LENGTH = attached_ads_num * 10 + 3
                for i in range(attached_ads_num):
                    sensor_loc = struct.unpack('H', binary_data[softsensor_num:softsensor_num + 2])[0]
                    softsensor_num += 2
                    # print('ADS loc:', sensor_loc)
                    _ADS_ID_list.append(sensor_loc)
                    xy = struct.unpack('< f f', binary_data[softsensor_num:softsensor_num + 8])
                    softsensor_num += 8

            elif sensor_ID == 0x0808:  # BNO080 IMU Sensor
                attached_bno_num = struct.unpack('< B', binary_data[softsensor_num:softsensor_num + 1])[0]
                softsensor_num += 1
                _BNO_num = attached_bno_num
                BNO080_PACKET_LENGTH = attached_bno_num * 56
                for i in range(attached_bno_num):
                    sensor_loc = struct.unpack('< H', binary_data[softsensor_num:softsensor_num + 2])[0]
                    softsensor_num += 2
                    # print('IMU loc:', sensor_loc)
                    _BNO_ID_list.append(sensor_loc)
                    for i in range(3):
                        softsensor_num += 12
                    softsensor_num += 16

            if softsensor_num >= len(binary_data) - 1:
                # print("ADS_NUM : ",_ADS_num,  "BNO_NUM : ",_BNO_num)
                break

        softsensor_num = test
        while True:
            # 3. Sensor ID : 2byte만큼 unpack해서 Sensor_ID에 저장
            sensor_ID = binary_data[softsensor_num:softsensor_num + 2]
            softsensor_num += 2
            sensor_ID = struct.unpack('< H', sensor_ID)[0]

            # print("Sensor ID : ", sensor_ID)
            if sensor_ID == 0x5353:  # ADS Soft Sensor
                sensor_port_num += 1
                # 4. Number of Sensor : 1byte만큼 unpack해서 Number_of_Sensor에 저장
                number_of_Sensor = struct.unpack('< B', binary_data[softsensor_num:softsensor_num + 1])[0] + 4
                softsensor_num += 1
                # print('ADS Num:', number_of_Sensor)

                sens_num = 1
                for i in range(number_of_Sensor):
                    # 5 - 1. i번째 센서의 Sensor Location ID : 2byte만큼 unpack해서 저장
                    unpacked_data_loc = struct.unpack('H', binary_data[softsensor_num:softsensor_num + 2])[0]

                    # ADS_data_list.append(unpacked_data_loc)
                    softsensor_num += 2
                    # print(unpacked_data_loc)
                    # #        각 센서별 ID list에 추가
                    # _ADS_ID_list.append(unpacked_data_loc)

                    # 5 - 2. i번째 센서의 ADS_X : 4byte만큼 unpack해서 저장 (float)
                    # 5 - 3. i번째 센서의 ADS_Y : 4byte만큼 unpack해서 저장 (float)
                    xy = struct.unpack('< f f', binary_data[softsensor_num:softsensor_num + 8])
                    softsensor_num += 8

                    sens_num, config_name, group_num = self.softsensor_config_name("ADS", sens_num, i)
                    if self.softsensor_config['Group' + str(group_num)][config_name] == -1:
                        ADS_data_list.append(0)
                        ADS_data_list.append(0)
                    else:
                        ADS_data_list.append(xy[0])
                        ADS_data_list.append(xy[1])
                    # print(ADS_BUFFER[i])

            elif sensor_ID == 0x0808:  # BNO080 IMU Sensor
                # Dread number of seonor
                number_of_Sensor = struct.unpack('< B', binary_data[softsensor_num:softsensor_num + 1])[0]
                softsensor_num += 1
                # print('BNO Num:', number_of_Sensor)

                for i in range(number_of_Sensor):
                    sensor_loc = struct.unpack('< H', binary_data[softsensor_num:softsensor_num + 2])[0]
                    BNO_data_list.append(sensor_loc)
                    softsensor_num += 2
                    # print('IMU loc:', sensor_loc)

                    # BNO가 -1 일 때 데이터 값을 0으로 채움)
                    config_name, group_num = self.softsensor_config_name("BNO", 0, i)
                    if self.softsensor_config['Group' + str(group_num)][config_name] == -1:
                        for j in range(3):
                            IMU_9AXIS = struct.unpack('< f f f', binary_data[softsensor_num:softsensor_num + 12])
                            for x in range(len(IMU_9AXIS)):
                                BNO_data_list.append(0)
                            softsensor_num += 12
                        IMU_QUATERNION = struct.unpack('< f f f f', binary_data[softsensor_num:softsensor_num + 16])
                        for q in range(len(IMU_QUATERNION)):
                            BNO_data_list.append(0)
                        softsensor_num += 16

                    else:
                        for j in range(3):
                            IMU_9AXIS = struct.unpack('< f f f', binary_data[softsensor_num:softsensor_num + 12])
                            for x in range(len(IMU_9AXIS)):
                                BNO_data_list.append(IMU_9AXIS[x])
                            softsensor_num += 12
                        IMU_QUATERNION = struct.unpack('< f f f f', binary_data[softsensor_num:softsensor_num + 16])
                        for q in range(len(IMU_QUATERNION)):
                            BNO_data_list.append(IMU_QUATERNION[q])
                        softsensor_num += 16

            if softsensor_num >= len(binary_data) - 1:
                ADS_list = []
                BNO_list = []
                _ADS_ID_list = _ADS_ID_list[:_ADS_num]
                _BNO_ID_list = _BNO_ID_list[:_BNO_num]

                for AD in range(int(len(ADS_data_list) / 3)):
                    for i in range(1, 3):
                        ADS_list.append(ADS_data_list[AD * 3 + i])
                        # ADS_list.append(ADS_data_list[AD*3+1])
                        # ADS_list.append(ADS_data_list[AD*3+2])

                for BD in range(int(len(BNO_data_list) / 14)):
                    for i in range(1, 14):
                        BNO_list.append(BNO_data_list[BD * 14 + i])
                """
                for ID in (_ADS_ID_list):
                    for i in range(1, len(ADS_list)-1):
                        softsensor_dict()
                """

                for ID in (_ADS_ID_list):
                    for i in range(1, len(ADS_list) - 1):
                        softsensor_dict["ADS" + str(ID)] = ADS_list[i:i + 2]

                for ID in (_BNO_ID_list):
                    for i in range(1, len(BNO_list) - 13):
                        softsensor_dict["BNO" + str(ID)] = BNO_list[i:i + 14]

                for i in range(int(len(_ADS_ID_list) / 6)):
                    id = i * 6

                    bno_start = i * 14 + 1
                    bno_end = (i + 1) * 14

                    xy_value = [ADS_data_list[i:i + 12] for i in range(0, len(ADS_data_list), 12)]

                    softsensor_data_list.append([_ADS_ID_list[id]] + BNO_data_list[bno_start:bno_end] + xy_value[i])

                return softsensor_data_list, softsensor_num

                # ADS_dict = {}
                # BNO_dict = {}
                # softsensor_test_list = []
                #
                # for i in range(len(_ADS_ID_list)):
                #     ADS_dict[_ADS_ID_list[i]] = ADS_data_list[i*2:(i*2)+2]
                #
                # for i in range(len(_BNO_ID_list)):
                #     BNO_dict[_BNO_ID_list[i]] = BNO_data_list[i*14+1:(i*14)+14]
                #
                # softsensor_config = self.read_softsensor_config()
                # group_name_list = list(softsensor_config.keys())
                #
                # for group_name in group_name_list:
                #     config_key_list = list(softsensor_config[group_name].keys())
                #
                #     for config_key in config_key_list:
                #         if config_key == 'Port_BNO':
                #             for i in range(len(_BNO_ID_list)):
                #                 if softsensor_config[group_name]['Port_BNO'] ==  _BNO_ID_list[i]:
                #                     softsensor_test_list.append(BNO_dict[_BNO_ID_list[i]])
                #
                #         elif config_key != 'Port_Name':
                #             print(config_key)
                #
                # return softsensor_data_list, softsensor_num

    def softsensor_parser_csv(self, binary_data, softsensor_num):
        _ADS_ID_list = []
        _BNO_ID_list = []
        ADS_data_list = []
        BNO_data_list = []

        softsensor_data_list = []

        softsensor_dict = {}
        # binary_data = binary_data[:-1]
        test = softsensor_num

        sensor_port_num = 0
        group_num = 0

        while True:
            # print("binary sensor ID data : ", sensor_data[0:2])
            sensor_ID = binary_data[softsensor_num:softsensor_num + 2]
            softsensor_num += 2
            sensor_ID = struct.unpack('< H', sensor_ID)[0]

            if sensor_ID == 0x5353:  # ADS Soft Sensor
                attached_ads_num = struct.unpack('< B', binary_data[softsensor_num:softsensor_num + 1])[0] + 4
                softsensor_num += 1
                _ADS_num = attached_ads_num
                # print('ADS Num:', attached_ads_num)
                SOFT_PACKET_LENGTH = attached_ads_num * 10 + 3
                for i in range(attached_ads_num):
                    sensor_loc = struct.unpack('H', binary_data[softsensor_num:softsensor_num + 2])[0]
                    softsensor_num += 2
                    # print('ADS loc:', sensor_loc)
                    _ADS_ID_list.append(sensor_loc)
                    xy = struct.unpack('< f f', binary_data[softsensor_num:softsensor_num + 8])
                    softsensor_num += 8

            elif sensor_ID == 0x0808:  # BNO080 IMU Sensor
                attached_bno_num = struct.unpack('< B', binary_data[softsensor_num:softsensor_num + 1])[0]
                softsensor_num += 1
                _BNO_num = attached_bno_num
                BNO080_PACKET_LENGTH = attached_bno_num * 56
                for i in range(attached_bno_num):
                    sensor_loc = struct.unpack('< H', binary_data[softsensor_num:softsensor_num + 2])[0]
                    softsensor_num += 2
                    # print('IMU loc:', sensor_loc)
                    _BNO_ID_list.append(sensor_loc)
                    for i in range(3):
                        softsensor_num += 12
                    softsensor_num += 16

            if softsensor_num >= len(binary_data) - 1:
                # print("ADS_NUM : ",_ADS_num,  "BNO_NUM : ",_BNO_num)
                break

        softsensor_num = test
        while True:
            # 3. Sensor ID : 2byte만큼 unpack해서 Sensor_ID에 저장
            sensor_ID = binary_data[softsensor_num:softsensor_num + 2]
            softsensor_num += 2
            sensor_ID = struct.unpack('< H', sensor_ID)[0]

            # print("Sensor ID : ", sensor_ID)
            if sensor_ID == 0x5353:  # ADS Soft Sensor
                sensor_port_num += 1
                # 4. Number of Sensor : 1byte만큼 unpack해서 Number_of_Sensor에 저장
                number_of_Sensor = struct.unpack('< B', binary_data[softsensor_num:softsensor_num + 1])[0]
                softsensor_num += 1
                # print('ADS Num:', number_of_Sensor)

                sens_num = 1
                for i in range(number_of_Sensor):
                    # 5 - 1. i번째 센서의 Sensor Location ID : 2byte만큼 unpack해서 저장
                    unpacked_data_loc = struct.unpack('H', binary_data[softsensor_num:softsensor_num + 2])[0]

                    # ADS_data_list.append(unpacked_data_loc)
                    softsensor_num += 2
                    # print(unpacked_data_loc)
                    # #        각 센서별 ID list에 추가
                    # _ADS_ID_list.append(unpacked_data_loc)

                    # 5 - 2. i번째 센서의 ADS_X : 4byte만큼 unpack해서 저장 (float)
                    # 5 - 3. i번째 센서의 ADS_Y : 4byte만큼 unpack해서 저장 (float)
                    xy = struct.unpack('< f f', binary_data[softsensor_num:softsensor_num + 8])
                    softsensor_num += 8

                    sens_num, config_name, group_num = self.softsensor_config_name("ADS", sens_num, i)
                    if self.softsensor_config['Group' + str(group_num)][config_name] == -1:
                        ADS_data_list.append(0)
                        # ADS_data_list.append(0)
                    else:
                        ADS_data_list.append(xy[0])
                        ADS_data_list.append(xy[1])
                    # print(ADS_BUFFER[i])

            elif sensor_ID == 0x0808:  # BNO080 IMU Sensor
                # Dread number of seonor
                number_of_Sensor = struct.unpack('< B', binary_data[softsensor_num:softsensor_num + 1])[0]
                softsensor_num += 1
                # print('BNO Num:', number_of_Sensor)

                for i in range(number_of_Sensor):
                    sensor_loc = struct.unpack('< H', binary_data[softsensor_num:softsensor_num + 2])[0]
                    BNO_data_list.append(sensor_loc)
                    softsensor_num += 2
                    # print('IMU loc:', sensor_loc)

                    # BNO가 -1 일 때 데이터 값을 0으로 채움)
                    config_name, group_num = self.softsensor_config_name("BNO", 0, i)
                    if self.softsensor_config['Group' + str(group_num)][config_name] == -1:
                        for j in range(3):
                            IMU_9AXIS = struct.unpack('< f f f', binary_data[softsensor_num:softsensor_num + 12])
                            for x in range(len(IMU_9AXIS)):
                                BNO_data_list.append(0)
                            softsensor_num += 12
                        IMU_QUATERNION = struct.unpack('< f f f f', binary_data[softsensor_num:softsensor_num + 16])
                        for q in range(len(IMU_QUATERNION)):
                            BNO_data_list.append(0)
                        softsensor_num += 16

                    else:
                        for j in range(3):
                            IMU_9AXIS = struct.unpack('< f f f', binary_data[softsensor_num:softsensor_num + 12])
                            for x in range(len(IMU_9AXIS)):
                                BNO_data_list.append(IMU_9AXIS[x])
                            softsensor_num += 12
                        IMU_QUATERNION = struct.unpack('< f f f f', binary_data[softsensor_num:softsensor_num + 16])
                        for q in range(len(IMU_QUATERNION)):
                            BNO_data_list.append(IMU_QUATERNION[q])
                        softsensor_num += 16

            if softsensor_num >= len(binary_data) - 1:
                ADS_list = []
                BNO_list = []
                _ADS_ID_list = _ADS_ID_list[:_ADS_num]
                _BNO_ID_list = _BNO_ID_list[:_BNO_num]

                for AD in range(int(len(ADS_data_list) / 3)):
                    for i in range(1, 3):
                        ADS_list.append(ADS_data_list[AD * 3 + i])
                        # ADS_list.append(ADS_data_list[AD*3+1])
                        # ADS_list.append(ADS_data_list[AD*3+2])

                for BD in range(int(len(BNO_data_list) / 14)):
                    for i in range(1, 14):
                        BNO_list.append(BNO_data_list[BD * 14 + i])

                # for ID in (_ADS_ID_list):
                #     for i in range(1, len(ADS_list)-1):
                #         softsensor_dict()

                for ID in (_ADS_ID_list):
                    for i in range(1, len(ADS_list) - 1):
                        softsensor_dict["ADS" + str(ID)] = ADS_list[i:i + 2]

                for ID in (_BNO_ID_list):
                    for i in range(1, len(BNO_list) - 13):
                        softsensor_dict["BNO" + str(ID)] = BNO_list[i:i + 14]

                return softsensor_dict, softsensor_num

    def read_config(self):
        file_path = evconfig.config_file_path
        with open(file_path, encoding='utf-8') as f:
        #with open(file_path) as f:
            config = json.load(f)
        return config

    def read_softsensor_config(self):
        file_path = evconfig.config_file_path
        with open(file_path, encoding='utf-8') as f:
            softsensor_config = json.load(f)
        return softsensor_config

    def softsensor_config_name(self, sort, sens_num, i):
        group_num = 1
        if sort == "ADS":

            if sens_num > 6:
                sens_num = 1

            # if 0 <= i <= 3:
            #     group_num = 1
            #     config_name = "Port_ADS" + str(sens_num)
            #     sens_num += 1
            # elif 4 <= i <= 7:
            #     group_num = 2
            #     config_name = "Port_ADS" + str(sens_num)
            #     sens_num += 1
            # elif 8 <= i <= 11:
            #     group_num = 3
            #     config_name = "Port_ADS" + str(sens_num)
            #     sens_num += 1
            # elif 12 <= i <= 15:
            #     group_num = 4
            #     config_name = "Port_ADS" + str(sens_num)
            #     sens_num += 1

            if 6 < sens_num < 12:
                sens_num = 1
                group_num += 1
            elif 12 < sens_num < 18:
                sens_num = 1
                group_num += 2
            elif 18 < sens_num < 24:
                sens_num = 1
                group_num += 3

            config_name = "Port_ADS" + str(sens_num)
            sens_num += 1

            return sens_num, config_name, group_num

        elif sort == "BNO":
            i += 1
            group_num = i
            config_name = "Port_BNO"
            return config_name, group_num
