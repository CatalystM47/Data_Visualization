import os
import struct
import pandas as pd
import numpy as np
from .base_parser import BaseParser, FlightData


class ArduLogParser(BaseParser):
    """Parser for ArduPilot/Mission Planner logs (.bin, .log)"""

    MSG_FORMATS = {
        'ATT': ['TimeUS', 'DesRoll', 'Roll', 'DesPitch', 'Pitch', 'DesYaw', 'Yaw', 'ErrRP', 'ErrYaw'],
        'RATE': ['TimeUS', 'RDes', 'R', 'ROut', 'PDes', 'P', 'POut', 'YDes', 'Y', 'YOut',
                 'ADes', 'A', 'AOut'],
        'IMU': ['TimeUS', 'GyrX', 'GyrY', 'GyrZ', 'AccX', 'AccY', 'AccZ'],
        'RCIN': ['TimeUS', 'C1', 'C2', 'C3', 'C4', 'C5', 'C6', 'C7', 'C8'],
        'RCOU': ['TimeUS', 'C1', 'C2', 'C3', 'C4', 'C5', 'C6', 'C7', 'C8'],
        'BAT': ['TimeUS', 'Volt', 'VoltR', 'Curr', 'CurrTot', 'EnrgTot', 'Temp', 'Res'],
        'GPS': ['TimeUS', 'Status', 'GMS', 'GWk', 'NSats', 'HDop', 'Lat', 'Lng', 'Alt',
                'Spd', 'GCrs', 'VZ', 'Yaw', 'U'],
        'PARM': ['TimeUS', 'Name', 'Value'],
        'PIDR': ['TimeUS', 'Tar', 'Act', 'Err', 'P', 'I', 'D', 'FF', 'Dmod', 'SRate', 'Limit'],
        'PIDP': ['TimeUS', 'Tar', 'Act', 'Err', 'P', 'I', 'D', 'FF', 'Dmod', 'SRate', 'Limit'],
        'PIDY': ['TimeUS', 'Tar', 'Act', 'Err', 'P', 'I', 'D', 'FF', 'Dmod', 'SRate', 'Limit'],
        'VIBE': ['TimeUS', 'VibeX', 'VibeY', 'VibeZ', 'Clip0', 'Clip1', 'Clip2'],
    }

    def can_parse(self, filepath: str) -> bool:
        ext = filepath.lower()
        if ext.endswith('.bin'):
            return self._is_ardubin(filepath)
        if ext.endswith('.log'):
            return self._is_ardulog(filepath)
        return False

    def _is_ardubin(self, filepath: str) -> bool:
        try:
            with open(filepath, 'rb') as f:
                magic = f.read(3)
                return magic[:2] == b'\xa3\x95'
        except Exception:
            return False

    def _is_ardulog(self, filepath: str) -> bool:
        try:
            with open(filepath, 'r', errors='ignore') as f:
                for _ in range(20):
                    line = f.readline()
                    if any(msg in line for msg in ['ATT,', 'IMU,', 'RATE,', 'FMT,']):
                        return True
            return False
        except Exception:
            return False

    def parse(self, filepath: str) -> FlightData:
        if filepath.lower().endswith('.bin'):
            return self._parse_bin(filepath)
        else:
            return self._parse_text_log(filepath)

    def _parse_text_log(self, filepath: str) -> FlightData:
        messages = {}
        parameters = {}

        with open(filepath, 'r', errors='ignore') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                parts = line.split(',')
                if len(parts) < 2:
                    continue

                msg_type = parts[0].strip()
                if msg_type == 'PARM' and len(parts) >= 3:
                    name = parts[1].strip()
                    try:
                        value = float(parts[2].strip())
                        parameters[name] = value
                    except ValueError:
                        parameters[parts[1].strip()] = parts[2].strip()
                elif msg_type in self.MSG_FORMATS:
                    if msg_type not in messages:
                        messages[msg_type] = []
                    values = []
                    for p in parts[1:]:
                        try:
                            values.append(float(p.strip()))
                        except ValueError:
                            values.append(p.strip())
                    messages[msg_type].append(values)

        return self._build_flight_data(messages, parameters)

    def _parse_bin(self, filepath: str) -> FlightData:
        messages = {}
        parameters = {}
        fmt_map = {}

        with open(filepath, 'rb') as f:
            data = f.read()

        pos = 0
        while pos < len(data) - 2:
            if data[pos] != 0xA3 or data[pos+1] != 0x95:
                pos += 1
                continue

            if pos + 3 > len(data):
                break
            msg_type_id = data[pos + 2]
            pos += 3

            if msg_type_id == 128:  # FMT message
                if pos + 86 > len(data):
                    break
                fmt_data = data[pos:pos+86]
                fmt_type = fmt_data[0]
                fmt_len = fmt_data[1]
                fmt_name = fmt_data[2:6].decode('ascii', errors='ignore').strip('\x00')
                fmt_format = fmt_data[6:22].decode('ascii', errors='ignore').strip('\x00')
                fmt_labels = fmt_data[22:86].decode('ascii', errors='ignore').strip('\x00')
                fmt_map[fmt_type] = {
                    'name': fmt_name,
                    'length': fmt_len,
                    'format': fmt_format,
                    'labels': fmt_labels.split(','),
                }
                pos += 86
            elif msg_type_id in fmt_map:
                fmt = fmt_map[msg_type_id]
                msg_len = fmt['length'] - 3
                if pos + msg_len > len(data):
                    break
                msg_data = data[pos:pos+msg_len]
                pos += msg_len

                try:
                    values = self._decode_message(msg_data, fmt['format'])
                    msg_name = fmt['name']

                    if msg_name == 'PARM' and len(values) >= 2:
                        pname = str(values[1]) if len(values) > 1 else ''
                        pval = values[2] if len(values) > 2 else 0
                        if pname:
                            parameters[pname] = pval
                    elif msg_name in self.MSG_FORMATS:
                        if msg_name not in messages:
                            messages[msg_name] = []
                        messages[msg_name].append(values)
                except Exception:
                    continue
            else:
                pos += 1

        return self._build_flight_data(messages, parameters)

    def _decode_message(self, data, fmt_str):
        values = []
        offset = 0
        for c in fmt_str:
            try:
                if c == 'Q':  # uint64
                    val = struct.unpack_from('<Q', data, offset)[0]
                    offset += 8
                elif c == 'q':  # int64
                    val = struct.unpack_from('<q', data, offset)[0]
                    offset += 8
                elif c == 'I':  # uint32
                    val = struct.unpack_from('<I', data, offset)[0]
                    offset += 4
                elif c == 'i':  # int32
                    val = struct.unpack_from('<i', data, offset)[0]
                    offset += 4
                elif c == 'H':  # uint16
                    val = struct.unpack_from('<H', data, offset)[0]
                    offset += 2
                elif c == 'h':  # int16
                    val = struct.unpack_from('<h', data, offset)[0]
                    offset += 2
                elif c == 'B':  # uint8
                    val = struct.unpack_from('<B', data, offset)[0]
                    offset += 1
                elif c == 'b':  # int8
                    val = struct.unpack_from('<b', data, offset)[0]
                    offset += 1
                elif c == 'f':  # float
                    val = struct.unpack_from('<f', data, offset)[0]
                    offset += 4
                elif c == 'd':  # double
                    val = struct.unpack_from('<d', data, offset)[0]
                    offset += 8
                elif c == 'n':  # char[4]
                    val = data[offset:offset+4].decode('ascii', errors='ignore').strip('\x00')
                    offset += 4
                elif c == 'N':  # char[16]
                    val = data[offset:offset+16].decode('ascii', errors='ignore').strip('\x00')
                    offset += 16
                elif c == 'Z':  # char[64]
                    val = data[offset:offset+64].decode('ascii', errors='ignore').strip('\x00')
                    offset += 64
                elif c == 'c':  # int16 * 100
                    val = struct.unpack_from('<h', data, offset)[0] / 100.0
                    offset += 2
                elif c == 'C':  # uint16 * 100
                    val = struct.unpack_from('<H', data, offset)[0] / 100.0
                    offset += 2
                elif c == 'e':  # int32 * 100
                    val = struct.unpack_from('<i', data, offset)[0] / 100.0
                    offset += 4
                elif c == 'E':  # uint32 * 100
                    val = struct.unpack_from('<I', data, offset)[0] / 100.0
                    offset += 4
                elif c == 'L':  # int32 (lat/lon)
                    val = struct.unpack_from('<i', data, offset)[0]
                    offset += 4
                elif c == 'M':  # uint8 flight mode
                    val = struct.unpack_from('<B', data, offset)[0]
                    offset += 1
                else:
                    val = 0
                    offset += 1
                values.append(val)
            except struct.error:
                values.append(0)
                break
        return values

    def _build_flight_data(self, messages: dict, parameters: dict) -> FlightData:
        fd = FlightData(source_type='ardupilot', fc_type='pixhawk')
        fd.parameters = parameters
        fd.metadata['source'] = 'ArduPilot/Mission Planner'

        # PID parameters
        for axis, prefix in [('roll', 'ATC_RAT_RLL_'),
                             ('pitch', 'ATC_RAT_PIT_'),
                             ('yaw', 'ATC_RAT_YAW_')]:
            fd.pid_params[axis] = {
                'P': parameters.get(f'{prefix}P', 0),
                'I': parameters.get(f'{prefix}I', 0),
                'D': parameters.get(f'{prefix}D', 0),
            }

        # Attitude (ATT)
        if 'ATT' in messages and messages['ATT']:
            att_data = self._msg_to_df(messages['ATT'], self.MSG_FORMATS['ATT'])
            if att_data is not None:
                ts = self._normalize_time(att_data['TimeUS'].values, unit='us')
                fd.attitude = pd.DataFrame({
                    'roll': att_data['Roll'].values,
                    'pitch': att_data['Pitch'].values,
                    'yaw': att_data['Yaw'].values,
                }, index=ts)
                fd.attitude.index.name = 'time'

                fd.attitude_target = pd.DataFrame({
                    'roll': att_data['DesRoll'].values,
                    'pitch': att_data['DesPitch'].values,
                    'yaw': att_data['DesYaw'].values,
                }, index=ts)
                fd.attitude_target.index.name = 'time'

        # IMU (Gyro + Accel)
        if 'IMU' in messages and messages['IMU']:
            imu_data = self._msg_to_df(messages['IMU'], self.MSG_FORMATS['IMU'])
            if imu_data is not None:
                ts = self._normalize_time(imu_data['TimeUS'].values, unit='us')
                fd.gyro = pd.DataFrame({
                    'roll': imu_data['GyrX'].values,
                    'pitch': imu_data['GyrY'].values,
                    'yaw': imu_data['GyrZ'].values,
                }, index=ts)
                fd.gyro.index.name = 'time'

                fd.accel = pd.DataFrame({
                    'x': imu_data['AccX'].values,
                    'y': imu_data['AccY'].values,
                    'z': imu_data['AccZ'].values,
                }, index=ts)
                fd.accel.index.name = 'time'

        # PID outputs
        for msg_name, axis in [('PIDR', 'roll'), ('PIDP', 'pitch'), ('PIDY', 'yaw')]:
            if msg_name in messages and messages[msg_name]:
                pid_data = self._msg_to_df(messages[msg_name], self.MSG_FORMATS[msg_name])
                if pid_data is not None:
                    ts = self._normalize_time(pid_data['TimeUS'].values, unit='us')
                    pid_df = pd.DataFrame({
                        f'{axis}_p': pid_data['P'].values,
                        f'{axis}_i': pid_data['I'].values,
                        f'{axis}_d': pid_data['D'].values,
                    }, index=ts)
                    pid_df.index.name = 'time'
                    if fd.pid_output is None:
                        fd.pid_output = pid_df
                    else:
                        fd.pid_output = fd.pid_output.join(pid_df, how='outer')

        # RC input
        if 'RCIN' in messages and messages['RCIN']:
            rc_data = self._msg_to_df(messages['RCIN'], self.MSG_FORMATS['RCIN'])
            if rc_data is not None:
                ts = self._normalize_time(rc_data['TimeUS'].values, unit='us')
                fd.rc_input = pd.DataFrame({
                    'roll': rc_data['C1'].values,
                    'pitch': rc_data['C2'].values,
                    'throttle': rc_data['C3'].values,
                    'yaw': rc_data['C4'].values,
                }, index=ts)
                fd.rc_input.index.name = 'time'

        # Motor output
        if 'RCOU' in messages and messages['RCOU']:
            rc_data = self._msg_to_df(messages['RCOU'], self.MSG_FORMATS['RCOU'])
            if rc_data is not None:
                ts = self._normalize_time(rc_data['TimeUS'].values, unit='us')
                motor_dict = {}
                for i, col in enumerate(['C1', 'C2', 'C3', 'C4']):
                    if col in rc_data.columns:
                        motor_dict[f'motor{i}'] = rc_data[col].values
                fd.motor_output = pd.DataFrame(motor_dict, index=ts)
                fd.motor_output.index.name = 'time'

        # Battery
        if 'BAT' in messages and messages['BAT']:
            bat_data = self._msg_to_df(messages['BAT'], self.MSG_FORMATS['BAT'])
            if bat_data is not None:
                ts = self._normalize_time(bat_data['TimeUS'].values, unit='us')
                fd.battery = pd.DataFrame({
                    'voltage': bat_data['Volt'].values,
                    'current': bat_data['Curr'].values,
                }, index=ts)
                fd.battery.index.name = 'time'

        # GPS
        if 'GPS' in messages and messages['GPS']:
            gps_data = self._msg_to_df(messages['GPS'], self.MSG_FORMATS['GPS'])
            if gps_data is not None:
                ts = self._normalize_time(gps_data['TimeUS'].values, unit='us')
                fd.gps = pd.DataFrame({
                    'lat': gps_data['Lat'].values / 1e7,
                    'lon': gps_data['Lng'].values / 1e7,
                    'alt': gps_data['Alt'].values,
                    'speed': gps_data['Spd'].values,
                    'satellites': gps_data['NSats'].values,
                }, index=ts)
                fd.gps.index.name = 'time'

        # Duration
        if fd.gyro is not None and len(fd.gyro) > 0:
            fd.duration_sec = fd.gyro.index[-1] - fd.gyro.index[0]
        elif fd.attitude is not None and len(fd.attitude) > 0:
            fd.duration_sec = fd.attitude.index[-1] - fd.attitude.index[0]

        return fd

    def _msg_to_df(self, rows, format_fields):
        if not rows:
            return None
        n_fields = len(format_fields)
        valid_rows = []
        for row in rows:
            if len(row) >= n_fields:
                numeric_row = []
                for v in row[:n_fields]:
                    try:
                        numeric_row.append(float(v))
                    except (ValueError, TypeError):
                        numeric_row.append(0.0)
                valid_rows.append(numeric_row)
        if not valid_rows:
            return None
        return pd.DataFrame(valid_rows, columns=format_fields[:len(valid_rows[0])])
