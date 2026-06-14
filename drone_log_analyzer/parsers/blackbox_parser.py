import os
import csv
import io
import struct
import pandas as pd
import numpy as np
from .base_parser import BaseParser, FlightData


class BlackboxParser(BaseParser):
    """Parser for Betaflight/iNav Blackbox logs (.bbl, .bfl, .csv, .txt)"""

    BETAFLIGHT_FIELDS = {
        'gyroADC[0]': ('gyro', 'roll'),
        'gyroADC[1]': ('gyro', 'pitch'),
        'gyroADC[2]': ('gyro', 'yaw'),
        'gyroUnfilt[0]': ('gyro', 'roll'),
        'gyroUnfilt[1]': ('gyro', 'pitch'),
        'gyroUnfilt[2]': ('gyro', 'yaw'),
        'gyroData[0]': ('gyro', 'roll'),
        'gyroData[1]': ('gyro', 'pitch'),
        'gyroData[2]': ('gyro', 'yaw'),
        'accSmooth[0]': ('accel', 'x'),
        'accSmooth[1]': ('accel', 'y'),
        'accSmooth[2]': ('accel', 'z'),
        'acc[0]': ('accel', 'x'),
        'acc[1]': ('accel', 'y'),
        'acc[2]': ('accel', 'z'),
        'axisP[0]': ('pid_output', 'roll_p'),
        'axisP[1]': ('pid_output', 'pitch_p'),
        'axisP[2]': ('pid_output', 'yaw_p'),
        'axisI[0]': ('pid_output', 'roll_i'),
        'axisI[1]': ('pid_output', 'pitch_i'),
        'axisI[2]': ('pid_output', 'yaw_i'),
        'axisD[0]': ('pid_output', 'roll_d'),
        'axisD[1]': ('pid_output', 'pitch_d'),
        'axisD[2]': ('pid_output', 'yaw_d'),
        'axisF[0]': ('pid_output', 'roll_f'),
        'axisF[1]': ('pid_output', 'pitch_f'),
        'axisF[2]': ('pid_output', 'yaw_f'),
        'rcCommand[0]': ('rc_input', 'roll'),
        'rcCommand[1]': ('rc_input', 'pitch'),
        'rcCommand[2]': ('rc_input', 'yaw'),
        'rcCommand[3]': ('rc_input', 'throttle'),
        'rcData[0]': ('rc_input', 'roll'),
        'rcData[1]': ('rc_input', 'pitch'),
        'rcData[2]': ('rc_input', 'yaw'),
        'rcData[3]': ('rc_input', 'throttle'),
        'motor[0]': ('motor_output', 'motor0'),
        'motor[1]': ('motor_output', 'motor1'),
        'motor[2]': ('motor_output', 'motor2'),
        'motor[3]': ('motor_output', 'motor3'),
        'eRPM[0]': ('motor_rpm', 'motor0'),
        'eRPM[1]': ('motor_rpm', 'motor1'),
        'eRPM[2]': ('motor_rpm', 'motor2'),
        'eRPM[3]': ('motor_rpm', 'motor3'),
        'vbatLatest': ('battery', 'voltage'),
        'vbat': ('battery', 'voltage'),
        'amperageLatest': ('battery', 'current'),
        'amperage': ('battery', 'current'),
        'setpoint[0]': ('attitude_target', 'roll'),
        'setpoint[1]': ('attitude_target', 'pitch'),
        'setpoint[2]': ('attitude_target', 'yaw'),
        'setpoint[3]': ('attitude_target', 'throttle'),
        'heading[0]': ('attitude', 'roll'),
        'heading[1]': ('attitude', 'pitch'),
        'heading[2]': ('attitude', 'yaw'),
    }

    CSV_DETECT_KEYWORDS = [
        'gyroADC', 'gyroUnfilt', 'gyroData', 'loopIteration',
        'axisP', 'axisI', 'axisD', 'axisF', 'rcCommand', 'rcData',
        'motor[', 'eRPM[', 'accSmooth', 'vbatLatest', 'vbat',
        'setpoint[', 'amperageLatest',
    ]

    def can_parse(self, filepath: str) -> bool:
        ext = filepath.lower()
        if ext.endswith(('.bbl', '.bfl')):
            return True
        if ext.endswith(('.csv', '.txt')):
            return self._is_blackbox_csv(filepath)
        return False

    def _is_blackbox_csv(self, filepath: str) -> bool:
        try:
            with open(filepath, 'r', errors='ignore') as f:
                for i in range(200):
                    line = f.readline()
                    if not line:
                        break
                    stripped = line.replace('"', '')
                    if any(k in stripped for k in self.CSV_DETECT_KEYWORDS):
                        return True
            return False
        except Exception:
            return False

    def parse(self, filepath: str) -> FlightData:
        ext = filepath.lower()
        if ext.endswith(('.bbl', '.bfl')):
            return self._parse_binary(filepath)
        else:
            return self._parse_csv(filepath)

    def _parse_binary(self, filepath: str) -> FlightData:
        csv_data = self._decode_binary_to_csv(filepath)
        if csv_data is not None:
            return self._parse_csv_data(csv_data)
        raise ValueError("바이너리 블랙박스 파일을 파싱할 수 없습니다. "
                         "Betaflight Blackbox Explorer에서 CSV로 내보낸 후 다시 시도해주세요.")

    def _decode_binary_to_csv(self, filepath: str):
        try:
            with open(filepath, 'rb') as f:
                header = f.read(1024)

            header_str = header.decode('ascii', errors='ignore')
            if 'H Product:Blackbox' not in header_str and 'H Firmware' not in header_str:
                return None

            with open(filepath, 'rb') as f:
                content = f.read()

            text = content.decode('ascii', errors='ignore')
            metadata = {}
            csv_lines = []
            header_line = None

            for line in text.split('\n'):
                line = line.strip()
                if line.startswith('H '):
                    parts = line[2:].split(':', 1)
                    if len(parts) == 2:
                        metadata[parts[0].strip()] = parts[1].strip()
                elif ',' in line and not line.startswith('H ') and not line.startswith('E '):
                    if header_line is None and ('loopIteration' in line or 'time' in line):
                        header_line = line
                    elif header_line is not None:
                        csv_lines.append(line)

            if header_line and csv_lines:
                return {
                    'header': header_line,
                    'lines': csv_lines,
                    'metadata': metadata,
                }
            return None
        except Exception:
            return None

    def _parse_csv(self, filepath: str) -> FlightData:
        metadata = {}
        header_line = None
        header_line_num = None

        with open(filepath, 'r', errors='ignore') as f:
            for line_num, line in enumerate(f):
                line = line.strip()
                if not line:
                    continue

                unquoted = line.replace('"', '')

                if line.startswith('H '):
                    parts = line[2:].split(':', 1)
                    if len(parts) == 2:
                        metadata[parts[0].strip()] = parts[1].strip()
                elif header_line is None:
                    if 'loopIteration' in unquoted:
                        header_line = line
                        header_line_num = line_num
                        break
                    # "key",value metadata format (Betaflight Blackbox Explorer)
                    parts = line.split(',', 1)
                    if len(parts) == 2:
                        key = parts[0].strip().strip('"')
                        val = parts[1].strip().strip('"')
                        metadata[key] = val

        if header_line is None:
            # Fallback: scan for any known column keyword
            with open(filepath, 'r', errors='ignore') as f:
                for line_num, line in enumerate(f):
                    unquoted = line.replace('"', '')
                    if any(k in unquoted for k in self.CSV_DETECT_KEYWORDS):
                        if ',' in line and sum(1 for c in line if c == ',') > 5:
                            header_line = line.strip()
                            header_line_num = line_num
                            break

        if header_line is None:
            raise ValueError("블랙박스 CSV 파일에서 데이터 헤더를 찾을 수 없습니다.")

        # Strip quotes from header columns
        columns = [c.strip().strip('"') for c in header_line.split(',')]

        # Read data rows using pandas for speed
        try:
            df = pd.read_csv(filepath, skiprows=header_line_num, header=0,
                             low_memory=False, on_bad_lines='skip')
            # Clean column names (strip quotes and whitespace)
            df.columns = [c.strip().strip('"') for c in df.columns]
        except Exception:
            # Manual fallback
            data_lines = []
            with open(filepath, 'r', errors='ignore') as f:
                for i, line in enumerate(f):
                    if i <= header_line_num:
                        continue
                    data_lines.append(line.strip())

            return self._parse_csv_data({
                'header': ','.join(columns),
                'lines': data_lines,
                'metadata': metadata,
            })

        return self._parse_dataframe(df, metadata)

    def _parse_csv_data(self, csv_data: dict) -> FlightData:
        header = csv_data['header']
        lines = csv_data['lines']
        metadata = csv_data.get('metadata', {})

        columns = [c.strip().strip('"') for c in header.split(',')]
        rows = []
        for line in lines:
            parts = line.split(',')
            if len(parts) == len(columns):
                row = []
                for p in parts:
                    p = p.strip().strip('"')
                    try:
                        row.append(float(p))
                    except ValueError:
                        row.append(np.nan)
                rows.append(row)

        if not rows:
            raise ValueError("로그 데이터가 비어있습니다.")

        df = pd.DataFrame(rows, columns=columns)
        return self._parse_dataframe(df, metadata)

    def _parse_dataframe(self, df: pd.DataFrame, metadata: dict) -> FlightData:
        # Clean column names
        df.columns = [c.strip().strip('"') for c in df.columns]

        source = 'betaflight'
        fw_type = metadata.get('Firmware type', metadata.get('firmwareType', ''))
        product = metadata.get('Product', '')
        if str(fw_type).lower() == 'inav' or 'inav' in product.lower():
            source = 'inav'

        fd = FlightData(source_type=source, fc_type='unknown')
        fd.metadata = metadata

        # Detect FC type
        fc_name = metadata.get('Board information', '') or metadata.get('Craft name', '')
        fc_lower = fc_name.lower()
        if 'matek' in fc_lower:
            fd.fc_type = 'matek'
        elif 'speedybee' in fc_lower or 'speedy' in fc_lower:
            fd.fc_type = 'speedybee'
        elif 'jhef' in fc_lower or 'jhemcu' in fc_lower:
            fd.fc_type = 'jhemcu'
        elif 'pixhawk' in fc_lower:
            fd.fc_type = 'pixhawk'

        # Firmware version
        fd.firmware_version = metadata.get('Firmware revision',
                                           metadata.get('firmwareVersion', ''))

        # Extract PID params from Betaflight metadata
        for pid_key, axis in [('rollPID', 'roll'), ('pitchPID', 'pitch'), ('yawPID', 'yaw')]:
            if pid_key in metadata:
                vals = metadata[pid_key].strip('"').split(',')
                if len(vals) >= 3:
                    fd.pid_params[axis] = {
                        'P': int(vals[0]) if vals[0].strip() else 0,
                        'I': int(vals[1]) if vals[1].strip() else 0,
                        'D': int(vals[2]) if vals[2].strip() else 0,
                    }
                    if len(vals) >= 4 and vals[3].strip():
                        fd.pid_params[axis]['F'] = int(vals[3])

        # Also store rate/filter metadata
        for key, val in metadata.items():
            k_lower = key.lower()
            if any(x in k_lower for x in ['rate', 'filter', 'lpf', 'notch', 'expo',
                                           'dterm', 'gyro_', 'throttle', 'tpa']):
                fd.pid_params.setdefault('_settings', {})[key] = val

        # Time column
        time_col = None
        for col in df.columns:
            if col.lower() in ('time', 'time (us)', 'time(us)', 'timeus', 'time_us'):
                time_col = col
                break
        if time_col is None:
            for col in df.columns:
                if 'time' in col.lower():
                    time_col = col
                    break

        if time_col:
            time_vals = pd.to_numeric(df[time_col], errors='coerce')
            timestamps = self._normalize_time(time_vals.fillna(0).values, unit='us')
        else:
            looptime = float(metadata.get('looptime', 125))
            pid_denom = float(metadata.get('pid_process_denom', 2))
            dt = looptime * pid_denom / 1e6
            timestamps = np.arange(len(df)) * dt

        # Parse known fields
        groups = {}
        for col in df.columns:
            if col in self.BETAFLIGHT_FIELDS:
                group, field = self.BETAFLIGHT_FIELDS[col]
                if group not in groups:
                    groups[group] = {}
                if field not in groups[group]:
                    groups[group][field] = pd.to_numeric(df[col], errors='coerce').values

        for group_name in ['gyro', 'accel', 'pid_output', 'rc_input',
                           'motor_output', 'motor_rpm', 'battery',
                           'attitude_target', 'attitude']:
            if group_name in groups:
                group_df = pd.DataFrame(groups[group_name], index=timestamps)
                group_df.index.name = 'time'
                group_df = group_df.dropna(how='all')
                if group_name == 'motor_rpm':
                    if fd.motor_output is None:
                        fd.motor_output = group_df
                elif group_name == 'attitude':
                    # Convert radians to degrees if values look like radians
                    if group_df.abs().mean().mean() < 10:
                        group_df = np.degrees(group_df)
                    fd.attitude = group_df
                else:
                    setattr(fd, group_name, group_df)

        # Scale battery voltage (Betaflight reports in 0.01V)
        if fd.battery is not None and 'voltage' in fd.battery.columns:
            mean_v = fd.battery['voltage'].mean()
            if mean_v > 100:
                fd.battery['voltage'] = fd.battery['voltage'] / 100.0
            elif mean_v > 50:
                fd.battery['voltage'] = fd.battery['voltage'] / 10.0
            if 'current' in fd.battery.columns:
                mean_c = fd.battery['current'].mean()
                if mean_c > 1000:
                    fd.battery['current'] = fd.battery['current'] / 100.0
                elif mean_c > 500:
                    fd.battery['current'] = fd.battery['current'] / 10.0

        # Duration
        fd.duration_sec = timestamps[-1] - timestamps[0] if len(timestamps) > 0 else 0

        return fd
