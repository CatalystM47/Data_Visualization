import os
import pandas as pd
import numpy as np
from .base_parser import BaseParser, FlightData

try:
    from pyulog import ULog
    HAS_PYULOG = True
except ImportError:
    HAS_PYULOG = False


class UlogParser(BaseParser):
    def can_parse(self, filepath: str) -> bool:
        return filepath.lower().endswith('.ulg') or filepath.lower().endswith('.ulog')

    def parse(self, filepath: str) -> FlightData:
        if not HAS_PYULOG:
            raise ImportError("pyulog 라이브러리가 필요합니다: pip install pyulog")

        ulog = ULog(filepath)
        fd = FlightData(source_type='px4', fc_type='pixhawk')

        # Extract parameters
        fd.parameters = {p.name: p.value for p in ulog.initial_parameters}
        fd.firmware_version = fd.parameters.get('SYS_AUTOSTART', '')

        # PID params
        for axis in ['ROLL', 'PITCH', 'YAW']:
            prefix = f'MC_{axis}_P' if axis != 'YAW' else f'MC_{axis}RATE_P'
            rate_prefix = f'MC_{axis}RATE'
            fd.pid_params[axis.lower()] = {
                'P': fd.parameters.get(f'{rate_prefix}_P', 0),
                'I': fd.parameters.get(f'{rate_prefix}_I', 0),
                'D': fd.parameters.get(f'{rate_prefix}_D', 0),
            }

        fd.metadata['fc_type'] = 'pixhawk'
        fd.metadata['source'] = 'PX4/QGroundControl'

        # Detect FC hardware
        sys_name = str(fd.parameters.get('SYS_AUTOSTART', ''))
        ver_hw = ''
        for msg in ulog.msg_info_dict:
            if 'ver_hw' in msg.lower():
                ver_hw = str(ulog.msg_info_dict[msg])
                break
        if ver_hw:
            fd.metadata['hardware'] = ver_hw
            hw_lower = ver_hw.lower()
            if 'matek' in hw_lower:
                fd.fc_type = 'matek'
            elif 'speedybee' in hw_lower:
                fd.fc_type = 'speedybee'
            elif 'pixhawk' in hw_lower or 'px4' in hw_lower or 'fmu' in hw_lower:
                fd.fc_type = 'pixhawk'

        # Parse sensor data
        fd.gyro = self._extract_topic(ulog, 'sensor_gyro', {
            'x': 'roll', 'y': 'pitch', 'z': 'yaw'
        })

        fd.accel = self._extract_topic(ulog, 'sensor_accel', {
            'x': 'x', 'y': 'y', 'z': 'z'
        })

        # Attitude
        att = self._extract_topic(ulog, 'vehicle_attitude', {
            'q[0]': 'q0', 'q[1]': 'q1', 'q[2]': 'q2', 'q[3]': 'q3'
        })
        if att is not None and len(att) > 0:
            fd.attitude = self._quat_to_euler(att)

        att_sp = self._extract_topic(ulog, 'vehicle_attitude_setpoint', {
            'q_d[0]': 'q0', 'q_d[1]': 'q1', 'q_d[2]': 'q2', 'q_d[3]': 'q3'
        })
        if att_sp is not None and len(att_sp) > 0:
            fd.attitude_target = self._quat_to_euler(att_sp)

        # RC input
        fd.rc_input = self._extract_topic(ulog, 'manual_control_setpoint', {
            'x': 'roll', 'y': 'pitch', 'r': 'yaw', 'z': 'throttle'
        })

        # Motor output
        motor_data = self._extract_topic(ulog, 'actuator_outputs', {
            'output[0]': 'motor0', 'output[1]': 'motor1',
            'output[2]': 'motor2', 'output[3]': 'motor3',
            'output[4]': 'motor4', 'output[5]': 'motor5',
        })
        if motor_data is not None:
            motor_cols = [c for c in motor_data.columns if motor_data[c].abs().sum() > 0]
            fd.motor_output = motor_data[motor_cols]

        # Rate controller output (PID)
        fd.pid_output = self._extract_topic(ulog, 'rate_ctrl_status', {
            'rollspeed_integ': 'roll_i',
            'pitchspeed_integ': 'pitch_i',
            'yawspeed_integ': 'yaw_i',
        })

        # Battery
        fd.battery = self._extract_topic(ulog, 'battery_status', {
            'voltage_v': 'voltage', 'current_a': 'current', 'remaining': 'remaining'
        })

        # GPS
        fd.gps = self._extract_topic(ulog, 'vehicle_gps_position', {
            'lat': 'lat', 'lon': 'lon', 'alt': 'alt',
            'vel_m_s': 'speed', 'satellites_used': 'satellites'
        })
        if fd.gps is not None and 'lat' in fd.gps.columns:
            fd.gps['lat'] = fd.gps['lat'] / 1e7
            fd.gps['lon'] = fd.gps['lon'] / 1e7
            fd.gps['alt'] = fd.gps['alt'] / 1e3

        # Duration
        if fd.gyro is not None and len(fd.gyro) > 0:
            fd.duration_sec = fd.gyro.index[-1] - fd.gyro.index[0]

        return fd

    def _extract_topic(self, ulog, topic_name, field_map):
        for d in ulog.data_list:
            if d.name == topic_name:
                result = {}
                timestamps = self._normalize_time(d.data['timestamp'], unit='us')
                for src, dst in field_map.items():
                    if src in d.data:
                        result[dst] = d.data[src]
                if result:
                    df = pd.DataFrame(result, index=timestamps)
                    df.index.name = 'time'
                    return df
        return None

    def _quat_to_euler(self, quat_df):
        q0 = quat_df['q0'].values
        q1 = quat_df['q1'].values
        q2 = quat_df['q2'].values
        q3 = quat_df['q3'].values

        roll = np.degrees(np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2)))
        pitch = np.degrees(np.arcsin(np.clip(2*(q0*q2 - q3*q1), -1, 1)))
        yaw = np.degrees(np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2)))

        df = pd.DataFrame({
            'roll': roll, 'pitch': pitch, 'yaw': yaw
        }, index=quat_df.index)
        df.index.name = 'time'
        return df
