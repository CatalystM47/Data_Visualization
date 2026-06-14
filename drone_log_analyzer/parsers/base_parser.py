from abc import ABC, abstractmethod
from dataclasses import dataclass, field
import pandas as pd
import numpy as np
from typing import Optional


@dataclass
class FlightData:
    source_type: str  # 'px4', 'ardupilot', 'betaflight', 'inav'
    fc_type: str  # 'pixhawk', 'matek', 'speedybee', 'unknown'
    firmware_version: str = ''
    duration_sec: float = 0.0

    # Time series (all indexed by timestamp in seconds)
    gyro: Optional[pd.DataFrame] = None       # columns: roll, pitch, yaw (deg/s)
    accel: Optional[pd.DataFrame] = None      # columns: x, y, z (m/s^2)
    attitude: Optional[pd.DataFrame] = None   # columns: roll, pitch, yaw (deg)
    attitude_target: Optional[pd.DataFrame] = None  # desired attitude
    rc_input: Optional[pd.DataFrame] = None   # columns: roll, pitch, yaw, throttle (normalized)
    motor_output: Optional[pd.DataFrame] = None  # columns: motor0..motorN (0-1 or us)
    pid_output: Optional[pd.DataFrame] = None    # columns: roll_p, roll_i, roll_d, pitch_p, ...
    battery: Optional[pd.DataFrame] = None    # columns: voltage, current, remaining
    gps: Optional[pd.DataFrame] = None        # columns: lat, lon, alt, speed, satellites

    # PID parameters if available from log
    pid_params: dict = field(default_factory=dict)
    # Raw parameters dict
    parameters: dict = field(default_factory=dict)
    # Metadata
    metadata: dict = field(default_factory=dict)

    def get_sample_rate(self) -> float:
        if self.gyro is not None and len(self.gyro) > 1:
            dt = np.diff(self.gyro.index).mean()
            if dt > 0:
                return 1.0 / dt
        return 0.0


class BaseParser(ABC):
    @abstractmethod
    def can_parse(self, filepath: str) -> bool:
        pass

    @abstractmethod
    def parse(self, filepath: str) -> FlightData:
        pass

    def _normalize_time(self, timestamps, unit='us') -> np.ndarray:
        ts = np.array(timestamps, dtype=np.float64)
        if unit == 'us':
            ts = ts / 1e6
        elif unit == 'ms':
            ts = ts / 1e3
        ts -= ts[0]
        return ts
