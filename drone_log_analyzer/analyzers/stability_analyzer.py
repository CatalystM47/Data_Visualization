import numpy as np
import pandas as pd
from scipy import signal
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class StabilityResult:
    vibration_rms: dict = field(default_factory=dict)   # {x, y, z} m/s^2
    vibration_peak: dict = field(default_factory=dict)
    vibration_clipping: dict = field(default_factory=dict)
    vibration_score: float = 0.0  # 0-100

    attitude_error_rms: dict = field(default_factory=dict)  # {roll, pitch, yaw} deg
    attitude_error_max: dict = field(default_factory=dict)
    attitude_score: float = 0.0

    motor_balance: float = 0.0       # 0-100, 100 = perfectly balanced
    motor_saturation_pct: dict = field(default_factory=dict)
    motor_min_values: dict = field(default_factory=dict)
    motor_max_values: dict = field(default_factory=dict)

    battery_voltage_drop: float = 0.0
    battery_min_voltage: float = 0.0
    battery_max_current: float = 0.0

    overall_score: float = 0.0


class StabilityAnalyzer:
    VIBE_THRESHOLDS = {
        'good': 3.0,      # m/s^2
        'warning': 15.0,
        'critical': 30.0,
        'clipping': 60.0,
    }

    def analyze(self, flight_data) -> StabilityResult:
        result = StabilityResult()

        if flight_data.accel is not None:
            self._analyze_vibration(flight_data, result)

        if flight_data.attitude is not None and flight_data.attitude_target is not None:
            self._analyze_attitude_tracking(flight_data, result)

        if flight_data.motor_output is not None:
            self._analyze_motors(flight_data, result)

        if flight_data.battery is not None:
            self._analyze_battery(flight_data, result)

        scores = []
        if result.vibration_score > 0:
            scores.append(result.vibration_score)
        if result.attitude_score > 0:
            scores.append(result.attitude_score)
        if result.motor_balance > 0:
            scores.append(result.motor_balance)
        result.overall_score = float(np.mean(scores)) if scores else 0

        return result

    def _analyze_vibration(self, flight_data, result: StabilityResult):
        accel = flight_data.accel
        for axis in ['x', 'y', 'z']:
            if axis not in accel.columns:
                continue
            data = accel[axis].dropna().values
            if len(data) == 0:
                continue

            if axis == 'z':
                data = data - np.mean(data)  # remove gravity component
            else:
                data = signal.detrend(data)

            result.vibration_rms[axis] = float(np.sqrt(np.mean(data**2)))
            result.vibration_peak[axis] = float(np.max(np.abs(data)))
            result.vibration_clipping[axis] = float(
                np.sum(np.abs(data) > self.VIBE_THRESHOLDS['clipping']) / len(data) * 100
            )

        if result.vibration_rms:
            avg_rms = np.mean(list(result.vibration_rms.values()))
            if avg_rms < self.VIBE_THRESHOLDS['good']:
                result.vibration_score = 95
            elif avg_rms < self.VIBE_THRESHOLDS['warning']:
                result.vibration_score = 70 - (avg_rms - self.VIBE_THRESHOLDS['good']) * 2
            elif avg_rms < self.VIBE_THRESHOLDS['critical']:
                result.vibration_score = 40 - (avg_rms - self.VIBE_THRESHOLDS['warning'])
            else:
                result.vibration_score = max(0, 10 - (avg_rms - self.VIBE_THRESHOLDS['critical']) * 0.5)

    def _analyze_attitude_tracking(self, flight_data, result: StabilityResult):
        attitude = flight_data.attitude
        target = flight_data.attitude_target

        scores = []
        for axis in ['roll', 'pitch', 'yaw']:
            if axis not in attitude.columns or axis not in target.columns:
                continue

            actual = attitude[axis].dropna()
            desired = target[axis].dropna()

            if len(actual) == 0 or len(desired) == 0:
                continue

            desired_resampled = np.interp(actual.index, desired.index, desired.values)
            error = actual.values - desired_resampled

            if axis == 'yaw':
                error = np.where(error > 180, error - 360, error)
                error = np.where(error < -180, error + 360, error)

            result.attitude_error_rms[axis] = float(np.sqrt(np.mean(error**2)))
            result.attitude_error_max[axis] = float(np.max(np.abs(error)))

            rms = result.attitude_error_rms[axis]
            if rms < 1.0:
                scores.append(95)
            elif rms < 3.0:
                scores.append(80)
            elif rms < 5.0:
                scores.append(60)
            elif rms < 10.0:
                scores.append(40)
            else:
                scores.append(20)

        result.attitude_score = float(np.mean(scores)) if scores else 0

    def _analyze_motors(self, flight_data, result: StabilityResult):
        motors = flight_data.motor_output
        motor_cols = [c for c in motors.columns if c.startswith('motor')]

        if not motor_cols:
            return

        motor_means = []
        for col in motor_cols:
            data = motors[col].dropna().values
            if len(data) == 0:
                continue

            motor_means.append(np.mean(data))
            result.motor_min_values[col] = float(np.min(data))
            result.motor_max_values[col] = float(np.max(data))

            max_val = np.max(data)
            if max_val > 1:
                sat_threshold = max_val * 0.95
            else:
                sat_threshold = 0.95
            result.motor_saturation_pct[col] = float(
                np.sum(data >= sat_threshold) / len(data) * 100
            )

        if len(motor_means) > 1:
            mean_of_means = np.mean(motor_means)
            if mean_of_means > 0:
                spread = np.std(motor_means) / mean_of_means * 100
                result.motor_balance = max(0, 100 - spread * 5)
            else:
                result.motor_balance = 0

    def _analyze_battery(self, flight_data, result: StabilityResult):
        bat = flight_data.battery
        if 'voltage' in bat.columns:
            v = bat['voltage'].dropna().values
            if len(v) > 0:
                result.battery_min_voltage = float(np.min(v))
                result.battery_voltage_drop = float(np.max(v) - np.min(v))
        if 'current' in bat.columns:
            c = bat['current'].dropna().values
            if len(c) > 0:
                result.battery_max_current = float(np.max(c))
