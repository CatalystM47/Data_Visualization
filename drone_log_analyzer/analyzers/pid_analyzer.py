import numpy as np
import pandas as pd
from scipy import signal
from scipy.fft import fft, fftfreq
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class PIDAnalysisResult:
    axis: str
    noise_level: float = 0.0          # RMS of gyro noise (deg/s)
    noise_floor_db: float = 0.0
    dominant_freq: float = 0.0         # Hz
    dominant_freq_amplitude: float = 0.0
    oscillation_detected: bool = False
    oscillation_freqs: list = field(default_factory=list)
    d_term_noise_ratio: float = 0.0
    p_term_rms: float = 0.0
    i_term_rms: float = 0.0
    d_term_rms: float = 0.0
    step_response_overshoot: float = 0.0
    step_response_rise_time: float = 0.0
    step_response_settling_time: float = 0.0
    tracking_error_rms: float = 0.0
    tracking_error_max: float = 0.0
    fft_freqs: Optional[np.ndarray] = None
    fft_magnitude: Optional[np.ndarray] = None
    psd_freqs: Optional[np.ndarray] = None
    psd_power: Optional[np.ndarray] = None
    score: float = 0.0  # 0-100


class PIDAnalyzer:
    def __init__(self):
        self.results = {}

    def analyze(self, flight_data) -> dict:
        axes = ['roll', 'pitch', 'yaw']
        self.results = {}

        for axis in axes:
            result = PIDAnalysisResult(axis=axis)

            if flight_data.gyro is not None and axis in flight_data.gyro.columns:
                gyro_data = flight_data.gyro[axis].dropna().values
                sample_rate = flight_data.get_sample_rate()
                if sample_rate <= 0:
                    sample_rate = 1000.0

                result.noise_level = self._compute_noise_rms(gyro_data)
                fft_result = self._compute_fft(gyro_data, sample_rate)
                result.fft_freqs = fft_result['freqs']
                result.fft_magnitude = fft_result['magnitude']
                result.dominant_freq = fft_result['dominant_freq']
                result.dominant_freq_amplitude = fft_result['dominant_amplitude']

                psd_result = self._compute_psd(gyro_data, sample_rate)
                result.psd_freqs = psd_result['freqs']
                result.psd_power = psd_result['power']
                result.noise_floor_db = psd_result['noise_floor_db']

                osc = self._detect_oscillations(gyro_data, sample_rate)
                result.oscillation_detected = osc['detected']
                result.oscillation_freqs = osc['frequencies']

            if flight_data.pid_output is not None:
                result.p_term_rms = self._get_term_rms(flight_data.pid_output, f'{axis}_p')
                result.i_term_rms = self._get_term_rms(flight_data.pid_output, f'{axis}_i')
                result.d_term_rms = self._get_term_rms(flight_data.pid_output, f'{axis}_d')

                if result.p_term_rms > 0:
                    result.d_term_noise_ratio = result.d_term_rms / result.p_term_rms

            if (flight_data.attitude is not None and flight_data.attitude_target is not None
                    and axis in flight_data.attitude.columns
                    and axis in flight_data.attitude_target.columns):
                tracking = self._compute_tracking_error(
                    flight_data.attitude[axis],
                    flight_data.attitude_target[axis]
                )
                result.tracking_error_rms = tracking['rms']
                result.tracking_error_max = tracking['max']

                step = self._analyze_step_response(
                    flight_data.attitude_target[axis],
                    flight_data.attitude[axis]
                )
                result.step_response_overshoot = step['overshoot']
                result.step_response_rise_time = step['rise_time']
                result.step_response_settling_time = step['settling_time']

            result.score = self._compute_score(result)
            self.results[axis] = result

        return self.results

    def _compute_noise_rms(self, data: np.ndarray) -> float:
        detrended = signal.detrend(data)
        return float(np.sqrt(np.mean(detrended**2)))

    def _compute_fft(self, data: np.ndarray, sample_rate: float) -> dict:
        n = len(data)
        if n < 64:
            return {'freqs': np.array([]), 'magnitude': np.array([]),
                    'dominant_freq': 0, 'dominant_amplitude': 0}

        windowed = data * signal.windows.hann(n)
        yf = fft(windowed)
        xf = fftfreq(n, 1.0 / sample_rate)

        positive = xf > 0
        freqs = xf[positive]
        magnitude = 2.0 / n * np.abs(yf[positive])

        if len(magnitude) == 0:
            return {'freqs': freqs, 'magnitude': magnitude,
                    'dominant_freq': 0, 'dominant_amplitude': 0}

        peak_idx = np.argmax(magnitude)
        return {
            'freqs': freqs,
            'magnitude': magnitude,
            'dominant_freq': float(freqs[peak_idx]),
            'dominant_amplitude': float(magnitude[peak_idx]),
        }

    def _compute_psd(self, data: np.ndarray, sample_rate: float) -> dict:
        if len(data) < 256:
            return {'freqs': np.array([]), 'power': np.array([]), 'noise_floor_db': 0}

        nperseg = min(1024, len(data) // 4)
        freqs, power = signal.welch(data, fs=sample_rate, nperseg=nperseg)

        power_db = 10 * np.log10(power + 1e-12)
        noise_floor = float(np.median(power_db))

        return {
            'freqs': freqs,
            'power': power_db,
            'noise_floor_db': noise_floor,
        }

    def _detect_oscillations(self, data: np.ndarray, sample_rate: float) -> dict:
        if len(data) < 256:
            return {'detected': False, 'frequencies': []}

        nperseg = min(1024, len(data) // 4)
        freqs, power = signal.welch(data, fs=sample_rate, nperseg=nperseg)

        power_db = 10 * np.log10(power + 1e-12)
        median_power = np.median(power_db)
        threshold = median_power + 15  # 15dB above noise floor

        peaks, properties = signal.find_peaks(power_db, height=threshold, distance=5)

        osc_freqs = []
        for p in peaks:
            if freqs[p] > 5:  # ignore very low frequencies
                osc_freqs.append({
                    'frequency': float(freqs[p]),
                    'amplitude_db': float(power_db[p]),
                    'above_floor_db': float(power_db[p] - median_power),
                })

        return {
            'detected': len(osc_freqs) > 0,
            'frequencies': osc_freqs,
        }

    def _get_term_rms(self, pid_df: pd.DataFrame, col: str) -> float:
        if col in pid_df.columns:
            data = pid_df[col].dropna().values
            if len(data) > 0:
                return float(np.sqrt(np.mean(data**2)))
        return 0.0

    def _compute_tracking_error(self, actual: pd.Series, target: pd.Series) -> dict:
        common_idx = actual.index.intersection(target.index)
        if len(common_idx) < 10:
            target_resampled = np.interp(actual.index, target.index, target.values)
            error = actual.values - target_resampled
        else:
            error = actual.loc[common_idx].values - target.loc[common_idx].values

        return {
            'rms': float(np.sqrt(np.mean(error**2))),
            'max': float(np.max(np.abs(error))),
        }

    def _analyze_step_response(self, target: pd.Series, actual: pd.Series) -> dict:
        result = {'overshoot': 0.0, 'rise_time': 0.0, 'settling_time': 0.0}

        target_vals = target.values
        target_diff = np.diff(target_vals)
        target_times = target.index.values

        step_threshold = np.std(target_vals) * 0.5
        if step_threshold < 1.0:
            return result

        step_indices = np.where(np.abs(target_diff) > step_threshold)[0]
        if len(step_indices) == 0:
            return result

        overshoots = []
        rise_times = []
        settling_times = []

        for idx in step_indices[:10]:
            if idx + 50 >= len(target_vals):
                continue

            step_start = target_times[idx]
            step_target = target_vals[idx + 1]
            step_initial = target_vals[idx]
            step_size = step_target - step_initial

            if abs(step_size) < 1.0:
                continue

            actual_resampled = np.interp(
                target_times[idx:idx+50],
                actual.index.values,
                actual.values
            )

            normalized = (actual_resampled - step_initial) / step_size
            overshoot = max(0, float(np.max(normalized) - 1.0) * 100) if step_size > 0 else \
                        max(0, float(1.0 - np.min(normalized)) * 100)
            overshoots.append(overshoot)

            rise_indices = np.where(normalized >= 0.9)[0]
            if len(rise_indices) > 0:
                dt = target_times[idx + rise_indices[0]] - target_times[idx]
                rise_times.append(float(dt))

            settle_band = 0.05
            settled = np.abs(normalized - 1.0) < settle_band
            if np.any(settled):
                last_outside = np.where(~settled)[0]
                if len(last_outside) > 0:
                    settle_idx = last_outside[-1] + 1
                    if settle_idx < len(target_times[idx:idx+50]):
                        settling_times.append(float(
                            target_times[idx + settle_idx] - target_times[idx]
                        ))

        if overshoots:
            result['overshoot'] = float(np.mean(overshoots))
        if rise_times:
            result['rise_time'] = float(np.mean(rise_times))
        if settling_times:
            result['settling_time'] = float(np.mean(settling_times))

        return result

    def _compute_score(self, result: PIDAnalysisResult) -> float:
        score = 100.0

        if result.noise_level > 50:
            score -= 30
        elif result.noise_level > 20:
            score -= 15
        elif result.noise_level > 10:
            score -= 5

        if result.oscillation_detected:
            score -= 10 * len(result.oscillation_freqs)

        if result.d_term_noise_ratio > 0.8:
            score -= 20
        elif result.d_term_noise_ratio > 0.5:
            score -= 10

        if result.tracking_error_rms > 10:
            score -= 20
        elif result.tracking_error_rms > 5:
            score -= 10
        elif result.tracking_error_rms > 2:
            score -= 5

        if result.step_response_overshoot > 30:
            score -= 15
        elif result.step_response_overshoot > 15:
            score -= 8

        return max(0, min(100, score))
