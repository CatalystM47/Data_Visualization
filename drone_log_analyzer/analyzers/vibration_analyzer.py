import numpy as np
import pandas as pd
from scipy import signal
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class VibrationDetail:
    axis: str
    rms: float = 0.0
    peak: float = 0.0
    dominant_freq: float = 0.0
    harmonic_freqs: list = field(default_factory=list)
    fft_freqs: Optional[np.ndarray] = None
    fft_magnitude: Optional[np.ndarray] = None
    severity: str = 'good'  # good, warning, critical


class VibrationAnalyzer:
    def analyze(self, flight_data) -> dict:
        results = {}

        if flight_data.accel is None:
            return results

        sample_rate = flight_data.get_sample_rate()
        if sample_rate <= 0:
            sample_rate = 1000.0

        for axis in ['x', 'y', 'z']:
            if axis not in flight_data.accel.columns:
                continue

            data = flight_data.accel[axis].dropna().values
            if len(data) < 64:
                continue

            if axis == 'z':
                data = data - np.mean(data)
            else:
                data = signal.detrend(data)

            detail = VibrationDetail(axis=axis)
            detail.rms = float(np.sqrt(np.mean(data**2)))
            detail.peak = float(np.max(np.abs(data)))

            # FFT analysis
            n = len(data)
            windowed = data * signal.windows.hann(n)
            yf = np.fft.fft(windowed)
            xf = np.fft.fftfreq(n, 1.0 / sample_rate)

            positive = xf > 0
            freqs = xf[positive]
            magnitude = 2.0 / n * np.abs(yf[positive])

            # Limit to reasonable range
            freq_mask = freqs <= min(500, sample_rate / 2)
            detail.fft_freqs = freqs[freq_mask]
            detail.fft_magnitude = magnitude[freq_mask]

            if len(detail.fft_magnitude) > 0:
                peak_idx = np.argmax(detail.fft_magnitude)
                detail.dominant_freq = float(detail.fft_freqs[peak_idx])

                # Find harmonics
                peaks, _ = signal.find_peaks(
                    detail.fft_magnitude,
                    height=np.max(detail.fft_magnitude) * 0.2,
                    distance=max(1, int(10 / (freqs[1] - freqs[0])))
                )
                detail.harmonic_freqs = [float(detail.fft_freqs[p]) for p in peaks[:5]]

            if detail.rms < 3.0:
                detail.severity = 'good'
            elif detail.rms < 15.0:
                detail.severity = 'warning'
            else:
                detail.severity = 'critical'

            results[axis] = detail

        return results
