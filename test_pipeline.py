"""End-to-end test using synthetic data."""
import numpy as np
import pandas as pd
import sys

from drone_log_analyzer.parsers.base_parser import FlightData
from drone_log_analyzer.analyzers.pid_analyzer import PIDAnalyzer
from drone_log_analyzer.analyzers.stability_analyzer import StabilityAnalyzer
from drone_log_analyzer.analyzers.vibration_analyzer import VibrationAnalyzer
from drone_log_analyzer.analyzers.parameter_advisor import ParameterAdvisor


def create_synthetic_flight():
    """Create a synthetic FlightData with known characteristics for testing."""
    duration = 30.0
    sample_rate = 1000.0
    n = int(duration * sample_rate)
    t = np.linspace(0, duration, n)

    # Gyro: base signal + 50Hz oscillation + noise
    gyro_roll = 5 * np.sin(2 * np.pi * 0.5 * t) + 2 * np.sin(2 * np.pi * 50 * t) + np.random.normal(0, 1, n)
    gyro_pitch = 3 * np.sin(2 * np.pi * 0.3 * t) + np.random.normal(0, 0.5, n)
    gyro_yaw = np.random.normal(0, 0.3, n)

    fd = FlightData(source_type='betaflight', fc_type='speedybee')
    fd.duration_sec = duration

    fd.gyro = pd.DataFrame({
        'roll': gyro_roll, 'pitch': gyro_pitch, 'yaw': gyro_yaw
    }, index=t)
    fd.gyro.index.name = 'time'

    # Accel with some vibration
    fd.accel = pd.DataFrame({
        'x': np.random.normal(0, 2, n),
        'y': np.random.normal(0, 2, n),
        'z': -9.81 + np.random.normal(0, 3, n),
    }, index=t)
    fd.accel.index.name = 'time'

    # Attitude
    fd.attitude = pd.DataFrame({
        'roll': 5 * np.sin(2 * np.pi * 0.5 * t) + np.random.normal(0, 1, n),
        'pitch': 3 * np.sin(2 * np.pi * 0.3 * t) + np.random.normal(0, 0.5, n),
        'yaw': np.cumsum(np.random.normal(0, 0.01, n)),
    }, index=t)
    fd.attitude.index.name = 'time'

    fd.attitude_target = pd.DataFrame({
        'roll': 5 * np.sin(2 * np.pi * 0.5 * t),
        'pitch': 3 * np.sin(2 * np.pi * 0.3 * t),
        'yaw': np.zeros(n),
    }, index=t)
    fd.attitude_target.index.name = 'time'

    # PID output
    fd.pid_output = pd.DataFrame({
        'roll_p': 10 * np.sin(2 * np.pi * 0.5 * t) + np.random.normal(0, 2, n),
        'roll_i': 0.5 * np.sin(2 * np.pi * 0.1 * t),
        'roll_d': np.random.normal(0, 5, n),
        'pitch_p': 8 * np.sin(2 * np.pi * 0.3 * t) + np.random.normal(0, 1, n),
        'pitch_i': 0.3 * np.sin(2 * np.pi * 0.05 * t),
        'pitch_d': np.random.normal(0, 2, n),
    }, index=t)
    fd.pid_output.index.name = 'time'

    # Motor output
    base_throttle = 0.5
    fd.motor_output = pd.DataFrame({
        'motor0': np.clip(base_throttle + np.random.normal(0, 0.05, n), 0, 1),
        'motor1': np.clip(base_throttle + 0.02 + np.random.normal(0, 0.05, n), 0, 1),
        'motor2': np.clip(base_throttle - 0.01 + np.random.normal(0, 0.05, n), 0, 1),
        'motor3': np.clip(base_throttle + 0.01 + np.random.normal(0, 0.05, n), 0, 1),
    }, index=t)
    fd.motor_output.index.name = 'time'

    # Battery
    fd.battery = pd.DataFrame({
        'voltage': np.linspace(16.8, 14.5, n) + np.random.normal(0, 0.1, n),
        'current': 15 + 5 * np.sin(2 * np.pi * 0.1 * t) + np.random.normal(0, 1, n),
    }, index=t)
    fd.battery.index.name = 'time'

    fd.pid_params = {
        'roll': {'P': 45, 'I': 80, 'D': 30},
        'pitch': {'P': 47, 'I': 82, 'D': 32},
        'yaw': {'P': 35, 'I': 90, 'D': 0},
    }

    return fd


def main():
    print("Generating synthetic flight data...")
    fd = create_synthetic_flight()

    print(f"  Source: {fd.source_type}, FC: {fd.fc_type}")
    print(f"  Duration: {fd.duration_sec}s, Sample rate: {fd.get_sample_rate():.0f} Hz")

    print("\nRunning PID analysis...")
    pid_analyzer = PIDAnalyzer()
    pid_results = pid_analyzer.analyze(fd)
    for axis, r in pid_results.items():
        print(f"  {axis}: score={r.score:.0f}, noise={r.noise_level:.2f}°/s, "
              f"osc={r.oscillation_detected}, d_noise_ratio={r.d_term_noise_ratio:.3f}")

    print("\nRunning stability analysis...")
    stability_analyzer = StabilityAnalyzer()
    stability_result = stability_analyzer.analyze(fd)
    print(f"  Vibration score: {stability_result.vibration_score:.0f}")
    print(f"  Attitude score: {stability_result.attitude_score:.0f}")
    print(f"  Motor balance: {stability_result.motor_balance:.0f}%")
    print(f"  Overall: {stability_result.overall_score:.0f}")

    print("\nRunning vibration analysis...")
    vibration_analyzer = VibrationAnalyzer()
    vibration_results = vibration_analyzer.analyze(fd)
    for axis, d in vibration_results.items():
        print(f"  {axis}: rms={d.rms:.2f}, dom_freq={d.dominant_freq:.1f}Hz, severity={d.severity}")

    print("\nGenerating recommendations...")
    advisor = ParameterAdvisor()
    recs = advisor.generate_recommendations(fd, pid_results, stability_result, vibration_results)
    for rec in recs:
        print(f"  [{rec.severity}] [{rec.category}] {rec.title}")
        if rec.steps:
            for step in rec.steps[:2]:
                print(f"    → {step}")

    print(f"\nTotal recommendations: {len(recs)}")
    print("\n✓ Pipeline test PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
