import os
import json
import uuid
import traceback
from flask import Flask, request, jsonify, render_template, send_from_directory
import numpy as np

from drone_log_analyzer.parsers import detect_and_parse
from drone_log_analyzer.analyzers.pid_analyzer import PIDAnalyzer
from drone_log_analyzer.analyzers.stability_analyzer import StabilityAnalyzer
from drone_log_analyzer.analyzers.vibration_analyzer import VibrationAnalyzer
from drone_log_analyzer.analyzers.parameter_advisor import ParameterAdvisor

app = Flask(__name__,
            template_folder='drone_log_analyzer/templates',
            static_folder='drone_log_analyzer/static')
app.config['MAX_CONTENT_LENGTH'] = 500 * 1024 * 1024  # 500MB
app.config['UPLOAD_FOLDER'] = os.path.join(os.path.dirname(__file__), 'uploads')

os.makedirs(app.config['UPLOAD_FOLDER'], exist_ok=True)

flight_data_store = {}


class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, (np.floating, np.float64, np.float32)):
            return float(obj)
        if isinstance(obj, (np.integer, np.int64, np.int32)):
            return int(obj)
        if isinstance(obj, np.bool_):
            return bool(obj)
        return super().default(obj)


app.json_encoder = NumpyEncoder


def numpy_safe(obj):
    if isinstance(obj, dict):
        return {k: numpy_safe(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [numpy_safe(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.floating, np.float64, np.float32)):
        return float(obj)
    if isinstance(obj, (np.integer, np.int64, np.int32)):
        return int(obj)
    if isinstance(obj, np.bool_):
        return bool(obj)
    return obj


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/upload', methods=['POST'])
def upload_file():
    if 'file' not in request.files:
        return jsonify({'error': '파일이 선택되지 않았습니다'}), 400

    file = request.files['file']
    if file.filename == '':
        return jsonify({'error': '파일이 선택되지 않았습니다'}), 400

    session_id = str(uuid.uuid4())
    filename = f"{session_id}_{file.filename}"
    filepath = os.path.join(app.config['UPLOAD_FOLDER'], filename)
    file.save(filepath)

    try:
        fd = detect_and_parse(filepath)
        flight_data_store[session_id] = fd

        summary = {
            'session_id': session_id,
            'source_type': fd.source_type,
            'fc_type': fd.fc_type,
            'firmware_version': str(fd.firmware_version),
            'duration_sec': round(fd.duration_sec, 1),
            'sample_rate': round(fd.get_sample_rate(), 0),
            'has_gyro': fd.gyro is not None,
            'has_accel': fd.accel is not None,
            'has_attitude': fd.attitude is not None,
            'has_pid': fd.pid_output is not None,
            'has_motor': fd.motor_output is not None,
            'has_battery': fd.battery is not None,
            'has_gps': fd.gps is not None,
            'has_rc': fd.rc_input is not None,
            'pid_params': numpy_safe(fd.pid_params),
            'metadata': {k: str(v) for k, v in fd.metadata.items()},
        }
        return jsonify(summary)

    except Exception as e:
        traceback.print_exc()
        return jsonify({'error': str(e)}), 400
    finally:
        try:
            os.remove(filepath)
        except OSError:
            pass


@app.route('/api/analyze', methods=['POST'])
def analyze():
    data = request.get_json()
    session_id = data.get('session_id')

    if not session_id or session_id not in flight_data_store:
        return jsonify({'error': '세션을 찾을 수 없습니다. 파일을 다시 업로드해주세요.'}), 404

    fd = flight_data_store[session_id]

    pid_analyzer = PIDAnalyzer()
    stability_analyzer = StabilityAnalyzer()
    vibration_analyzer = VibrationAnalyzer()
    advisor = ParameterAdvisor()

    pid_results = pid_analyzer.analyze(fd)
    stability_result = stability_analyzer.analyze(fd)
    vibration_results = vibration_analyzer.analyze(fd)
    recommendations = advisor.generate_recommendations(fd, pid_results, stability_result, vibration_results)

    # Build response
    response = {
        'pid': {},
        'stability': {},
        'vibration': {},
        'recommendations': [],
        'charts': {},
    }

    # PID results
    for axis, result in pid_results.items():
        response['pid'][axis] = {
            'score': result.score,
            'noise_level': round(result.noise_level, 2),
            'noise_floor_db': round(result.noise_floor_db, 1),
            'dominant_freq': round(result.dominant_freq, 1),
            'oscillation_detected': result.oscillation_detected,
            'oscillation_freqs': result.oscillation_freqs,
            'd_term_noise_ratio': round(result.d_term_noise_ratio, 3),
            'p_term_rms': round(result.p_term_rms, 2),
            'i_term_rms': round(result.i_term_rms, 2),
            'd_term_rms': round(result.d_term_rms, 2),
            'step_response_overshoot': round(result.step_response_overshoot, 1),
            'step_response_rise_time': round(result.step_response_rise_time, 4),
            'step_response_settling_time': round(result.step_response_settling_time, 4),
            'tracking_error_rms': round(result.tracking_error_rms, 2),
            'tracking_error_max': round(result.tracking_error_max, 2),
        }

    # Stability results
    response['stability'] = numpy_safe({
        'vibration_rms': stability_result.vibration_rms,
        'vibration_peak': stability_result.vibration_peak,
        'vibration_clipping': stability_result.vibration_clipping,
        'vibration_score': round(stability_result.vibration_score, 1),
        'attitude_error_rms': stability_result.attitude_error_rms,
        'attitude_error_max': stability_result.attitude_error_max,
        'attitude_score': round(stability_result.attitude_score, 1),
        'motor_balance': round(stability_result.motor_balance, 1),
        'motor_saturation_pct': stability_result.motor_saturation_pct,
        'battery_voltage_drop': round(stability_result.battery_voltage_drop, 2),
        'battery_min_voltage': round(stability_result.battery_min_voltage, 2),
        'battery_max_current': round(stability_result.battery_max_current, 1),
        'overall_score': round(stability_result.overall_score, 1),
    })

    # Vibration details
    for axis, detail in vibration_results.items():
        response['vibration'][axis] = numpy_safe({
            'rms': round(detail.rms, 2),
            'peak': round(detail.peak, 2),
            'dominant_freq': round(detail.dominant_freq, 1),
            'harmonic_freqs': detail.harmonic_freqs,
            'severity': detail.severity,
        })

    # Recommendations
    for rec in recommendations:
        response['recommendations'].append({
            'severity': rec.severity,
            'category': rec.category,
            'title': rec.title,
            'description': rec.description,
            'steps': rec.steps,
            'param_changes': numpy_safe(rec.param_changes),
        })

    # Chart data
    response['charts'] = _build_chart_data(fd, pid_results, vibration_results)

    return jsonify(numpy_safe(response))


def _downsample(arr, max_points=2000):
    if arr is None or len(arr) <= max_points:
        return arr
    step = len(arr) // max_points
    return arr[::step]


def _build_chart_data(fd, pid_results, vibration_results):
    charts = {}

    # Gyro time series
    if fd.gyro is not None:
        charts['gyro'] = {
            'time': _downsample(fd.gyro.index.values).tolist(),
        }
        for axis in ['roll', 'pitch', 'yaw']:
            if axis in fd.gyro.columns:
                charts['gyro'][axis] = _downsample(fd.gyro[axis].values).tolist()

    # Attitude tracking
    if fd.attitude is not None:
        charts['attitude'] = {
            'time': _downsample(fd.attitude.index.values).tolist(),
        }
        for axis in ['roll', 'pitch', 'yaw']:
            if axis in fd.attitude.columns:
                charts['attitude'][f'{axis}_actual'] = _downsample(fd.attitude[axis].values).tolist()

    if fd.attitude_target is not None:
        if 'attitude' not in charts:
            charts['attitude'] = {
                'time': _downsample(fd.attitude_target.index.values).tolist(),
            }
        for axis in ['roll', 'pitch', 'yaw']:
            if axis in fd.attitude_target.columns:
                target_vals = np.interp(
                    np.array(charts['attitude']['time']),
                    fd.attitude_target.index.values,
                    fd.attitude_target[axis].values
                )
                charts['attitude'][f'{axis}_target'] = target_vals.tolist()

    # Motor output
    if fd.motor_output is not None:
        charts['motors'] = {
            'time': _downsample(fd.motor_output.index.values).tolist(),
        }
        for col in fd.motor_output.columns:
            charts['motors'][col] = _downsample(fd.motor_output[col].values).tolist()

    # PID output
    if fd.pid_output is not None:
        charts['pid_output'] = {
            'time': _downsample(fd.pid_output.index.values).tolist(),
        }
        for col in fd.pid_output.columns:
            if col in fd.pid_output.columns:
                charts['pid_output'][col] = _downsample(fd.pid_output[col].values).tolist()

    # FFT / PSD for each axis
    for axis, result in pid_results.items():
        if result.fft_freqs is not None and len(result.fft_freqs) > 0:
            max_freq_idx = np.searchsorted(result.fft_freqs, 500)
            charts[f'fft_{axis}'] = {
                'freqs': result.fft_freqs[:max_freq_idx].tolist(),
                'magnitude': result.fft_magnitude[:max_freq_idx].tolist(),
            }
        if result.psd_freqs is not None and len(result.psd_freqs) > 0:
            max_freq_idx = np.searchsorted(result.psd_freqs, 500)
            charts[f'psd_{axis}'] = {
                'freqs': result.psd_freqs[:max_freq_idx].tolist(),
                'power': result.psd_power[:max_freq_idx].tolist(),
            }

    # Vibration FFT
    for axis, detail in vibration_results.items():
        if detail.fft_freqs is not None and len(detail.fft_freqs) > 0:
            charts[f'vibe_fft_{axis}'] = {
                'freqs': detail.fft_freqs.tolist(),
                'magnitude': detail.fft_magnitude.tolist(),
            }

    # Battery
    if fd.battery is not None:
        charts['battery'] = {
            'time': _downsample(fd.battery.index.values).tolist(),
        }
        for col in fd.battery.columns:
            charts['battery'][col] = _downsample(fd.battery[col].values).tolist()

    # RC Input
    if fd.rc_input is not None:
        charts['rc_input'] = {
            'time': _downsample(fd.rc_input.index.values).tolist(),
        }
        for col in fd.rc_input.columns:
            charts['rc_input'][col] = _downsample(fd.rc_input[col].values).tolist()

    return charts


if __name__ == '__main__':
    import webbrowser
    port = 10917
    print(f"\n{'='*60}")
    print(f"  드론 비행 로그 분석기 (Drone Flight Log Analyzer)")
    print(f"  브라우저에서 http://localhost:{port} 에 접속하세요")
    print(f"{'='*60}\n")
    print("지원 파일: .ulg/.ulog (PX4), .bbl/.bfl/.csv (Betaflight/iNav), .bin/.log (ArduPilot)")
    print("종료하려면 Ctrl+C를 누르세요\n")

    webbrowser.open(f'http://localhost:{port}')
    app.run(host='0.0.0.0', port=port, debug=False)
