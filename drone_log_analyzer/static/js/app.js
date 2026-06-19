let sessionId = null;
let analysisData = null;
const PLOTLY_LAYOUT = {
    paper_bgcolor: '#1a1d27',
    plot_bgcolor: '#0f1117',
    font: { color: '#e4e4e7', family: '-apple-system, sans-serif' },
    margin: { t: 40, r: 20, b: 50, l: 60 },
    xaxis: { gridcolor: '#2a2d3a', zerolinecolor: '#2a2d3a' },
    yaxis: { gridcolor: '#2a2d3a', zerolinecolor: '#2a2d3a' },
    legend: { bgcolor: 'rgba(0,0,0,0)', font: { size: 11 } },
};
const PLOTLY_CONFIG = { responsive: true, displayModeBar: true, displaylogo: false };
const COLORS = {
    roll: '#3b82f6', pitch: '#22c55e', yaw: '#f59e0b',
    target: '#ef4444', motor0: '#3b82f6', motor1: '#22c55e',
    motor2: '#f59e0b', motor3: '#ef4444', p: '#3b82f6',
    i: '#22c55e', d: '#f59e0b', f: '#a855f7',
    x: '#3b82f6', y: '#22c55e', z: '#f59e0b',
};

// ─── Upload ───
const dropZone = document.getElementById('drop-zone');
const fileInput = document.getElementById('file-input');

dropZone.addEventListener('click', () => fileInput.click());
dropZone.addEventListener('dragover', e => { e.preventDefault(); dropZone.classList.add('dragover'); });
dropZone.addEventListener('dragleave', () => dropZone.classList.remove('dragover'));
dropZone.addEventListener('drop', e => {
    e.preventDefault();
    dropZone.classList.remove('dragover');
    if (e.dataTransfer.files.length) uploadFile(e.dataTransfer.files[0]);
});
fileInput.addEventListener('change', e => { if (e.target.files.length) uploadFile(e.target.files[0]); });

function uploadFile(file) {
    const progress = document.getElementById('upload-progress');
    const fill = progress.querySelector('.progress-fill');
    const text = progress.querySelector('.progress-text');
    progress.style.display = 'block';
    fill.style.width = '0%';
    text.textContent = `업로드 중: ${file.name}`;

    const formData = new FormData();
    formData.append('file', file);

    const xhr = new XMLHttpRequest();
    xhr.open('POST', '/api/upload');

    xhr.upload.onprogress = e => {
        if (e.lengthComputable) {
            const pct = (e.loaded / e.total * 100).toFixed(0);
            fill.style.width = pct + '%';
            text.textContent = `업로드 중: ${pct}%`;
        }
    };

    xhr.onload = () => {
        if (xhr.status === 200) {
            const data = JSON.parse(xhr.responseText);
            sessionId = data.session_id;
            fill.style.width = '100%';
            text.textContent = '파싱 완료!';
            showSummary(data);
        } else {
            const err = JSON.parse(xhr.responseText);
            text.textContent = `오류: ${err.error}`;
            fill.style.width = '100%';
            fill.style.background = 'var(--danger)';
        }
    };

    xhr.onerror = () => {
        text.textContent = '업로드 실패. 서버 연결을 확인하세요.';
    };

    xhr.send(formData);
}

// ─── Summary ───
function showSummary(data) {
    const section = document.getElementById('summary-section');
    const grid = document.getElementById('summary-grid');
    section.style.display = 'block';

    const sourceNames = {
        px4: 'PX4 / QGroundControl',
        ardupilot: 'ArduPilot / Mission Planner',
        betaflight: 'Betaflight',
        inav: 'iNav',
    };
    const fcNames = {
        pixhawk: 'Pixhawk',
        matek: 'Matek',
        speedybee: 'SpeedyBee',
        unknown: '자동 감지',
    };

    const items = [
        { label: '제어 소프트웨어', value: sourceNames[data.source_type] || data.source_type },
        { label: 'FC 타입', value: fcNames[data.fc_type] || data.fc_type },
        { label: '펌웨어', value: data.firmware_version || 'N/A' },
        { label: '비행 시간', value: formatDuration(data.duration_sec) },
        { label: '샘플 레이트', value: data.sample_rate > 0 ? `${data.sample_rate} Hz` : 'N/A' },
        { label: '자이로 데이터', value: data.has_gyro ? '✓' : '✗' },
        { label: '가속도 데이터', value: data.has_accel ? '✓' : '✗' },
        { label: '자세 데이터', value: data.has_attitude ? '✓' : '✗' },
        { label: 'PID 데이터', value: data.has_pid ? '✓' : '✗' },
        { label: '모터 출력', value: data.has_motor ? '✓' : '✗' },
        { label: '배터리', value: data.has_battery ? '✓' : '✗' },
        { label: 'RC 입력', value: data.has_rc ? '✓' : '✗' },
    ];

    grid.innerHTML = items.map(i => `
        <div class="summary-item">
            <div class="label">${i.label}</div>
            <div class="value">${i.value}</div>
        </div>
    `).join('');

    if (data.pid_params && Object.keys(data.pid_params).length > 0) {
        let pidHtml = '<div class="section-title" style="grid-column:1/-1;margin-top:1rem">현재 PID 파라미터</div>';
        for (const [axis, params] of Object.entries(data.pid_params)) {
            if (typeof params === 'object') {
                pidHtml += `<div class="summary-item">
                    <div class="label">${axis.toUpperCase()}</div>
                    <div class="value">P:${params.P || 0} I:${params.I || 0} D:${params.D || 0}</div>
                </div>`;
            }
        }
        grid.innerHTML += pidHtml;
    }

    document.getElementById('analyze-btn').onclick = runAnalysis;
    section.scrollIntoView({ behavior: 'smooth' });
}

// ─── Analysis ───
async function runAnalysis() {
    const btn = document.getElementById('analyze-btn');
    const progress = document.getElementById('analyze-progress');
    btn.disabled = true;
    progress.style.display = 'block';

    try {
        const resp = await fetch('/api/analyze', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ session_id: sessionId }),
        });
        analysisData = await resp.json();
        if (analysisData.error) {
            alert(analysisData.error);
            return;
        }
        showResults();
    } catch (e) {
        alert('분석 중 오류 발생: ' + e.message);
    } finally {
        btn.disabled = false;
        progress.style.display = 'none';
    }
}

// ─── Results ───
function showResults() {
    document.getElementById('results-section').style.display = 'block';
    setupTabs();
    renderOverview();
    renderPID('roll');
    renderStability();
    renderVibration();
    renderAllCharts();
    renderRecommendations();
    document.getElementById('results-section').scrollIntoView({ behavior: 'smooth' });
}

function setupTabs() {
    document.querySelectorAll('.tab').forEach(tab => {
        tab.addEventListener('click', () => {
            document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
            document.querySelectorAll('.tab-content').forEach(c => c.classList.remove('active'));
            tab.classList.add('active');
            document.getElementById('tab-' + tab.dataset.tab).classList.add('active');
            if (tab.dataset.tab === 'charts') {
                setTimeout(() => window.dispatchEvent(new Event('resize')), 100);
            }
        });
    });

    document.querySelectorAll('.axis-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            document.querySelectorAll('.axis-btn').forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
            renderPID(btn.dataset.axis);
        });
    });
}

// ─── Overview ───
function renderOverview() {
    const grid = document.getElementById('scores-grid');
    const d = analysisData;

    const scores = [];
    if (d.pid) {
        const avgPid = Object.values(d.pid).reduce((s, p) => s + p.score, 0) / Object.keys(d.pid).length;
        scores.push({ label: 'PID 튜닝', score: avgPid, details: Object.entries(d.pid).map(([a, p]) => `${a.toUpperCase()}: ${p.score.toFixed(0)}점`).join(' / ') });
    }
    if (d.stability) {
        scores.push({ label: '진동 수준', score: d.stability.vibration_score, details: Object.entries(d.stability.vibration_rms || {}).map(([a, v]) => `${a}: ${v.toFixed(1)} m/s²`).join(' / ') });
        scores.push({ label: '자세 추적', score: d.stability.attitude_score, details: Object.entries(d.stability.attitude_error_rms || {}).map(([a, v]) => `${a}: ${v.toFixed(1)}°`).join(' / ') });
        scores.push({ label: '모터 균형', score: d.stability.motor_balance, details: `전압 강하: ${d.stability.battery_voltage_drop}V` });
    }

    grid.innerHTML = scores.map(s => {
        const cls = s.score >= 70 ? 'score-good' : s.score >= 40 ? 'score-ok' : 'score-bad';
        return `<div class="score-card">
            <h3>${s.label}</h3>
            <div class="score-circle ${cls}" style="--score:${s.score}">
                <span>${s.score.toFixed(0)}</span>
            </div>
            <p style="color:var(--text-dim);font-size:0.85rem">${s.details}</p>
        </div>`;
    }).join('');
}

// ─── PID ───
function renderPID(axis) {
    const container = document.getElementById('pid-details');
    const chartContainer = document.getElementById('pid-charts');
    const p = analysisData.pid[axis];
    if (!p) {
        container.innerHTML = '<p>이 축의 PID 데이터가 없습니다.</p>';
        return;
    }

    const statusOf = (val, good, warn) => val <= good ? 'status-good' : val <= warn ? 'status-warning' : 'status-bad';
    const axisKr = { roll: '롤(Roll)', pitch: '피치(Pitch)', yaw: '요(Yaw)' }[axis];

    container.innerHTML = `
        <div class="card">
            <h3>${axisKr} PID 분석 결과 (점수: ${p.score.toFixed(0)}/100)</h3>
            <table class="detail-table">
                <tr><th>항목</th><th>값</th><th>상태</th></tr>
                <tr><td>자이로 노이즈 (RMS)</td><td>${p.noise_level} °/s</td>
                    <td class="${statusOf(p.noise_level, 10, 30)}">${p.noise_level <= 10 ? '양호' : p.noise_level <= 30 ? '주의' : '심각'}</td></tr>
                <tr><td>지배 주파수</td><td>${p.dominant_freq} Hz</td><td>-</td></tr>
                <tr><td>발진 감지</td><td>${p.oscillation_detected ? '감지됨' : '없음'}</td>
                    <td class="${p.oscillation_detected ? 'status-bad' : 'status-good'}">${p.oscillation_detected ? '⚠️ 조치 필요' : '정상'}</td></tr>
                <tr><td>D텀 노이즈 비율</td><td>${p.d_term_noise_ratio}</td>
                    <td class="${statusOf(p.d_term_noise_ratio, 0.3, 0.5)}">${p.d_term_noise_ratio <= 0.3 ? '양호' : p.d_term_noise_ratio <= 0.5 ? '주의' : '과다'}</td></tr>
                <tr><td>P텀 RMS</td><td>${p.p_term_rms}</td><td>-</td></tr>
                <tr><td>I텀 RMS</td><td>${p.i_term_rms}</td><td>-</td></tr>
                <tr><td>D텀 RMS</td><td>${p.d_term_rms}</td><td>-</td></tr>
                <tr><td>오버슈트</td><td>${p.step_response_overshoot}%</td>
                    <td class="${statusOf(p.step_response_overshoot, 15, 30)}">${p.step_response_overshoot <= 15 ? '양호' : p.step_response_overshoot <= 30 ? '주의' : '과다'}</td></tr>
                <tr><td>추적 오차 (RMS)</td><td>${p.tracking_error_rms}°</td>
                    <td class="${statusOf(p.tracking_error_rms, 3, 5)}">${p.tracking_error_rms <= 3 ? '양호' : p.tracking_error_rms <= 5 ? '주의' : '과다'}</td></tr>
            </table>
        </div>
    `;

    chartContainer.innerHTML = '';
    const charts = analysisData.charts;

    // FFT chart
    if (charts[`fft_${axis}`]) {
        const div = createChartDiv(chartContainer, `${axisKr} FFT 스펙트럼`);
        Plotly.newPlot(div, [{
            x: charts[`fft_${axis}`].freqs,
            y: charts[`fft_${axis}`].magnitude,
            type: 'scatter', mode: 'lines',
            line: { color: COLORS[axis], width: 1 },
            name: '크기',
        }], {
            ...PLOTLY_LAYOUT,
            title: `${axisKr} 자이로 FFT 스펙트럼`,
            xaxis: { ...PLOTLY_LAYOUT.xaxis, title: '주파수 (Hz)' },
            yaxis: { ...PLOTLY_LAYOUT.yaxis, title: '크기 (°/s)' },
        }, PLOTLY_CONFIG);
    }

    // PSD chart
    if (charts[`psd_${axis}`]) {
        const div = createChartDiv(chartContainer, `${axisKr} PSD (파워 스펙트럼 밀도)`);
        Plotly.newPlot(div, [{
            x: charts[`psd_${axis}`].freqs,
            y: charts[`psd_${axis}`].power,
            type: 'scatter', mode: 'lines',
            line: { color: COLORS[axis], width: 1 },
            fill: 'tozeroy', fillcolor: COLORS[axis] + '20',
            name: 'PSD',
        }], {
            ...PLOTLY_LAYOUT,
            title: `${axisKr} 파워 스펙트럼 밀도`,
            xaxis: { ...PLOTLY_LAYOUT.xaxis, title: '주파수 (Hz)' },
            yaxis: { ...PLOTLY_LAYOUT.yaxis, title: '파워 (dB)' },
        }, PLOTLY_CONFIG);
    }

    // PID output chart
    if (charts.pid_output) {
        const traces = [];
        for (const suffix of ['p', 'i', 'd', 'f']) {
            const col = `${axis}_${suffix}`;
            if (charts.pid_output[col]) {
                traces.push({
                    x: charts.pid_output.time,
                    y: charts.pid_output[col],
                    type: 'scatter', mode: 'lines',
                    line: { width: 1, color: COLORS[suffix] },
                    name: suffix.toUpperCase() + '텀',
                });
            }
        }
        if (traces.length > 0) {
            const div = createChartDiv(chartContainer, `${axisKr} PID 출력`);
            Plotly.newPlot(div, traces, {
                ...PLOTLY_LAYOUT,
                title: `${axisKr} PID 출력 (P/I/D/FF)`,
                xaxis: { ...PLOTLY_LAYOUT.xaxis, title: '시간 (초)' },
                yaxis: { ...PLOTLY_LAYOUT.yaxis, title: 'PID 출력' },
            }, PLOTLY_CONFIG);
        }
    }
}

// ─── Stability ───
function renderStability() {
    const container = document.getElementById('stability-details');
    const chartContainer = document.getElementById('stability-charts');
    const s = analysisData.stability;

    let html = '<div class="card"><h3>안정성 분석 결과</h3>';

    // Vibration table
    html += '<h4 class="section-title">진동 수준</h4><table class="detail-table">';
    html += '<tr><th>축</th><th>RMS (m/s²)</th><th>피크 (m/s²)</th><th>클리핑 (%)</th><th>상태</th></tr>';
    for (const axis of ['x', 'y', 'z']) {
        const rms = s.vibration_rms[axis] || 0;
        const peak = s.vibration_peak[axis] || 0;
        const clip = s.vibration_clipping[axis] || 0;
        const status = rms < 3 ? '양호' : rms < 15 ? '주의' : '심각';
        const cls = rms < 3 ? 'status-good' : rms < 15 ? 'status-warning' : 'status-bad';
        html += `<tr><td>${axis.toUpperCase()}</td><td>${rms.toFixed(2)}</td><td>${peak.toFixed(2)}</td><td>${clip.toFixed(2)}</td><td class="${cls}">${status}</td></tr>`;
    }
    html += '</table>';

    // Attitude tracking
    html += '<h4 class="section-title">자세 추적 정확도</h4><table class="detail-table">';
    html += '<tr><th>축</th><th>RMS 오차 (°)</th><th>최대 오차 (°)</th><th>상태</th></tr>';
    for (const axis of ['roll', 'pitch', 'yaw']) {
        const rms = s.attitude_error_rms[axis] || 0;
        const max = s.attitude_error_max[axis] || 0;
        const status = rms < 2 ? '양호' : rms < 5 ? '주의' : '과다';
        const cls = rms < 2 ? 'status-good' : rms < 5 ? 'status-warning' : 'status-bad';
        html += `<tr><td>${axis.toUpperCase()}</td><td>${rms.toFixed(2)}</td><td>${max.toFixed(2)}</td><td class="${cls}">${status}</td></tr>`;
    }
    html += '</table>';

    // Motor & Battery
    html += '<h4 class="section-title">모터 & 배터리</h4>';
    html += `<div class="metric-row"><span class="metric-label">모터 균형도</span><span class="metric-value">${s.motor_balance.toFixed(1)}%</span></div>`;
    for (const [motor, pct] of Object.entries(s.motor_saturation_pct || {})) {
        html += `<div class="metric-row"><span class="metric-label">${motor} 포화율</span><span class="metric-value">${pct.toFixed(1)}%</span></div>`;
    }
    html += `<div class="metric-row"><span class="metric-label">최저 배터리 전압</span><span class="metric-value">${s.battery_min_voltage}V</span></div>`;
    html += `<div class="metric-row"><span class="metric-label">전압 강하</span><span class="metric-value">${s.battery_voltage_drop}V</span></div>`;
    html += `<div class="metric-row"><span class="metric-label">최대 전류</span><span class="metric-value">${s.battery_max_current}A</span></div>`;

    html += '</div>';
    container.innerHTML = html;

    // Charts
    chartContainer.innerHTML = '';
    const charts = analysisData.charts;

    // Attitude tracking chart
    if (charts.attitude) {
        for (const axis of ['roll', 'pitch', 'yaw']) {
            if (charts.attitude[`${axis}_actual`]) {
                const traces = [{
                    x: charts.attitude.time,
                    y: charts.attitude[`${axis}_actual`],
                    type: 'scatter', mode: 'lines',
                    line: { color: COLORS[axis], width: 1 },
                    name: `${axis} 실제`,
                }];
                if (charts.attitude[`${axis}_target`]) {
                    traces.push({
                        x: charts.attitude.time,
                        y: charts.attitude[`${axis}_target`],
                        type: 'scatter', mode: 'lines',
                        line: { color: COLORS.target, width: 1, dash: 'dash' },
                        name: `${axis} 목표`,
                    });
                }
                const div = createChartDiv(chartContainer, `${axis.toUpperCase()} 자세 추적`);
                Plotly.newPlot(div, traces, {
                    ...PLOTLY_LAYOUT,
                    title: `${axis.toUpperCase()} 자세 추적 (목표 vs 실제)`,
                    xaxis: { ...PLOTLY_LAYOUT.xaxis, title: '시간 (초)' },
                    yaxis: { ...PLOTLY_LAYOUT.yaxis, title: '각도 (°)' },
                }, PLOTLY_CONFIG);
            }
        }
    }

    // Motor output chart
    if (charts.motors) {
        const traces = [];
        for (const col of Object.keys(charts.motors)) {
            if (col === 'time') continue;
            traces.push({
                x: charts.motors.time,
                y: charts.motors[col],
                type: 'scatter', mode: 'lines',
                line: { width: 1, color: COLORS[col] || '#888' },
                name: col,
            });
        }
        if (traces.length) {
            const div = createChartDiv(chartContainer, '모터 출력');
            Plotly.newPlot(div, traces, {
                ...PLOTLY_LAYOUT,
                title: '모터 출력',
                xaxis: { ...PLOTLY_LAYOUT.xaxis, title: '시간 (초)' },
                yaxis: { ...PLOTLY_LAYOUT.yaxis, title: '출력' },
            }, PLOTLY_CONFIG);
        }
    }
}

// ─── Vibration ───
function renderVibration() {
    const container = document.getElementById('vibration-details');
    const chartContainer = document.getElementById('vibration-charts');
    const v = analysisData.vibration;

    let html = '<div class="card"><h3>진동 주파수 분석</h3><table class="detail-table">';
    html += '<tr><th>축</th><th>RMS (m/s²)</th><th>피크 (m/s²)</th><th>지배 주파수 (Hz)</th><th>고조파</th><th>상태</th></tr>';
    for (const [axis, d] of Object.entries(v)) {
        const cls = d.severity === 'good' ? 'status-good' : d.severity === 'warning' ? 'status-warning' : 'status-bad';
        const sevKr = { good: '양호', warning: '주의', critical: '심각' }[d.severity];
        const harmonics = (d.harmonic_freqs || []).map(f => f.toFixed(0) + 'Hz').join(', ') || '-';
        html += `<tr><td>${axis.toUpperCase()}</td><td>${d.rms.toFixed(2)}</td><td>${d.peak.toFixed(2)}</td><td>${d.dominant_freq.toFixed(1)}</td><td>${harmonics}</td><td class="${cls}">${sevKr}</td></tr>`;
    }
    html += '</table></div>';
    container.innerHTML = html;

    // Vibration FFT charts
    chartContainer.innerHTML = '';
    const charts = analysisData.charts;
    for (const axis of ['x', 'y', 'z']) {
        if (charts[`vibe_fft_${axis}`]) {
            const axisKr = { x: 'X(전후)', y: 'Y(좌우)', z: 'Z(상하)' }[axis];
            const div = createChartDiv(chartContainer, `${axisKr} 가속도계 FFT`);
            Plotly.newPlot(div, [{
                x: charts[`vibe_fft_${axis}`].freqs,
                y: charts[`vibe_fft_${axis}`].magnitude,
                type: 'scatter', mode: 'lines',
                line: { color: COLORS[axis], width: 1 },
                fill: 'tozeroy', fillcolor: COLORS[axis] + '20',
            }], {
                ...PLOTLY_LAYOUT,
                title: `${axisKr} 가속도계 주파수 스펙트럼`,
                xaxis: { ...PLOTLY_LAYOUT.xaxis, title: '주파수 (Hz)' },
                yaxis: { ...PLOTLY_LAYOUT.yaxis, title: '크기 (m/s²)' },
            }, PLOTLY_CONFIG);
        }
    }
}

// ─── All Charts ───
function renderAllCharts() {
    const container = document.getElementById('all-charts');
    container.innerHTML = '';
    const charts = analysisData.charts;

    // Gyro
    if (charts.gyro) {
        const traces = ['roll', 'pitch', 'yaw'].filter(a => charts.gyro[a]).map(a => ({
            x: charts.gyro.time, y: charts.gyro[a],
            type: 'scatter', mode: 'lines',
            line: { width: 1, color: COLORS[a] }, name: a.toUpperCase(),
        }));
        const div = createChartDiv(container, '자이로 데이터');
        Plotly.newPlot(div, traces, {
            ...PLOTLY_LAYOUT,
            title: '자이로 (각속도) 시계열',
            xaxis: { ...PLOTLY_LAYOUT.xaxis, title: '시간 (초)' },
            yaxis: { ...PLOTLY_LAYOUT.yaxis, title: '각속도 (°/s)' },
        }, PLOTLY_CONFIG);
    }

    // RC Input
    if (charts.rc_input) {
        const traces = ['roll', 'pitch', 'yaw', 'throttle'].filter(a => charts.rc_input[a]).map(a => ({
            x: charts.rc_input.time, y: charts.rc_input[a],
            type: 'scatter', mode: 'lines',
            line: { width: 1 }, name: a.toUpperCase(),
        }));
        const div = createChartDiv(container, 'RC 입력');
        Plotly.newPlot(div, traces, {
            ...PLOTLY_LAYOUT,
            title: 'RC 조종 입력',
            xaxis: { ...PLOTLY_LAYOUT.xaxis, title: '시간 (초)' },
            yaxis: { ...PLOTLY_LAYOUT.yaxis, title: '입력값' },
        }, PLOTLY_CONFIG);
    }

    // Battery
    if (charts.battery) {
        const traces = [];
        if (charts.battery.voltage) {
            traces.push({
                x: charts.battery.time, y: charts.battery.voltage,
                type: 'scatter', mode: 'lines',
                line: { color: '#f59e0b', width: 1.5 }, name: '전압 (V)', yaxis: 'y',
            });
        }
        if (charts.battery.current) {
            traces.push({
                x: charts.battery.time, y: charts.battery.current,
                type: 'scatter', mode: 'lines',
                line: { color: '#ef4444', width: 1 }, name: '전류 (A)', yaxis: 'y2',
            });
        }
        if (traces.length) {
            const div = createChartDiv(container, '배터리');
            Plotly.newPlot(div, traces, {
                ...PLOTLY_LAYOUT,
                title: '배터리 전압 / 전류',
                xaxis: { ...PLOTLY_LAYOUT.xaxis, title: '시간 (초)' },
                yaxis: { ...PLOTLY_LAYOUT.yaxis, title: '전압 (V)', side: 'left' },
                yaxis2: { ...PLOTLY_LAYOUT.yaxis, title: '전류 (A)', side: 'right', overlaying: 'y' },
            }, PLOTLY_CONFIG);
        }
    }
}

// ─── Recommendations ───
function renderRecommendations() {
    const container = document.getElementById('recommendations-list');
    const recs = analysisData.recommendations || [];

    if (recs.length === 0) {
        container.innerHTML = '<div class="card"><p>분석 결과, 특별한 권장 사항이 없습니다. 현재 설정이 양호합니다.</p></div>';
        return;
    }

    // Count by severity
    const counts = { critical: 0, warning: 0, info: 0 };
    recs.forEach(r => counts[r.severity]++);

    let html = `<div class="card" style="margin-bottom:1.5rem">
        <h3>분석 결과 요약</h3>
        <p>총 ${recs.length}개 항목 —
        <span class="status-bad">긴급 ${counts.critical}개</span> ·
        <span class="status-warning">주의 ${counts.warning}개</span> ·
        <span style="color:var(--info)">정보 ${counts.info}개</span></p>
    </div>`;

    for (const rec of recs) {
        const sevKr = { critical: '긴급', warning: '주의', info: '정보' }[rec.severity];
        html += `<div class="rec-card ${rec.severity}">
            <div class="rec-header">
                <span class="rec-badge ${rec.severity}">${sevKr}</span>
                <span class="rec-category">${rec.category}</span>
                <span class="rec-title">${rec.title}</span>
            </div>
            <p class="rec-description">${rec.description}</p>`;

        if (rec.steps && rec.steps.length > 0) {
            html += '<div class="rec-steps"><ol>';
            for (const step of rec.steps) {
                const cleanStep = step.replace(/^\d+\.\s*/, '');
                html += `<li>${cleanStep}</li>`;
            }
            html += '</ol></div>';
        }

        html += '</div>';
    }

    container.innerHTML = html;
}

// ─── Helpers ───
function createChartDiv(parent, title) {
    const wrapper = document.createElement('div');
    wrapper.className = 'chart-container';
    const h3 = document.createElement('h3');
    h3.textContent = title;
    wrapper.appendChild(h3);
    const div = document.createElement('div');
    div.style.width = '100%';
    div.style.height = '400px';
    wrapper.appendChild(div);
    parent.appendChild(wrapper);
    return div;
}

function formatDuration(sec) {
    if (sec < 60) return `${sec.toFixed(1)}초`;
    const m = Math.floor(sec / 60);
    const s = Math.floor(sec % 60);
    return `${m}분 ${s}초`;
}
