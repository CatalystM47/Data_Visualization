from dataclasses import dataclass, field


@dataclass
class Recommendation:
    severity: str       # critical, warning, info
    category: str       # PID, 필터, 진동, 모터, 배터리, 하드웨어
    title: str
    description: str
    steps: list         # step-by-step instructions
    param_changes: dict = field(default_factory=dict)  # {param_name: {current, suggested}}


class ParameterAdvisor:
    def generate_recommendations(self, flight_data, pid_results: dict,
                                  stability_result, vibration_results: dict) -> list:
        recommendations = []

        for axis, pid in pid_results.items():
            recommendations.extend(self._pid_recommendations(axis, pid, flight_data))

        recommendations.extend(self._stability_recommendations(stability_result, flight_data))
        recommendations.extend(self._vibration_recommendations(vibration_results, flight_data))
        recommendations.extend(self._motor_recommendations(stability_result, flight_data))
        recommendations.extend(self._battery_recommendations(stability_result))

        recommendations.sort(key=lambda r: {'critical': 0, 'warning': 1, 'info': 2}.get(r.severity, 3))
        return recommendations

    def _pid_recommendations(self, axis, pid_result, flight_data) -> list:
        recs = []
        axis_kr = {'roll': '롤(Roll)', 'pitch': '피치(Pitch)', 'yaw': '요(Yaw)'}[axis]
        source = flight_data.source_type

        # Oscillation detected
        if pid_result.oscillation_detected:
            freqs_str = ', '.join([f"{f['frequency']:.0f}Hz" for f in pid_result.oscillation_freqs[:3]])
            steps = self._get_oscillation_fix_steps(axis, pid_result, source, flight_data)
            recs.append(Recommendation(
                severity='critical',
                category='PID',
                title=f'{axis_kr} 축 진동/발진 감지',
                description=(
                    f'{axis_kr} 축에서 {freqs_str} 주파수의 발진이 감지되었습니다. '
                    f'이는 PID 게인이 과도하게 높거나, 필터 설정이 부적절하여 발생할 수 있습니다. '
                    f'즉시 조치하지 않으면 모터 과열, 프롭 손상, 비행 불안정으로 이어질 수 있습니다.'
                ),
                steps=steps,
            ))

        # High D-term noise
        if pid_result.d_term_noise_ratio > 0.5:
            steps = self._get_d_noise_fix_steps(axis, pid_result, source, flight_data)
            recs.append(Recommendation(
                severity='warning' if pid_result.d_term_noise_ratio < 0.8 else 'critical',
                category='PID',
                title=f'{axis_kr} 축 D텀 노이즈 과다',
                description=(
                    f'{axis_kr} 축의 D텀 노이즈 비율이 {pid_result.d_term_noise_ratio:.2f}입니다. '
                    f'D텀 노이즈가 P텀 대비 50%를 초과하면 모터에 불필요한 열이 발생하고 '
                    f'비행 효율이 저하됩니다. D 게인을 낮추거나 D텀 필터를 강화해야 합니다.'
                ),
                steps=steps,
            ))

        # Step response overshoot
        if pid_result.step_response_overshoot > 20:
            steps = self._get_overshoot_fix_steps(axis, pid_result, source, flight_data)
            severity = 'critical' if pid_result.step_response_overshoot > 40 else 'warning'
            recs.append(Recommendation(
                severity=severity,
                category='PID',
                title=f'{axis_kr} 축 과도한 오버슈트 ({pid_result.step_response_overshoot:.1f}%)',
                description=(
                    f'{axis_kr} 축 스텝 응답에서 {pid_result.step_response_overshoot:.1f}%의 오버슈트가 감지되었습니다. '
                    f'15% 이하가 이상적이며, 현재 수치는 기체가 명령된 자세를 초과했다가 '
                    f'되돌아오고 있음을 의미합니다. P 게인이 너무 높거나 D 게인이 부족할 수 있습니다.'
                ),
                steps=steps,
            ))

        # Poor tracking
        if pid_result.tracking_error_rms > 5:
            recs.append(Recommendation(
                severity='warning',
                category='PID',
                title=f'{axis_kr} 축 자세 추적 오차 과다',
                description=(
                    f'{axis_kr} 축의 RMS 추적 오차가 {pid_result.tracking_error_rms:.1f}°입니다. '
                    f'3° 이하가 이상적이며, 현재 기체가 명령된 자세를 정확히 따라가지 못하고 있습니다.'
                ),
                steps=[
                    f'1. P 게인을 현재 값에서 10-20% 올려보세요',
                    f'2. I 게인도 약간 올려 정상상태 오차를 줄이세요',
                    f'3. 변경 후 짧은 호버링 비행으로 개선 여부를 확인하세요',
                    f'4. 진동이 발생하면 P 게인을 다시 낮추세요',
                ],
            ))

        # Good PID score
        if pid_result.score >= 85 and not pid_result.oscillation_detected:
            recs.append(Recommendation(
                severity='info',
                category='PID',
                title=f'{axis_kr} 축 PID 튜닝 양호',
                description=(
                    f'{axis_kr} 축의 PID 튜닝 점수는 {pid_result.score:.0f}/100으로 양호합니다. '
                    f'현재 설정을 유지하시면 됩니다.'
                ),
                steps=[],
            ))

        return recs

    def _get_oscillation_fix_steps(self, axis, pid_result, source, flight_data) -> list:
        pid_params = flight_data.pid_params.get(axis, {})
        current_p = pid_params.get('P', '알 수 없음')
        current_d = pid_params.get('D', '알 수 없음')

        if source == 'betaflight':
            return [
                f'1. Betaflight Configurator를 열고 PID Tuning 탭으로 이동합니다',
                f'2. {axis.upper()} 축의 현재 P 값({current_p})을 20% 낮춰보세요',
                f'3. {axis.upper()} 축의 현재 D 값({current_d})을 15% 낮춰보세요',
                f'4. 필터 탭에서 Gyro Lowpass 필터가 활성화되어 있는지 확인하세요',
                f'5. D Term Lowpass 필터의 컷오프 주파수를 현재보다 낮춰보세요 (예: 150Hz → 120Hz)',
                f'6. 변경사항을 저장(Save)하고, 안전한 공간에서 짧은 호버링으로 테스트하세요',
                f'7. 블랙박스 로그를 다시 기록하여 개선 여부를 확인하세요',
            ]
        elif source == 'inav':
            return [
                f'1. INAV Configurator를 열고 PID Tuning 페이지로 이동합니다',
                f'2. {axis.upper()} 축의 P 값({current_p})을 20% 낮추세요',
                f'3. {axis.upper()} 축의 D 값({current_d})을 15% 낮추세요',
                f'4. Filtering 탭에서 Gyro LPF 설정을 확인하세요',
                f'5. 변경 후 저장하고 테스트 비행을 실시하세요',
            ]
        elif source == 'ardupilot':
            prefix = {'roll': 'ATC_RAT_RLL', 'pitch': 'ATC_RAT_PIT', 'yaw': 'ATC_RAT_YAW'}[axis]
            return [
                f'1. Mission Planner의 Config > Full Parameter List를 엽니다',
                f'2. {prefix}_P 값({current_p})을 20% 낮추세요',
                f'3. {prefix}_D 값({current_d})을 15% 낮추세요',
                f'4. INS_GYRO_FILTER 값을 확인하세요 (기본 20Hz, 필요시 낮춤)',
                f'5. Write Params를 눌러 FC에 저장합니다',
                f'6. 안전한 공간에서 호버링 테스트 후 로그를 다시 확인하세요',
            ]
        else:  # px4
            return [
                f'1. QGroundControl의 Vehicle Setup > PID Tuning으로 이동합니다',
                f'2. MC_{axis.upper()}RATE_P 값({current_p})을 20% 낮추세요',
                f'3. MC_{axis.upper()}RATE_D 값({current_d})을 15% 낮추세요',
                f'4. IMU_GYRO_CUTOFF 파라미터를 확인하세요',
                f'5. 변경사항을 적용하고 기체를 재부팅합니다',
                f'6. 호버링 테스트 후 새 로그를 확인하세요',
            ]

    def _get_d_noise_fix_steps(self, axis, pid_result, source, flight_data) -> list:
        pid_params = flight_data.pid_params.get(axis, {})
        current_d = pid_params.get('D', '알 수 없음')

        if source == 'betaflight':
            return [
                f'1. Betaflight Configurator > PID Tuning 탭을 엽니다',
                f'2. {axis.upper()} 축 D 값({current_d})을 25% 낮추세요',
                f'3. Filter 탭에서 D Term Lowpass 1 필터를 PT1으로 설정하세요',
                f'4. D Term Lowpass 컷오프를 100-120Hz로 설정하세요',
                f'5. D Term Lowpass 2도 활성화하고 컷오프를 200Hz로 설정하세요',
                f'6. 저장 후 테스트 비행에서 개선 여부를 확인하세요',
            ]
        elif source == 'ardupilot':
            prefix = {'roll': 'ATC_RAT_RLL', 'pitch': 'ATC_RAT_PIT', 'yaw': 'ATC_RAT_YAW'}[axis]
            return [
                f'1. Mission Planner > Full Parameter List를 엽니다',
                f'2. {prefix}_D({current_d})를 25% 낮추세요',
                f'3. {prefix}_FLTD (D텀 필터) 값을 확인/낮추세요 (기본 20Hz)',
                f'4. INS_GYRO_FILTER 값도 확인하세요',
                f'5. Write Params 후 테스트 비행을 실시하세요',
            ]
        else:
            return [
                f'1. D 게인({current_d})을 25% 낮추세요',
                f'2. D텀 로우패스 필터를 강화하세요 (컷오프 주파수 낮춤)',
                f'3. 자이로 필터 설정도 확인하세요',
                f'4. 변경 후 테스트 비행으로 확인하세요',
            ]

    def _get_overshoot_fix_steps(self, axis, pid_result, source, flight_data) -> list:
        pid_params = flight_data.pid_params.get(axis, {})
        current_p = pid_params.get('P', '알 수 없음')
        current_d = pid_params.get('D', '알 수 없음')

        return [
            f'1. P 게인({current_p})을 10-15% 낮추세요 — 오버슈트의 주요 원인입니다',
            f'2. D 게인({current_d})을 10% 올려보세요 — 오버슈트를 억제하는 역할입니다',
            f'3. 단, D 게인을 올릴 때 모터 소음이 증가하면 올리지 마세요',
            f'4. P를 먼저 낮추고, D를 조금씩 올리는 순서로 접근하세요',
            f'5. 변경 후 테스트 비행에서 스틱을 빠르게 움직여 응답을 확인하세요',
        ]

    def _stability_recommendations(self, stability_result, flight_data) -> list:
        recs = []

        if stability_result.vibration_score < 50:
            severity = 'critical' if stability_result.vibration_score < 30 else 'warning'
            vibe_str = ', '.join([f'{k}축: {v:.1f} m/s²'
                                  for k, v in stability_result.vibration_rms.items()])
            recs.append(Recommendation(
                severity=severity,
                category='진동',
                title='기체 진동 수준 과다',
                description=(
                    f'가속도계 진동 RMS 값: {vibe_str}. '
                    f'진동이 심하면 자세 추정 오차가 커지고, GPS 성능 저하, '
                    f'비정상적인 PID 동작을 유발합니다.'
                ),
                steps=[
                    '1. 프로펠러 밸런싱을 확인하세요 — 가장 흔한 진동 원인입니다',
                    '2. 프로펠러에 균열이나 손상이 없는지 검사하세요',
                    '3. 모터 마운트가 단단히 고정되어 있는지 확인하세요',
                    '4. FC(비행 컨트롤러)가 진동 흡수 마운트(댐퍼)에 장착되어 있는지 확인하세요',
                    '5. 프레임 나사가 풀어진 곳이 없는지 전체적으로 점검하세요',
                    '6. 모터 베어링 상태를 확인하세요 — 손으로 돌려서 걸리는 느낌이 있으면 교체',
                    '7. 개선 후 다시 비행하여 로그를 비교하세요',
                ],
            ))

        clipping = stability_result.vibration_clipping
        if any(v > 1.0 for v in clipping.values()):
            recs.append(Recommendation(
                severity='critical',
                category='진동',
                title='가속도계 클리핑 감지',
                description=(
                    '가속도계 값이 센서 범위를 초과하는 클리핑이 감지되었습니다. '
                    '이는 매우 심한 진동을 의미하며, 자세 추정에 심각한 오류를 유발합니다.'
                ),
                steps=[
                    '1. 즉시 비행을 중단하고 기체를 점검하세요',
                    '2. 프로펠러, 모터, 프레임 상태를 철저히 검사하세요',
                    '3. FC 마운팅에 진동 댐퍼를 추가하거나 교체하세요',
                    '4. 소프트 마운트 옵션을 활성화하세요 (해당되는 경우)',
                ],
            ))

        return recs

    def _vibration_recommendations(self, vibration_results, flight_data) -> list:
        recs = []

        for axis, detail in vibration_results.items():
            if detail.dominant_freq > 0 and detail.severity != 'good':
                axis_kr = {'x': 'X(전후)', 'y': 'Y(좌우)', 'z': 'Z(상하)'}[axis]
                harmonics = ', '.join([f'{f:.0f}Hz' for f in detail.harmonic_freqs[:3]])
                recs.append(Recommendation(
                    severity='info',
                    category='진동',
                    title=f'{axis_kr}축 주요 진동 주파수: {detail.dominant_freq:.0f}Hz',
                    description=(
                        f'{axis_kr}축의 주요 진동 주파수는 {detail.dominant_freq:.0f}Hz이며, '
                        f'고조파: {harmonics}. 이 주파수가 모터 RPM과 관련되는지 확인하세요.'
                    ),
                    steps=[
                        f'모터 RPM과 비교: {detail.dominant_freq:.0f}Hz = {detail.dominant_freq * 60:.0f} RPM (2블레이드 기준)',
                        '주파수가 모터 회전수와 일치하면 프롭 밸런스 문제입니다',
                        '일치하지 않으면 프레임 공진일 수 있으니 마운팅을 확인하세요',
                    ],
                ))

        return recs

    def _motor_recommendations(self, stability_result, flight_data) -> list:
        recs = []

        for motor, sat_pct in stability_result.motor_saturation_pct.items():
            if sat_pct > 10:
                motor_num = motor.replace('motor', '')
                recs.append(Recommendation(
                    severity='critical' if sat_pct > 30 else 'warning',
                    category='모터',
                    title=f'모터 {motor_num} 포화 감지 ({sat_pct:.1f}%)',
                    description=(
                        f'모터 {motor_num}이 출력 상한에 {sat_pct:.1f}% 시간 동안 도달했습니다. '
                        f'모터 포화는 FC가 기체를 제어할 수 없는 상태를 의미하며 위험합니다.'
                    ),
                    steps=[
                        '1. 기체 무게를 줄이거나 더 강한 모터/프롭 조합을 사용하세요',
                        '2. 무게중심(CG)이 정확한지 확인하세요 — 한쪽 모터만 포화되면 CG 문제',
                        '3. 모터/ESC 이상이 없는지 확인하세요',
                        '4. 최대 스로틀에서의 비행을 자제하세요',
                    ],
                ))

        if stability_result.motor_balance < 70 and stability_result.motor_balance > 0:
            recs.append(Recommendation(
                severity='warning',
                category='모터',
                title=f'모터 출력 불균형 (균형도: {stability_result.motor_balance:.0f}%)',
                description=(
                    '모터 간 평균 출력 차이가 큽니다. 이는 무게중심(CG) 편향, '
                    '모터/ESC 성능 차이, 또는 프레임 뒤틀림을 의미할 수 있습니다.'
                ),
                steps=[
                    '1. 배터리 위치를 조정하여 무게중심을 맞추세요',
                    '2. 모든 모터의 회전 방향과 프롭 장착이 올바른지 확인하세요',
                    '3. 각 모터를 개별로 돌려 RPM 차이가 없는지 확인하세요',
                    '4. ESC 캘리브레이션을 다시 실행하세요',
                    '5. 프레임이 휠거나 틀어지지 않았는지 확인하세요',
                ],
            ))

        return recs

    def _battery_recommendations(self, stability_result) -> list:
        recs = []

        if stability_result.battery_min_voltage > 0:
            cell_voltage = stability_result.battery_min_voltage
            if cell_voltage > 20:
                cells = round(cell_voltage / 3.7)
                cell_voltage = stability_result.battery_min_voltage / cells if cells > 0 else cell_voltage
            else:
                cells = max(1, round(cell_voltage / 3.7))
                cell_voltage = stability_result.battery_min_voltage / cells

            if cell_voltage < 3.3:
                recs.append(Recommendation(
                    severity='critical',
                    category='배터리',
                    title=f'배터리 과방전 위험 (최저 셀 전압: {cell_voltage:.2f}V)',
                    description=(
                        f'비행 중 셀당 최저 전압이 {cell_voltage:.2f}V까지 떨어졌습니다. '
                        f'3.3V 이하는 배터리 수명을 크게 단축시키며, 3.0V 이하는 영구 손상을 유발합니다.'
                    ),
                    steps=[
                        '1. 배터리 경고 전압을 높이세요 (셀당 3.5V 권장)',
                        '2. 비행 시간을 줄이거나 더 큰 용량의 배터리를 사용하세요',
                        '3. 배터리 페일세이프(자동 착륙/귀환)를 설정하세요',
                        '4. 현재 배터리의 내부 저항을 측정하여 열화 여부를 확인하세요',
                    ],
                ))

        if stability_result.battery_voltage_drop > 2.0:
            recs.append(Recommendation(
                severity='warning',
                category='배터리',
                title=f'비행 중 전압 강하 과다 ({stability_result.battery_voltage_drop:.1f}V)',
                description=(
                    f'비행 중 전압 강하가 {stability_result.battery_voltage_drop:.1f}V입니다. '
                    f'큰 전압 강하는 배터리 노화, 높은 전류 소모, 또는 커넥터 접촉 불량을 의미합니다.'
                ),
                steps=[
                    '1. 배터리 내부 저항을 측정하세요 (셀당 10mΩ 이상이면 교체 고려)',
                    '2. 배터리 커넥터와 배선 상태를 점검하세요',
                    '3. 필요 시 더 높은 C-레이팅의 배터리를 사용하세요',
                ],
            ))

        return recs
