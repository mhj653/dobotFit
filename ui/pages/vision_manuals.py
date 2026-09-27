from __future__ import annotations


def calibration_manual_html(method: str) -> str:
    common_style = """
    <style>
        body { font-family: 'Segoe UI', 'Malgun Gothic', Arial, sans-serif; color: #e5edf7; background: #0f172a; line-height: 1.45; }
        h2 { color: #93c5fd; margin-top: 0; }
        h3 { color: #f8fafc; margin-bottom: 6px; }
        ol { margin-top: 6px; padding-left: 24px; }
        li { margin-bottom: 8px; }
        .box { border: 1px solid #33445f; background: #111827; border-radius: 8px; padding: 10px 12px; margin: 10px 0; }
        .warn { color: #facc15; }
        code { color: #bfdbfe; background: #172033; padding: 2px 5px; border-radius: 4px; }
    </style>
    """
    if method == "Checkerboard":
        return (
            common_style
            + """
            <h2>Checkerboard Calibration Manual</h2>
            <div class="box">
                체커보드를 카메라로 인식하고, 같은 체커보드 기준점을 로봇 TCP로 찍어서
                <code>Camera XYZ -> Robot XYZ</code> 변환 행렬을 계산하는 방식입니다.
            </div>
            <h3>진행 순서</h3>
            <ol>
                <li><b>Method</b>를 <code>Checkerboard</code>로 선택합니다.</li>
                <li><b>Inner X / Inner Y / Square</b> 값을 실제 체커보드와 동일하게 입력합니다.<br>
                    예: 내부 코너가 7 x 6이고 한 칸이 20 mm이면 <code>Inner X=7, Inner Y=6, Square=20</code></li>
                <li><b>Live Preview</b>를 켜고 체커보드가 화면에 안정적으로 보이는지 확인합니다.</li>
                <li><b>1 Detect CB</b>를 눌러 체커보드 코너/중심이 검출되는지 확인합니다.</li>
                <li>로봇을 움직여 TCP를 체커보드의 동일한 기준점에 맞춥니다. 기준점은 매번 같은 물리 위치여야 합니다.</li>
                <li><b>2 GetPose</b>를 눌러 현재 로봇 위치를 가져오거나, Robot Pose Input에 X/Y/Z/RZ를 수동 입력합니다.</li>
                <li><b>3 Add Sample</b>로 카메라 좌표와 로봇 좌표를 한 세트로 저장합니다.</li>
                <li>체커보드를 다른 위치/높이로 이동해서 최소 3세트 이상, 권장 6세트 이상 반복합니다.</li>
                <li><b>4 Solve &amp;&amp; Apply</b>를 눌러 행렬을 계산하고, Quality의 mean/max error를 확인합니다.</li>
            </ol>
            <div class="box warn">
                좋은 캘리브레이션을 위해 샘플은 한 줄로만 찍지 말고 X/Y/Z 방향으로 분산시켜야 합니다.
                같은 높이와 비슷한 위치만 반복하면 행렬 품질이 나빠질 수 있습니다.
            </div>
            """
        )
    return (
        common_style
        + """
        <h2>Point Pair Calibration Manual</h2>
        <div class="box">
            Vision에서 찾은 대상 중심의 <code>Camera XYZ</code>와, 같은 실제 위치를 로봇 TCP로 찍은
            <code>Robot XYZ/RZ</code>를 쌍으로 저장해서 변환 행렬을 계산하는 방식입니다.
        </div>
        <h3>진행 순서</h3>
        <ol>
            <li><b>Method</b>를 <code>Point Pair</code>로 선택합니다.</li>
            <li>Live Vision에서 ROI, Threshold, Blob 파라미터를 조정해 대상 중심이 안정적으로 검출되게 합니다.</li>
            <li><b>1 Detect Point</b>를 눌러 Camera XYZ 값이 갱신되는지 확인합니다.</li>
            <li>로봇을 움직여 TCP를 카메라가 찾은 실제 대상 중심에 맞춥니다.</li>
            <li><b>2 GetPose</b>를 눌러 현재 로봇 위치를 가져오거나, Robot Pose Input에 X/Y/Z/RZ를 수동 입력합니다.</li>
            <li><b>3 Add Pair</b>를 눌러 카메라 좌표와 로봇 좌표를 한 세트로 저장합니다.</li>
            <li>대상을 다른 위치/높이로 옮겨 최소 3세트 이상, 권장 6세트 이상 반복합니다.</li>
            <li><b>4 Solve &amp;&amp; Apply</b>를 눌러 행렬을 계산하고, Quality의 mean/max error를 확인합니다.</li>
        </ol>
        <div class="box">
            예시: Camera XYZ <code>(12.0, -4.0, 310.0)</code> 와 Robot pose
            <code>(320.0, 80.0, 120.0, 90.0)</code> 를 한 pair로 저장합니다.
        </div>
        <div class="box warn">
            Add Pair 전에 반드시 카메라 검출점과 로봇 TCP가 같은 실제 물리 지점을 가리켜야 합니다.
        </div>
        """
    )
