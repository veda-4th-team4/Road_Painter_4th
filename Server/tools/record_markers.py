"""
1단계-A: 캘리브레이션 주행 중 마커 픽셀좌표 기록.

하는 일:
  - RTSP 스트림에서 매 프레임 ArUco 마커 검출 + cornerSubPix
  - 마커 중심 픽셀좌표 (u,v)를 타임스탬프와 함께 markers.csv 에 기록
  - 화면에 지금까지의 궤적을 그려주고,
    궤적이 '직선에 가까운지'를 실시간으로 경고 (← H 특이해지는 문제 방지)
  - 종료 시 바닥영역 추정용 배경 프레임 floor_bg.png 저장

사용법:
  1) 로봇 위에 ArUco 마커를 바닥과 평행하게(위를 보게) 부착.
  2) 이 스크립트 실행 → 화면 뜨면 로봇을 'ㄹ자/사각형'으로 주행시킴.
     (직진만 X. 반드시 횡방향 이동 포함해서 2D 면적을 덮을 것)
  3) 화면 좌상단 COVERAGE 막대가 초록이 되면 충분. q로 종료.
  4) 동시에 RPi4 쪽은 같은 시간 동안 odometry.csv (t,x,y,theta) 를 기록해야 함.
     ★ 두 로그의 타임스탬프가 같은 시계(epoch초) 기준이어야 동기화됨 → ICD로 합의.

출력: markers.csv , floor_bg.png
"""
import cv2, csv, time
import numpy as np

# ── 설정 ─────────────────────────────────────────────
RTSP_URL   = "rtsp://admin:5hanwha!@172.20.35.77:554/profile2/media.smp"
ARUCO_DICT = cv2.aruco.DICT_4X4_50   # 사용할 마커 사전
TARGET_ID  = None                    # 특정 ID만 추적하려면 숫자, 전체면 None
OUT_CSV    = "markers.csv"
BG_PNG     = "floor_bg.png"
# ────────────────────────────────────────────────────

def make_detector():
    d = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    try:  # OpenCV 4.7+ 신 API
        det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
        return lambda g: det.detectMarkers(g)
    except AttributeError:  # 구 API
        params = cv2.aruco.DetectorParameters_create()
        return lambda g: cv2.aruco.detectMarkers(g, d, parameters=params)

def coverage_ratio(pts):
    """궤적의 2D 퍼짐 정도. PCA 2번째축/1번째축 비율(0=직선, 1=원형)."""
    if len(pts) < 10:
        return 0.0
    a = np.asarray(pts, np.float32)
    a = a - a.mean(0)
    eig = np.linalg.eigvalsh(np.cov(a.T))
    return float(eig[0] / eig[1]) if eig[1] > 1e-9 else 0.0

detect = make_detector()
subpix_crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)

cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)
if not cap.isOpened():
    raise SystemExit("RTSP 연결 실패")

rows, traj = [], []
last_bg = None
print("주행 시작. 'ㄹ자'로 움직이세요. q=종료")
while True:
    ok, frame = cap.read()
    if not ok:
        time.sleep(0.05); continue
    t = time.time()                       # 카메라측 타임스탬프(epoch초)
    last_bg = frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detect(gray)

    if ids is not None:
        for c, i in zip(corners, ids.flatten()):
            if TARGET_ID is not None and i != TARGET_ID:
                continue
            cv2.cornerSubPix(gray, c, (5, 5), (-1, -1), subpix_crit)
            uv = c.reshape(4, 2).mean(0)  # 마커 중심
            rows.append([f"{t:.4f}", int(i), f"{uv[0]:.3f}", f"{uv[1]:.3f}",
                         *[f"{v:.3f}" for v in c.reshape(-1)]])
            traj.append(uv)

    # ── 미리보기 ──
    view = cv2.resize(frame, (1280, 720)); sx, sy = 1280/frame.shape[1], 720/frame.shape[0]
    for p in traj:
        cv2.circle(view, (int(p[0]*sx), int(p[1]*sy)), 2, (0, 255, 255), -1)
    r = coverage_ratio(traj)
    col = (0, 200, 0) if r > 0.25 else (0, 0, 255)
    cv2.rectangle(view, (20, 20), (20 + int(300*min(r/0.25, 1)), 45), col, -1)
    cv2.putText(view, f"COVERAGE {r:.2f} (>0.25 OK)  pts={len(traj)}",
                (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
    cv2.imshow("calib drive", view)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release(); cv2.destroyAllWindows()

with open(OUT_CSV, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "id", "u", "v",
                "c0x","c0y","c1x","c1y","c2x","c2y","c3x","c3y"])
    w.writerows(rows)
if last_bg is not None:
    cv2.imwrite(BG_PNG, last_bg)

r = coverage_ratio(traj)
print(f"\n저장: {OUT_CSV} ({len(rows)}점), {BG_PNG}")
print(f"최종 커버리지 {r:.2f}", "→ 충분" if r > 0.25 else "→ 부족! 더 넓게 재주행 권장")
