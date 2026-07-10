"""
캡처한 프레임으로 렌즈 왜곡 계수 산출 (0단계).

전제: capture_frames.py로 calib_frames/ 에 15~20장 저장돼 있음.
설정: 체커보드의 '내부 코너' 개수와 사각형 한 변 길이(mm)를 실제 인쇄물에 맞게 입력.
      예) 10x7 격자 보드라면 내부 코너는 (9, 6).
출력: camera_calib.npz  (K = 카메라행렬, dist = 왜곡계수)
      RPi4 파트에 전달할 캘리브레이션 데이터가 이 파일.
"""
import cv2, glob, numpy as np

# ── 설정 ─────────────────────────────────────────────
PATTERN = (9, 6)      # ← 내부 코너 (cols, rows). 격자칸 수 - 1
SQUARE_MM = 25.0      # ← 사각형 한 변 실측(mm)
FRAME_DIR = "calib_frames"
# ────────────────────────────────────────────────────

# 월드 좌표계 상의 코너 좌표 (Z=0 평면, 실측 스케일)
objp = np.zeros((PATTERN[0] * PATTERN[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2) * SQUARE_MM

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objpoints, imgpoints, used = [], [], []
files = sorted(glob.glob(f"{FRAME_DIR}/*.png") + glob.glob(f"{FRAME_DIR}/*.jpg"))
if not files:
    raise SystemExit(f"{FRAME_DIR} 에 이미지 없음")

img_size = None
for f in files:
    img = cv2.imread(f)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_size = gray.shape[::-1]
    found, corners = cv2.findChessboardCorners(
        gray, PATTERN,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        print("코너 검출 실패:", f); continue
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    objpoints.append(objp); imgpoints.append(corners); used.append(f)
    print("OK:", f)

print(f"\n사용된 프레임 {len(used)}/{len(files)}")
if len(used) < 8:
    raise SystemExit("유효 프레임 부족 — 더 촬영 필요 (구석/기울기 다양하게)")

rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, img_size, None, None)

# 프레임별 재투영 오차
print(f"\n전체 RMS 재투영 오차: {rms:.4f} px  (목표 <1px, 이상적 <0.5px)")
for i, f in enumerate(used):
    proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
    err = cv2.norm(imgpoints[i], proj, cv2.NORM_L2) / len(proj)
    print(f"  {f}: {err:.3f} px")   # 특정 프레임만 오차 크면 그 장 빼고 재실행

np.savez("camera_calib.npz", K=K, dist=dist, img_size=img_size, rms=rms)
# C++(RPi4)가 읽을 수 있게 OpenCV FileStorage(.yml)로도 저장
fs = cv2.FileStorage("camera_calib.yml", cv2.FILE_STORAGE_WRITE)
fs.write("K", K); fs.write("dist", dist)
fs.write("img_w", int(img_size[0])); fs.write("img_h", int(img_size[1]))
fs.release()
print("\n저장: camera_calib.npz, camera_calib.yml")
print("K =\n", K)
print("dist =", dist.ravel())
