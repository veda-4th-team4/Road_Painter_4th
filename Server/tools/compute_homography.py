"""
1단계-B: 마커 픽셀좌표 + 오도메트리로 호모그래피 H 계산.

원리:
  - 마커 픽셀 (u,v)  ↔  같은 시각 로봇의 지면좌표 (X,Y, 오도메트리)  대응쌍을 만든다.
  - 픽셀은 먼저 undistortPoints 로 왜곡 제거(0단계 K,dist 사용)
    → H가 '왜곡보정된 좌표계' 위에 정의됨 → 3단계 실시간 파이프라인과 일치.
  - findHomography 로 H(픽셀→지면 미터) 산출.

★ 직선 주행 방어:
  대응점이 일직선에 가까우면 H가 특이(singular)해진다.
  아래에서 픽셀/월드 점의 PCA 퍼짐을 검사해서, 직선이면 중단하고 재주행을 요구한다.

입력:
  markers.csv     (record_markers.py 출력)
  odometry.csv    (RPi4 출력; 컬럼: t,x,y[,theta], t=epoch초, x/y=미터)
  camera_calib.npz(0단계 출력; K, dist)
출력:
  homography.npz  (H, 그리고 검증 지표)
"""
import numpy as np, cv2, csv, sys

MARKERS = "markers.csv"
ODOM    = "odometry.csv"
CALIB   = "camera_calib.npz"
OUT     = "homography.npz"
MIN_RATIO = 0.05   # PCA 2축/1축 최소 비율(이하이면 직선으로 간주)

def load_markers(path):
    t, uv = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            t.append(float(row["t"])); uv.append([float(row["u"]), float(row["v"])])
    return np.array(t), np.array(uv, np.float32)

def load_odom(path):
    t, xy = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            t.append(float(row["t"])); xy.append([float(row["x"]), float(row["y"])])
    o = np.argsort(t)
    return np.array(t)[o], np.array(xy)[o]

def pca_ratio(pts):
    a = pts - pts.mean(0)
    e = np.linalg.eigvalsh(np.cov(a.T))
    return e[0] / e[1] if e[1] > 1e-12 else 0.0

# 1) 로드
mt, muv = load_markers(MARKERS)
ot, oxy = load_odom(ODOM)
cal = np.load(CALIB); K, dist = cal["K"], cal["dist"]

# 2) 시간 동기화: 마커 시각에 오도메트리 (x,y) 보간
lo, hi = max(mt.min(), ot.min()), min(mt.max(), ot.max())
m = (mt >= lo) & (mt <= hi)
mt, muv = mt[m], muv[m]
if len(mt) < 8:
    sys.exit("겹치는 구간의 대응점이 부족. 두 로그의 시계(타임스탬프) 정렬을 확인.")
X = np.interp(mt, ot, oxy[:, 0])
Y = np.interp(mt, ot, oxy[:, 1])
world = np.column_stack([X, Y]).astype(np.float32)

# 3) 픽셀 왜곡 제거 (정규→다시 픽셀좌표로: P=K 로 되돌림)
undist = cv2.undistortPoints(muv.reshape(-1, 1, 2), K, dist, P=K).reshape(-1, 2)

# 4) ★ 공선성(직선) 검사
rp, rw = pca_ratio(undist), pca_ratio(world)
print(f"픽셀 퍼짐비 {rp:.3f}, 월드 퍼짐비 {rw:.3f} (>{MIN_RATIO} 필요)")
if rp < MIN_RATIO or rw < MIN_RATIO:
    sys.exit("궤적이 직선에 가까워 H를 안정적으로 못 구함 → 'ㄹ자'로 재주행 필요.")

# 5) H 계산 (픽셀→월드)
H, mask = cv2.findHomography(undist, world, cv2.RANSAC, 5.0)
if H is None:
    sys.exit("findHomography 실패.")

# 6) 검증: 재투영 오차(미터)
pred = cv2.perspectiveTransform(undist.reshape(-1, 1, 2), H).reshape(-1, 2)
err = np.linalg.norm(pred - world, axis=1)
inl = mask.ravel().astype(bool)
print(f"\n대응점 {len(world)}개, 인라이어 {inl.sum()}개")
print(f"재투영 오차(미터)  평균 {err[inl].mean():.4f}  최대 {err[inl].max():.4f}")

np.savez(OUT, H=H, err_mean=err[inl].mean(), err_max=err[inl].max())
# C++(RPi4)가 읽을 수 있게 .yml 로도 저장
fs = cv2.FileStorage("homography.yml", cv2.FILE_STORAGE_WRITE)
fs.write("H", H); fs.release()
print("\n저장:", OUT, ", homography.yml")
print("H =\n", H)
