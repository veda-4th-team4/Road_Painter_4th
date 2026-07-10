"""
1단계-C: 바닥영역 확정(A ∩ B) + 거리별 오차맵.

A(주행 궤적 기반): 마커 궤적 픽셀에 convexHull → 실제로 로봇이 지난 '검증된' 영역.
B(기하 기반):      Canny → HoughLinesP → 소실점 + 바닥-벽 경계 → 사다리꼴 근사.
최종:              A ∩ B (교집합) 마스크.
오차맵:            H로 픽셀 1개가 지면 몇 mm에 해당하는지(=측위 분해능) 히트맵.
                   카메라에서 멀수록(화면 안쪽) 픽셀당 실거리가 커져 정밀도 저하.

입력: floor_bg.png, markers.csv, homography.npz
출력: floor_mask.png, floor_overlay.png, error_map.png
"""
import numpy as np, cv2, csv

BG = "floor_bg.png"; MARKERS = "markers.csv"; H_NPZ = "homography.npz"

img = cv2.imread(BG)
if img is None:
    raise SystemExit("floor_bg.png 없음 (record_markers.py 먼저 실행)")
h, w = img.shape[:2]
H = np.load(H_NPZ)["H"]

# ── A: 궤적 convexHull ──
pts = []
with open(MARKERS) as f:
    for r in csv.DictReader(f):
        pts.append([float(r["u"]), float(r["v"])])
pts = np.array(pts, np.float32)
maskA = np.zeros((h, w), np.uint8)
if len(pts) >= 3:
    hull = cv2.convexHull(pts.astype(np.int32))
    cv2.fillConvexPoly(maskA, hull, 255)

# ── B: 엣지/직선 → 사다리꼴 근사 ──
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray, 50, 150)
lines = cv2.HoughLinesP(edges, 1, np.pi/180, 80, minLineLength=w//6, maxLineGap=30)
maskB = np.zeros((h, w), np.uint8)
if lines is not None:
    # 가로에 가까운 강한 엣지(바닥-벽 경계 후보)의 최상단 y를 지평선으로 사용
    ys = [min(y1, y2) for x1, y1, x2, y2 in lines[:, 0]
          if abs(y2 - y1) < abs(x2 - x1) * 0.3]   # 기울기 완만 = 가로선
    horizon = int(np.percentile(ys, 20)) if ys else int(h*0.25)
    # 지평선 아래 전체를 바닥 후보 사다리꼴로 근사(원근상 아래로 갈수록 넓음)
    poly = np.array([[0, h], [w, h],
                     [int(w*0.85), horizon], [int(w*0.15), horizon]], np.int32)
    cv2.fillConvexPoly(maskB, poly, 255)
else:
    maskB[int(h*0.25):, :] = 255

# ── A ∩ B ──
floor = cv2.bitwise_and(maskA, maskB)
floor = cv2.morphologyEx(floor, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
cv2.imwrite("floor_mask.png", floor)

overlay = img.copy()
overlay[floor > 0] = (0.5*overlay[floor > 0] + np.array([0, 128, 0])).astype(np.uint8)
cv2.imwrite("floor_overlay.png", overlay)

# ── 거리별 오차맵: 픽셀당 지면 실거리(mm/px) ──
# 각 픽셀에서 오른쪽/아래 1px 이동이 지면상 몇 미터인지 → 값 클수록 저정밀
step = 20
gx, gy = np.meshgrid(np.arange(0, w, step), np.arange(0, h, step))
base = np.stack([gx.ravel(), gy.ravel()], 1).astype(np.float32)
dx = base + [1, 0]; dy = base + [0, 1]
def proj(p): return cv2.perspectiveTransform(p.reshape(-1,1,2), H).reshape(-1,2)
wb, wdx, wdy = proj(base), proj(dx), proj(dy)
mm_per_px = (np.linalg.norm(wdx-wb,axis=1) + np.linalg.norm(wdy-wb,axis=1))/2 * 1000
res = mm_per_px.reshape(gy.shape)

heat = np.clip(res, 0, np.percentile(res, 95))
heat = (heat/heat.max()*255).astype(np.uint8)
heat = cv2.applyColorMap(cv2.resize(heat,(w,h)), cv2.COLORMAP_JET)
heat[floor == 0] = img[floor == 0]//2   # 바닥영역만 강조
cv2.imwrite("error_map.png", heat)

print("저장: floor_mask.png, floor_overlay.png, error_map.png")
print(f"바닥영역 내 픽셀 분해능  최소 {res[res>0].min():.1f} ~ 최대 {res.max():.1f} mm/px")
print("→ error_map: 파랑=정밀, 빨강=저정밀(멀리). 빨간 구역은 유효 작업범위에서 제외 고려.")
