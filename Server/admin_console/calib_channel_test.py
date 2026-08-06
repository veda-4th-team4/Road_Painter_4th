#!/usr/bin/env python3
"""관리자 창 채널별 캘리브레이션 회귀 테스트 (프로토콜 v0.4).

    python3 calib_channel_test.py

서버·카메라 없이 돈다 (server_send 를 가짜로 갈아끼운다).

확인하는 것 — 전부 "틀려도 에러가 안 나고 좌표만 조용히 틀리는" 종류다:
 1) 채널을 바꾸면 실제로 그 채널 슬롯에 저장되는가
 2) CH2 를 캘리해도 CH1 의 K/H 가 살아있는가 (캐시가 하나면 두 채널이 섞인다)
 3) H_MATRIX payload 최상위에 ch 가 실리는가
    (안 실으면 서버가 전부 채널 1로 보고 마지막에 캘리한 하나만 남는다)
 4) 채널 범위 밖 입력을 거절하는가
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rp_core

sent = []
rp_core.server_send = lambda t, p, log_payload=None: sent.append((t, p)) or True
rp_core.broadcast = lambda line: None

import cctv
cctv.server_send = rp_core.server_send
cctv.broadcast = rp_core.broadcast

fails = 0
def check(cond, msg):
    global fails
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        fails += 1

H1 = [1, 0, 0, 0, 1, 0, 0, 0, 1]
H2 = [2, 0, 0, 0, 2, 0, 0, 0, 1]

print("[1] 기본 채널 + 채널 전환")
check(cctv.calib_channel() == 1, "기본 캘리 대상은 채널 1")
ok, why = cctv.set_calib_channel(2)
check(ok and cctv.calib_channel() == 2, "채널 2 로 전환")
check(any(t == "CMD" and p.get("cmd") == "SELECT_CHANNEL" and p.get("ch") == 2
          for t, p in sent), "SELECT_CHANNEL 이 카메라로 나간다")

print("[2] 범위 검증")
ok, why = cctv.set_calib_channel(0)
check(not ok, "채널 0 거절: " + str(why))
ok, why = cctv.set_calib_channel(99)
check(not ok, "채널 99 거절")
ok, why = cctv.set_calib_channel("abc")
check(not ok, "숫자가 아니면 거절")
check(cctv.calib_channel() == 2, "거절돼도 현재 채널은 안 바뀐다")

print("[3] 채널별 저장 + H_MATRIX.ch")
sent.clear()
cctv.set_calib_channel(1)
cctv.calib_cache_k(100.0, 100.0, 320.0, 240.0, [0.1, 0, 0, 0, 0])
cctv.calib_cache_h(H1)
hm = [p for t, p in sent if t == "H_MATRIX"]
check(len(hm) >= 1, "H 가 들어오면 H_MATRIX 전송")
check(hm[-1].get("ch") == 1, "payload 최상위에 ch=1")
check(hm[-1]["calib"]["K"][0][0] == 100.0, "CH1 의 K 가 실렸다")

sent.clear()
cctv.set_calib_channel(2)
cctv.calib_cache_k(200.0, 200.0, 320.0, 240.0, [0.2, 0, 0, 0, 0])
cctv.calib_cache_h(H2)
hm = [p for t, p in sent if t == "H_MATRIX"]
check(hm[-1].get("ch") == 2, "payload 최상위에 ch=2")
check(hm[-1]["calib"]["K"][0][0] == 200.0, "CH2 의 K 가 실렸다")
check(hm[-1]["calib"]["H_floor"][0][0] == 2.0, "CH2 의 H 가 실렸다")

print("[4] (중요) CH2 캘리가 CH1 을 덮지 않는가")
sent.clear()
cctv.push_calib_to_server(1)
hm = [p for t, p in sent if t == "H_MATRIX"]
check(len(hm) == 1 and hm[0]["ch"] == 1, "CH1 을 다시 보낼 수 있다")
check(hm[0]["calib"]["K"][0][0] == 100.0, "CH1 의 K 가 그대로 100 (200 으로 안 섞임)")
check(hm[0]["calib"]["H_floor"][0][0] == 1.0, "CH1 의 H 가 그대로")

print("[5] 상태 요약 (UI 표시용)")
st = cctv.calib_channel_status()
check(st["channel"] == 2, "현재 채널 보고")
check(st["count"] == rp_core.CAM_CHANNELS, "채널 수 보고")
done = {c["ch"]: c["has_h"] for c in st["channels"]}
check(done.get(1) and done.get(2), "CH1·CH2 는 완료로 표시")
check(not done.get(3), "CH3 는 미완료로 표시")

print()
print(("실패" if fails else "전부 통과") + f" ({fails} fail)")
sys.exit(1 if fails else 0)
