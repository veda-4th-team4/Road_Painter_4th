#!/usr/bin/env python3
"""테스트용 사용자 계정을 config/users.json에 넣는다 (개발/데모 전용).

config/users.json은 비밀번호 해시가 들어있어 .gitignore 대상이라 커밋되지 않는다.
그래서 "테스트 계정 하나 만들어두기"를 이 스크립트로 재현 가능하게 만들어 둔다.

비밀번호 해시는 서버(src/user_store.cpp hashPw)와 동일하게 만든다:
  PBKDF2-HMAC-SHA256(pw, salt=<salt의 hex 문자열 그대로>, iter=10000, dkLen=32)
  ※ salt로 쓰는 값은 16바이트 원본이 아니라 그것을 hex로 찍은 32글자 ASCII다
    (서버가 saltHex.data()를 그대로 salt로 넘기므로 여기서도 똑같이 맞춰야 한다).

호모그래피 단위(중요) — 2026-08-11 규격:
  CCTV가 보내는 H는 pixel -> world "mm"이고, 서버는 그 번들을 mm 그대로 저장·중계한다.
  ÷1000 은 서버 내부 좌표 계산용 사본에서만 일어난다(calib.hpp calibFromJson).
  단위 판단은 번들의 "unit" 필드 한 곳만 본다(bundleMeterScale).
  따라서 이 스크립트는 mm 행렬을 그대로 넣고 unit:"mm" 을 함께 적는다.
  ※ 예전 이 스크립트가 만든 파일(미터, unit 없음)도 그대로 읽힌다 - 서버가 로그인
    때 bundleToMm 으로 mm 환산해 Qt에 내보낸다(파일 자체는 안 고친다).

사용:
  python3 tools/seed_user.py                 # 기본 test/1234 + 기본 H + 기본 카메라 IP
  python3 tools/seed_user.py --id u2 --pw pw --no-calib
  python3 tools/seed_user.py --cam-ip 192.168.0.31   # 카메라 IP만 바꿔서
  python3 tools/seed_user.py --h-json '[[...],[...],[...]]'
  (Server/ 디렉토리에서 실행. 이미 있는 id면 --force 없이는 덮어쓰지 않는다)
"""
import argparse
import hashlib
import json
import os
import secrets
import sys

PBKDF2_ITERS = 10000   # src/user_store.cpp kPbkdf2Iters
HASH_LEN = 32          # src/user_store.cpp kHashLen (SHA-256)

# 현장 CCTV 카메라 IP. Qt가 이 값으로 RTSP URL을 조립한다(서버는 검증 없이 저장만).
DEFAULT_CAM_IP = "192.168.0.9"

# 기본 더미 호모그래피 (pixel -> world mm). 실측 CCTV 캘리 결과 예시.
DEFAULT_H_MM = [
    [-9.27907264, 23.17884185, -8981.30666756],
    [12.12652417, 14.94214153, -23044.52588633],
    [-0.00050888, -0.01767461, 1.0],
]


def hash_pw(pw, salt_hex):
    """서버 user_store.cpp의 hashPw와 동일한 해시."""
    return hashlib.pbkdf2_hmac(
        "sha256", pw.encode(), salt_hex.encode(), PBKDF2_ITERS, HASH_LEN
    ).hex()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", default="config/users.json")
    ap.add_argument("--id", default="test")
    ap.add_argument("--pw", default="1234")
    ap.add_argument("--cam-ip", default=DEFAULT_CAM_IP,
                    help=f"등록할 카메라 IP (기본 {DEFAULT_CAM_IP}, 빈 문자열이면 null)")
    ap.add_argument("--h-json", default=None,
                    help="pixel->world mm 호모그래피 3x3 JSON (기본: 내장 예시)")
    ap.add_argument("--no-calib", action="store_true", help="캘리브레이션 없이 계정만")
    ap.add_argument("--force", action="store_true", help="이미 있는 id도 덮어쓰기")
    args = ap.parse_args()

    users = {}
    if os.path.exists(args.file):
        with open(args.file, encoding="utf-8") as f:
            try:
                users = json.load(f)
            except ValueError:
                print(f"[!] {args.file} 파싱 실패 - 덮어쓰면 기존 계정이 날아갑니다. 중단.")
                return 1
        if not isinstance(users, dict):
            print(f"[!] {args.file} 형식이 예상과 다릅니다(오브젝트 아님). 중단.")
            return 1

    if args.id in users and not args.force:
        print(f"[!] '{args.id}' 계정이 이미 있습니다. 덮어쓰려면 --force")
        return 1

    calib = None
    if not args.no_calib:
        h_mm = json.loads(args.h_json) if args.h_json else DEFAULT_H_MM
        if (not isinstance(h_mm, list) or len(h_mm) != 3
                or any(not isinstance(r, list) or len(r) != 3 for r in h_mm)):
            print("[!] --h-json은 3x3 배열이어야 합니다.")
            return 1
        # K/D 없음 = 왜곡 보정 생략, H_marker=H_floor = 시차 보정 생략.
        # (현재 admin_console 캘리 도구가 올리는 번들과 같은 수준)
        # unit:"mm" 을 명시해 mm 행렬을 그대로 넣는다 - 서버가 이 필드만 보고
        # 단위를 판단하고, ÷1000 은 내부 계산용 사본에서만 한다.
        calib = {"version": 1, "unit": "mm", "H_floor": h_mm, "H_marker": h_mm,
                 "marker_height_mm": 0}

    salt_hex = secrets.token_hex(16)
    users[args.id] = {
        "salt": salt_hex,
        "hash": hash_pw(args.pw, salt_hex),
        "calib": calib,
        # 서버 registerUser와 같은 규약: 빈 문자열은 "등록 안 함"을 뜻하는 null
        "cam_ip": args.cam_ip or None,
    }

    os.makedirs(os.path.dirname(args.file) or ".", exist_ok=True)
    with open(args.file, "w", encoding="utf-8") as f:
        json.dump(users, f, indent=2, ensure_ascii=False)
        f.write("\n")

    print(f"[+] '{args.id}' 계정 등록 완료 -> {args.file}")
    print(f"    비밀번호: {args.pw}")
    print(f"    카메라 IP: {args.cam_ip or '(없음 - null)'}")
    if calib:
        print("    캘리브레이션: 있음 (mm 그대로 저장, unit:\"mm\")")
        print(f"      H_floor[0] = {calib['H_floor'][0]}")
    else:
        print("    캘리브레이션: 없음 (로그인 시 calib=null)")
    print("    ※ 서버가 실행 중이면 재시작해야 파일 변경이 반영됩니다"
          " (UserStore는 기동 시 1회 로드).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
