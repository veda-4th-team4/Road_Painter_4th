#!/usr/bin/env python3
"""카메라에게 채널별 RTSP 주소를 직접 물어본다 (ONVIF).

왜 이 스크립트가 필요한가:
    PNM-C16083RVQ 같은 멀티디렉셔널 카메라는 채널마다 RTSP 경로가 다르고, 그
    경로는 펌웨어와 프로파일 구성에 따라 달라진다. 짐작해서 넣으면 안 된다 —
    틀린 URL 로 반복 접속하면 Hanwha 카메라가 계정을 잠근다
    (RTSP/1.0 490 Account Blocked). 그래서 카메라가 스스로 알려주는 값을 받는다.

사용법:
    python3 probe_onvif.py --host 192.168.0.20 --user admin --password '...'
    python3 probe_onvif.py --host ... --user ... --password ... --env

    --env 를 붙이면 cameras.env 에 그대로 붙여넣을 수 있는 형태로 출력한다.

의존성: requests (표준 설치에 이미 있음). onvif-zeep 같은 추가 패키지는 안 쓴다.
"""

import argparse
import base64
import hashlib
import os
import re
import secrets
import sys
from datetime import datetime, timedelta, timezone
from urllib.parse import urlparse, urlunparse, quote

import requests
import urllib3

SOAP_NS = "http://www.w3.org/2003/05/soap-envelope"
MEDIA10_NS = "http://www.onvif.org/ver10/media/wsdl"
DEVICE_NS = "http://www.onvif.org/ver10/device/wsdl"

# 카메라는 자체서명 인증서를 쓴다 (브라우저에서도 "주의 요함" 이 뜨는 그것).
# 검증을 켜두면 SSLCertVerificationError 로 아무것도 못 한다. 폐쇄망의 장비를
# 직접 지정해서 부르는 용도라 검증을 끈다 — 경고도 같이 끈다.
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)


def ws_security_header(user, password, clock_offset):
    """ONVIF WS-Security UsernameToken(Digest) 헤더를 만든다.

    ⚠️ Created 타임스탬프는 **카메라 시계** 기준이어야 한다. 카메라와 PC 의
       시각이 몇 분만 어긋나도 카메라가 토큰을 거부한다 (NotAuthorized). 그래서
       GetSystemDateAndTime 으로 미리 잰 오차(clock_offset)를 반영한다.
    """
    nonce = secrets.token_bytes(16)
    created = (datetime.now(timezone.utc) + clock_offset).strftime("%Y-%m-%dT%H:%M:%SZ")
    digest = base64.b64encode(
        hashlib.sha1(nonce + created.encode() + password.encode()).digest()
    ).decode()
    return f"""<s:Header>
    <Security s:mustUnderstand="1"
      xmlns="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd">
      <UsernameToken>
        <Username>{user}</Username>
        <Password Type="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-username-token-profile-1.0#PasswordDigest">{digest}</Password>
        <Nonce EncodingType="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-soap-message-security-1.0#Base64Binary">{base64.b64encode(nonce).decode()}</Nonce>
        <Created xmlns="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd">{created}</Created>
      </UsernameToken>
    </Security>
  </s:Header>"""


def soap_call(url, body, user, password, clock_offset, timeout=8):
    """SOAP 요청 한 번. WS-Security 로 먼저 시도하고, 거부되면 HTTP digest 로 재시도.

    카메라 펌웨어에 따라 둘 중 하나만 받는 경우가 있어서 양쪽을 다 시도한다.

    ⚠️ 리다이렉트를 requests 에 맡기면 안 된다. 카메라가 http 를 https 로 301 로
       넘기는데, requests 는 301 을 따라갈 때 POST 를 GET 으로 바꾼다 (브라우저와
       같은 동작) → SOAP 본문이 통째로 사라진다. 그래서 따라가지 않게 막고
       Location 을 직접 읽어 같은 방식(POST)으로 다시 부른다.
    """
    header = ws_security_header(user, password, clock_offset) if user else ""
    envelope = (
        f'<?xml version="1.0" encoding="UTF-8"?>'
        f'<s:Envelope xmlns:s="{SOAP_NS}">{header}<s:Body>{body}</s:Body></s:Envelope>'
    )
    headers = {"Content-Type": "application/soap+xml; charset=utf-8"}

    def post(target, **kw):
        return requests.post(
            target, data=envelope.encode(), headers=headers, timeout=timeout,
            verify=False, allow_redirects=False, **kw
        )

    resp = post(url)
    for _ in range(3):                      # http -> https 같은 이동은 보통 1번이다
        if resp.status_code not in (301, 302, 303, 307, 308):
            break
        loc = resp.headers.get("Location")
        if not loc:
            break
        url = loc if "://" in loc else url.rsplit("/", 3)[0] + loc
        resp = post(url)

    if resp.status_code == 200:
        return resp.text
    if resp.status_code in (400, 401) and user:
        # WS-Security 를 안 받는 펌웨어 → HTTP digest 로 다시.
        plain = (
            f'<?xml version="1.0" encoding="UTF-8"?>'
            f'<s:Envelope xmlns:s="{SOAP_NS}"><s:Body>{body}</s:Body></s:Envelope>'
        )
        resp = requests.post(
            url,
            data=plain.encode(),
            headers=headers,
            timeout=timeout,
            verify=False,
            auth=requests.auth.HTTPDigestAuth(user, password),
        )
        if resp.status_code == 200:
            return resp.text
    raise RuntimeError(
        f"ONVIF 요청 실패 ({resp.status_code}) — 계정/비밀번호와 카메라의 ONVIF 사용 설정을 확인하세요.\n"
        f"{resp.text[:400]}"
    )


def tag_values(xml, local_name):
    """네임스페이스 접두어를 무시하고 <...:local_name>값</...> 을 전부 뽑는다."""
    return re.findall(
        rf"<(?:\w+:)?{local_name}\b[^>]*>(.*?)</(?:\w+:)?{local_name}>", xml, re.S
    )


def get_clock_offset(host, port, timeout=8):
    """카메라 시계와 이 PC 시계의 차이를 잰다 (WS-Security 인증용, 무인증 호출)."""
    url = f"http://{host}:{port}/onvif/device_service"
    try:
        xml = soap_call(
            url, f'<GetSystemDateAndTime xmlns="{DEVICE_NS}"/>', None, None, timedelta(0), timeout
        )
    except Exception:
        return timedelta(0)

    utc = re.search(r"<(?:\w+:)?UTCDateTime>(.*?)</(?:\w+:)?UTCDateTime>", xml, re.S)
    if not utc:
        return timedelta(0)
    blob = utc.group(1)

    def one(name):
        m = re.search(rf"<(?:\w+:)?{name}>(\d+)</(?:\w+:)?{name}>", blob)
        return int(m.group(1)) if m else None

    parts = {n: one(n) for n in ("Year", "Month", "Day", "Hour", "Minute", "Second")}
    if any(v is None for v in parts.values()):
        return timedelta(0)
    cam = datetime(
        parts["Year"], parts["Month"], parts["Day"],
        parts["Hour"], parts["Minute"], parts["Second"], tzinfo=timezone.utc,
    )
    offset = cam - datetime.now(timezone.utc)
    if abs(offset.total_seconds()) > 5:
        print(f"[i] 카메라 시계가 이 PC 보다 {offset.total_seconds():+.0f}초 어긋나 있습니다 — 보정합니다.",
              file=sys.stderr)
    return offset


def find_media_service(host, port, user, password, clock_offset):
    """Media 서비스 주소를 찾는다. 못 찾으면 device_service 주소를 그대로 쓴다."""
    device_url = f"http://{host}:{port}/onvif/device_service"
    try:
        xml = soap_call(
            device_url, f'<GetServices xmlns="{DEVICE_NS}"><IncludeCapability>false</IncludeCapability></GetServices>',
            user, password, clock_offset,
        )
    except Exception:
        return device_url

    for block in re.findall(r"<(?:\w+:)?Service>(.*?)</(?:\w+:)?Service>", xml, re.S):
        ns = tag_values(block, "Namespace")
        addr = tag_values(block, "XAddr")
        if ns and addr and ns[0].strip() == MEDIA10_NS:
            return addr[0].strip()
    return device_url


def get_profiles(media_url, user, password, clock_offset):
    """프로파일 토큰 + 이름 + 해상도/코덱을 뽑는다."""
    xml = soap_call(media_url, f'<GetProfiles xmlns="{MEDIA10_NS}"/>', user, password, clock_offset)

    profiles = []
    for block in re.findall(r"<(?:\w+:)?Profiles\b(.*?)</(?:\w+:)?Profiles>", xml, re.S):
        token = re.search(r'token="([^"]+)"', block)
        name = tag_values(block, "Name")
        enc = tag_values(block, "Encoding")
        w = tag_values(block, "Width")
        h = tag_values(block, "Height")
        fps = tag_values(block, "FrameRateLimit")
        profiles.append({
            "token": token.group(1) if token else "?",
            "name": name[0].strip() if name else "?",
            "encoding": enc[0].strip() if enc else "?",
            "resolution": f"{w[0]}x{h[0]}" if w and h else "?",
            "fps": fps[0].strip() if fps else "?",
        })
    return profiles


def get_stream_uri(media_url, token, user, password, clock_offset):
    body = (
        f'<GetStreamUri xmlns="{MEDIA10_NS}">'
        f"<StreamSetup>"
        f'<Stream xmlns="http://www.onvif.org/ver10/schema">RTP-Unicast</Stream>'
        f'<Transport xmlns="http://www.onvif.org/ver10/schema"><Protocol>RTSP</Protocol></Transport>'
        f"</StreamSetup>"
        f"<ProfileToken>{token}</ProfileToken>"
        f"</GetStreamUri>"
    )
    xml = soap_call(media_url, body, user, password, clock_offset)
    uris = tag_values(xml, "Uri")
    return uris[0].strip() if uris else None


def with_credentials(uri, user, password):
    """카메라가 준 URL 에 계정을 끼워넣는다. 특수문자는 URL 인코딩한다."""
    p = urlparse(uri)
    host = p.hostname or ""
    if p.port:
        host = f"{host}:{p.port}"
    cred = f"{quote(user, safe='')}:{quote(password, safe='')}@"
    return urlunparse((p.scheme, cred + host, p.path, p.params, p.query, p.fragment))


def pick_main_per_channel(rows):
    """채널별로 '메인스트림' 하나씩 고른다.

    멀티디렉셔널 카메라는 URL 경로에 채널 번호가 들어간다:
        rtsp://<ip>:554/<채널 0-3>/onvif/profile<N>/media.smp
    그리고 채널마다 프로파일이 여러 개다 (MJPEG / H.264 / 모바일). 그냥 앞에서
    4개를 집으면 한 채널의 프로파일 3개 + 옆 채널 1개가 잡혀서 완전히 틀린다.
    그래서 채널로 묶은 뒤, 채널 안에서 화질이 가장 좋은 것을 고른다.

    고르는 기준 (순서대로):
      1. JPEG 제외 — MJPEG 은 스냅샷용이고 대역을 훨씬 많이 먹는다
      2. 해상도(픽셀 수)가 큰 것
      3. fps 가 높은 것
    """
    by_channel = {}
    for p, uri in rows:
        if not uri:
            continue
        m = re.search(r"://[^/]+/(\d+)/", uri)
        ch = int(m.group(1)) if m else -1
        by_channel.setdefault(ch, []).append((p, uri))

    def score(item):
        p, _ = item
        w, _, h = p["resolution"].partition("x")
        try:
            pixels = int(w) * int(h)
        except ValueError:
            pixels = 0
        try:
            fps = float(p["fps"])
        except ValueError:
            fps = 0.0
        is_video = 0 if p["encoding"].upper().startswith("JPEG") else 1
        return (is_video, pixels, fps)

    return [(ch, *max(by_channel[ch], key=score)) for ch in sorted(by_channel)]


def print_env(rows, user, password):
    picked = pick_main_per_channel(rows)

    print("# probe_onvif.py 출력 — cameras.env 에 붙여넣으세요.")
    print("# 채널 번호는 URL 경로의 0-based 인덱스이며, 카메라 웹 UI 의 CH 1~4 에 대응합니다.")
    for i, (ch, p, _uri) in enumerate(picked, start=1):
        print(f"#   ch{i}  <- 카메라 채널 {ch} (웹UI CH{ch + 1}) : "
              f"{p['name']} {p['resolution']} {p['encoding']} {p['fps']}fps")
    print()
    for i, (_ch, _p, uri) in enumerate(picked, start=1):
        print(f"MTX_PATHS_CH{i}_SOURCE={with_credentials(uri, user, password)}")

    if len(picked) != 4:
        print(f"\n# ⚠️ 채널이 4개가 아니라 {len(picked)}개로 잡혔습니다. "
              f"mediamtx.yml 의 paths 항목 수와 맞는지 확인하세요.", file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(description="ONVIF 로 채널별 RTSP 주소를 조회한다")
    ap.add_argument("--host", required=True, help="카메라 IP")
    ap.add_argument("--port", type=int, default=80, help="카메라 HTTP 포트 (기본 80)")
    ap.add_argument("--user", default=os.environ.get("CAM_USER", "admin"))
    ap.add_argument("--password", default=os.environ.get("CAM_PASS"))
    ap.add_argument("--env", action="store_true",
                    help="cameras.env 에 붙여넣을 형태로 출력")
    args = ap.parse_args()

    if not args.password:
        ap.error("--password 를 주거나 CAM_PASS 환경변수를 설정하세요.")

    offset = get_clock_offset(args.host, args.port)
    media_url = find_media_service(args.host, args.port, args.user, args.password, offset)
    print(f"[i] Media 서비스: {media_url}", file=sys.stderr)

    profiles = get_profiles(media_url, args.user, args.password, offset)
    if not profiles:
        print("프로파일을 하나도 못 받았습니다. 카메라 웹 UI 에서 ONVIF 가 켜져 있는지 확인하세요.",
              file=sys.stderr)
        return 1

    rows = []
    for p in profiles:
        uri = get_stream_uri(media_url, p["token"], args.user, args.password, offset)
        rows.append((p, uri))

    if args.env:
        print_env(rows, args.user, args.password)
    else:
        print(f"\n프로파일 {len(rows)}개\n")
        for p, uri in rows:
            print(f"  이름   : {p['name']}")
            print(f"  토큰   : {p['token']}")
            print(f"  스트림 : {p['resolution']} {p['encoding']} {p['fps']}fps")
            print(f"  RTSP   : {uri}")
            print()
        print("cameras.env 형태로 뽑으려면 --env 를 붙이세요.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
