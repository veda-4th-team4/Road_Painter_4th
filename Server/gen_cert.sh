#!/bin/bash
# 자체서명 TLS 인증서 생성 (서버 RPi에서 1회 실행)
# 사용법: ./gen_cert.sh [서버IP]   예: ./gen_cert.sh 192.168.0.10
set -e
SERVER_IP=${1:-127.0.0.1}
mkdir -p certs
openssl req -x509 -newkey rsa:2048 -nodes -days 365 \
  -keyout certs/server.key -out certs/server.crt \
  -subj "/CN=road-painter-server" \
  -addext "subjectAltName=IP:127.0.0.1,IP:${SERVER_IP},DNS:localhost"
chmod 600 certs/server.key
echo "생성 완료: certs/server.crt, certs/server.key"
echo "-> server.crt 파일을 QT/로봇/CCTV 클라이언트에 복사해서 신뢰 CA로 사용"
