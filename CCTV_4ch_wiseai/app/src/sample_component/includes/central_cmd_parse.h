#ifndef CENTRAL_CMD_PARSE_H
#define CENTRAL_CMD_PARSE_H

/**
 * 중앙 서버가 TLS 링크로 보내는 CMD 한 줄을, 이 앱 내부에서 쓰는 납작한 명령
 * 문자열로 바꾼다.
 *
 * central_tls_sender.cc 안이 아니라 별도 파일인 이유는 **호스트에서 테스트하기
 * 위해서**다. central_tls_sender.cc 는 openssl 헤더를 물고 있어서 카메라
 * 크로스 컴파일 환경에서만 빌드된다 — 그 안에 파싱을 두면 서버가 실제로 붙어야만
 * 검증할 수 있다. 이 파일은 <string.h>/<stdio.h> 만 쓰므로 개발 PC 에서
 * g++ 한 줄로 컴파일해 가짜 JSON 을 먹여볼 수 있다.
 *
 * 소켓도 전역 상태도 만지지 않는 순수 함수다.
 */

/**
 * `line`(개행 없는 JSON 한 줄)을 파싱해 `out` 에 명령 문자열을 쓴다.
 *
 * 반환 1 = 우리가 아는 명령(out 채움), 0 = 모르는 줄(out 손대지 않음).
 *
 * 출력 형식 — 전부 공백 구분, request_id 는 가변 길이라 항상 맨 뒤:
 *
 *   CALIB_START
 *   SELECT_CHANNEL <ch>
 *   SELECT_CHANNEL_BAD
 *   CALIB_CAPTURE <ch> <point_index> <x_mm> <y_mm> <request_id>
 *   CALIB_CAPTURE_BAD
 *   CALIB_DONE <ch> <m_mm> <n_mm> <request_id>
 *   CALIB_DONE_BAD
 *   CALIB_CANCEL <ch> <request_id>
 *   CALIB_CANCEL_BAD
 *
 * `ch` 는 **와이어 그대로 1-based** 로 넘긴다(서버 규약: 카메라 웹UI 의 CH1~CH4 와
 * 같은 번호). 내부 0-based 로의 변환은 이걸 받는 핸들러가 한다 — 여기서 미리
 * 빼면 "이 숫자가 어느 규약인지"가 두 곳으로 흩어진다.
 *
 * 값이 하나라도 없거나 범위를 벗어나면 `_BAD` 를 낸다. 조용히 0 이나 -1 로
 * 채우지 않는 이유는 그 값이 그대로 캘리브레이션 데이터가 되기 때문이다 —
 * 기존 SELECT_CHANNEL_BAD 과 같은 규칙.
 *
 * 단 `CALIB_CANCEL_BAD` 만은 성격이 다르다. 다른 `_BAD` 는 "이 줄은 버린다"는
 * 뜻이지만, 취소는 **버리면 안 되는 명령**이다 — 서버는 CALIB_STOPPED 를
 * calib_cancel_ack_ms(5초) 동안 기다리고, 그 동안 로봇은 이미 서 있다.
 * 그래서 ch 를 못 읽어도 명령 자체는 올려보내고, 어느 렌즈인지 모른다는
 * 판단은 받는 쪽이 한다(그쪽은 "열려 있는 세션 전부"를 접을 수 있다).
 */
int central_cmd_parse(const char* line, char* out, int out_len);

#endif  // CENTRAL_CMD_PARSE_H
