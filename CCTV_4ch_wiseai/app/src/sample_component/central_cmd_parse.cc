#include "central_cmd_parse.h"

#include <stdio.h>
#include <string.h>

namespace {

// "key": 를 찾아 값의 첫 글자를 가리키는 포인터. 없으면 NULL.
//
// 키를 따옴표까지 포함해 찾는 이유: "ch" 를 그냥 찾으면 "search" 같은 다른 키의
// 일부에도 걸린다. 서버 payload 가 늘어날수록 그런 오검출이 생긴다.
//
// 콜론 뒤 공백은 건너뛴다. {"ch":2} 로 보낼지 {"ch": 2} 로 보낼지는 직렬화
// 구현에 달렸고 둘 다 유효한 JSON 이다 — 여기서 받아들이지 않으면 서버가
// 직렬화기를 바꾸는 날 명령이 통째로 조용히 사라진다.
const char* after_key(const char* line, const char* key) {
  char pat[48];
  if (snprintf(pat, sizeof(pat), "\"%s\":", key) >= (int)sizeof(pat)) return NULL;
  const char* q = strstr(line, pat);
  if (!q) return NULL;
  q += strlen(pat);
  while (*q == ' ' || *q == '\t') ++q;
  return q;
}

// "key":<숫자> 에서 정수 하나. 못 찾거나 숫자가 아니면 false.
bool json_int(const char* line, const char* key, int* out) {
  const char* q = after_key(line, key);
  return q && sscanf(q, "%d", out) == 1;
}

// "key":<실수>
bool json_num(const char* line, const char* key, double* out) {
  const char* q = after_key(line, key);
  return q && sscanf(q, "%lf", out) == 1;
}

// "key":[<실수>,<실수>] — world_xy_mm 용. 배열 안 공백도 허용한다.
bool json_pair(const char* line, const char* key, double* a, double* b) {
  const char* q = after_key(line, key);
  return q && sscanf(q, " [ %lf , %lf", a, b) == 2;
}

// "key":"<문자열>" 에서 값만. 넘치면 자른다(잘라도 로그·대조용이라 무해).
//
// 숫자와 달리 sscanf("%s") 로는 못 뽑는다 — 따옴표가 그대로 붙어 오고, 값에
// 공백이 있으면 거기서 끊긴다. request_id 는 출력 문자열의 맨 뒤에 놓이므로
// 공백이 섞이면 뒤쪽 파싱이 흔들린다. 그래서 여기서 공백을 '_' 로 바꿔 둔다.
bool json_str(const char* line, const char* key, char* out, int out_len) {
  if (out_len < 1) return false;
  out[0] = '\0';
  const char* q = after_key(line, key);
  if (!q || *q != '"') return false;
  ++q;
  int i = 0;
  for (; q[i] && q[i] != '"' && i < out_len - 1; ++i)
    out[i] = (q[i] == ' ' || q[i] == '\t') ? '_' : q[i];
  out[i] = '\0';
  return i > 0;
}

// 와이어의 채널 번호는 1-based. 서버 프로토콜이 8채널까지 열어두고 있어
// 상한도 거기 맞춘다(이 앱이 실제로 쓰는 건 1..4).
bool valid_wire_ch(int ch) { return ch >= 1 && ch <= 8; }

// 이 줄이 `kind` 명령인가 — 서버는 **두 가지 봉투**를 섞어 쓴다:
//
//   CALIB_START / SELECT_CHANNEL / CALIB_CANCEL
//     -> {"type":"CMD","payload":{"cmd":"CALIB_START",...}}
//   CALIB_CAPTURE / CALIB_DONE
//     -> {"type":"CALIB_CAPTURE","payload":{...}}      (독립 메시지 타입)
//
// wire 스펙이 그렇게 정해져 있다 — §1 의 세션 개시만 CMD 이고, §2 의 5종은
// 자체 type 이다. 2026-08-12 에 여기서 "type":"CMD" 를 먼저 요구했다가
// CALIB_CAPTURE/CALIB_DONE 이 통째로 버려졌다: 취소만 되고 캡처는 침묵해
// 서버가 15초 뒤 capture_timeout 으로 세션을 접었다(서버 로그 18:56:53 ->
// 18:57:08). 봉투를 가리지 않고 명령 이름으로만 판별한다.
//
// 값을 닫는 따옴표까지 맞춰 보는 이유: 접두사만 보면 CALIB_CAPTURE 가 카메라가
// 되돌려 보내는 CALIB_CAPTURE_OK/_FAIL 에도 걸린다. 지금은 그 메시지가 들어올
// 일이 없지만, 걸릴 수 있는 패턴을 남겨 둘 이유도 없다.
bool key_is(const char* line, const char* key, const char* want) {
  const char* q = after_key(line, key);
  if (!q || *q != '"') return false;
  ++q;
  const size_t n = strlen(want);
  return strncmp(q, want, n) == 0 && q[n] == '"';
}

bool is_kind(const char* line, const char* kind) {
  return key_is(line, "cmd", kind) || key_is(line, "type", kind);
}

}  // namespace

int central_cmd_parse(const char* line, char* out, int out_len) {
  if (!line || !out || out_len < 2) return 0;
  // 봉투(type)로 미리 거르지 않는다 — is_kind() 주석 참고. 어느 것도 아니면
  // 맨 끝에서 0 을 돌려주므로 무관한 메시지(ACK 등)는 그대로 무시된다.

  if (is_kind(line, "CALIB_START")) {
    snprintf(out, (size_t)out_len, "CALIB_START");
    return 1;
  }

  if (is_kind(line, "SELECT_CHANNEL")) {
    int ch = 0;
    if (json_int(line, "ch", &ch) && valid_wire_ch(ch))
      snprintf(out, (size_t)out_len, "SELECT_CHANNEL %d", ch);
    else
      snprintf(out, (size_t)out_len, "SELECT_CHANNEL_BAD");
    return 1;
  }

  if (is_kind(line, "CALIB_CAPTURE")) {
    int ch = 0, idx = -1;
    double x = 0.0, y = 0.0;
    char rid[64];
    // point_index 는 0 이 유효한 값이라(출발점) "못 찾음"과 구분해야 한다.
    // json_int 의 반환값으로 구분하고 idx 자체는 음수만 거른다.
    const bool ok = json_int(line, "ch", &ch) && valid_wire_ch(ch)
                 && json_int(line, "point_index", &idx) && idx >= 0
                 && json_pair(line, "world_xy_mm", &x, &y);
    if (!json_str(line, "request_id", rid, sizeof(rid))) rid[0] = '\0';
    if (ok)
      snprintf(out, (size_t)out_len, "CALIB_CAPTURE %d %d %.3f %.3f %s",
               ch, idx, x, y, rid);
    else
      snprintf(out, (size_t)out_len, "CALIB_CAPTURE_BAD");
    return 1;
  }

  if (is_kind(line, "CALIB_DONE")) {
    int ch = 0;
    double m = 0.0, n = 0.0;
    char rid[64];
    // m_mm/n_mm 은 이번 주행 사각형의 크기다(번들의 canvas_mm 을 채우는 데 쓴다).
    // 없으면 받은 지점들의 바운딩 박스로 역산할 수도 있지만, 서버가 명시해 주기로
    // 했으므로 빠지면 BAD 로 둔다 — 조용히 추정하면 그 추정이 캘리에 박힌다.
    const bool ok = json_int(line, "ch", &ch) && valid_wire_ch(ch)
                 && json_num(line, "m_mm", &m) && m > 0.0
                 && json_num(line, "n_mm", &n) && n > 0.0;
    if (!json_str(line, "request_id", rid, sizeof(rid))) rid[0] = '\0';
    if (ok)
      snprintf(out, (size_t)out_len, "CALIB_DONE %d %.3f %.3f %s", ch, m, n, rid);
    else
      snprintf(out, (size_t)out_len, "CALIB_DONE_BAD");
    return 1;
  }

  if (is_kind(line, "CALIB_CANCEL")) {
    int ch = 0;
    char rid[64];
    if (!json_str(line, "request_id", rid, sizeof(rid))) rid[0] = '\0';
    // request_id 가 없어도 취소는 취소다. 다른 명령과 달리 여기서 되돌려
    // 보낼 데이터가 없으므로(ack 한 줄뿐) 없는 채로 진행한다.
    if (json_int(line, "ch", &ch) && valid_wire_ch(ch))
      snprintf(out, (size_t)out_len, "CALIB_CANCEL %d %s", ch, rid);
    else
      snprintf(out, (size_t)out_len, "CALIB_CANCEL_BAD");
    return 1;
  }

  return 0;
}
