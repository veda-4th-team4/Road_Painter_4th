#include "central_tls_sender.h"
#include "app_config.h"        // CENTRAL_TLS_MAX_LINE, POSE_RECONNECT_MS, CENTRAL_TLS_HANDSHAKE_MS
#include "central_cmd_parse.h" // 서버 CMD 한 줄 -> 내부 명령 문자열 (openssl 무관, 호스트 테스트 가능)

#include <openssl/ssl.h>
#include <openssl/x509_vfy.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace {

enum State { OFFLINE, TCP_PENDING, TLS_PENDING, ONLINE };

State state = OFFLINE;
int fd = -1;
SSL_CTX* ctx = NULL;
SSL* ssl = NULL;
char read_buf[1024];
int read_len = 0;

// Up to two connection targets for one central server: [0] primary,
// [1] fallback (e.g. wired vs Wi-Fi NIC on the same Pi). cand_ip is kept
// alongside cand_addr because certificate IP verification takes a string,
// not the sockaddr_in -- and it has to be pinned to whichever candidate THIS
// attempt is actually dialling, since a cert may list one NIC's address in
// its SAN and not the other's. See the per-attempt pin in advance() below.
sockaddr_in cand_addr[2];
char cand_ip[2][INET_ADDRSTRLEN];
bool cand_valid[2] = {false, false};
int active_candidate = 0;

// Whether THIS connection attempt (since the last close_link()) ever reached
// ONLINE. Doubles as "suppress candidate rotation on this close": true means
// either the attempt succeeded, or the close was requested (set_enabled(0),
// central_tls_sender_close()) rather than failed -- see close_link().
bool reached_online = false;

long last_try = 0;
unsigned long seq = 0;

// When the current connect()/SSL_connect() attempt started. Neither call has
// a timeout of its own: without this a SYN that is silently dropped parks
// the state machine in TCP_PENDING/TLS_PENDING forever, and because a
// pending state never returns to OFFLINE the retry below never runs again.
long phase_start = 0;

// Operator kill switch (CENTRAL_LINK). Deliberately NOT reset by init() so a
// disabled link stays disabled if anything re-inits the sender.
bool link_enabled = true;

long now_ms() {
  timeval tv;
  gettimeofday(&tv, NULL);
  return (long)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

void close_link() {
  if (ssl) { SSL_free(ssl); ssl = NULL; }
  if (fd >= 0) { close(fd); fd = -1; }
  // A close that never reached ONLINE was a failed connection attempt on
  // whichever candidate we just tried -- rotate so the NEXT attempt (after
  // POSE_RECONNECT_MS, in advance()'s OFFLINE branch) tries the other one.
  // A close that DID reach ONLINE (server dropped an established link) or
  // that was requested (set_enabled(0), central_tls_sender_close()) keeps
  // the same candidate -- see reached_online's callers.
  if (!reached_online && cand_valid[1]) active_candidate = 1 - active_candidate;
  reached_online = false;
  state = OFFLINE;
  read_len = 0;
}

int write_line(const char* text) {
  char line[CENTRAL_TLS_MAX_LINE];
  int n = snprintf(line, sizeof(line), "%s\n", text);
  if (!ssl || n <= 0 || n >= (int)sizeof(line)) return -1;
  int sent = SSL_write(ssl, line, n);
  if (sent == n) return 0;
  close_link();  // partial JSON must not contaminate a new stream
  return -1;
}

void advance() {
  if (!link_enabled) return;

  if (state == OFFLINE) {
    long now = now_ms();
    if (!cand_valid[0] || now - last_try < POSE_RECONNECT_MS) return;
    // Rotation in close_link() only ever points at a cand_valid slot, but
    // guard anyway: an unconfigured fallback must never be dialled.
    if (!cand_valid[active_candidate]) active_candidate = 0;
    last_try = now;
    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return;
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    phase_start = now;
    const sockaddr_in& target = cand_addr[active_candidate];
    int rc = connect(fd, (const sockaddr*)&target, sizeof(target));
    if (!rc) state = TLS_PENDING;
    else if (errno == EINPROGRESS) state = TCP_PENDING;
    else close_link();
  }

  // A stalled handshake is indistinguishable from a slow one, so bound both
  // phases and let the OFFLINE branch start a clean attempt.
  if ((state == TCP_PENDING || state == TLS_PENDING) &&
      now_ms() - phase_start > CENTRAL_TLS_HANDSHAKE_MS) {
    close_link();
    return;
  }

  if (state == TCP_PENDING) {
    fd_set w;
    FD_ZERO(&w);
    FD_SET(fd, &w);
    timeval z = {0, 0};
    int rc = select(fd + 1, NULL, &w, NULL, &z);
    if (rc <= 0) return;
    int e = 0;
    socklen_t s = sizeof(e);
    if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &e, &s) || e) { close_link(); return; }
    state = TLS_PENDING;
  }

  if (state == TLS_PENDING) {
    if (!ssl) {
      ssl = SSL_new(ctx);
      if (!ssl) { close_link(); return; }
      // Pin certificate IP verification to WHICHEVER candidate this attempt
      // is actually connecting to. Per-SSL (SSL_get0_param), not per-CTX:
      // the active candidate can change between attempts (rotation in
      // close_link() above), and a cert whose SAN lists only one NIC's
      // address must not be checked against the other's.
      X509_VERIFY_PARAM_set1_ip_asc(SSL_get0_param(ssl), cand_ip[active_candidate]);
      SSL_set_fd(ssl, fd);
      SSL_set_connect_state(ssl);
    }
    int rc = SSL_connect(ssl);
    if (rc == 1) {
      if (SSL_get_verify_result(ssl) != X509_V_OK) { close_link(); return; }
      X509* peer = SSL_get_peer_certificate(ssl);
      if (!peer) { close_link(); return; }
      X509_free(peer);
      state = ONLINE;
      reached_online = true;
      char hello[128];
      snprintf(hello, sizeof(hello),
               "{\"type\":\"HELLO\",\"seq\":%lu,\"payload\":{\"role\":\"CCTV\"}}", seq++);
      write_line(hello);
    } else {
      int e = SSL_get_error(ssl, rc);
      if (e != SSL_ERROR_WANT_READ && e != SSL_ERROR_WANT_WRITE) close_link();
    }
  }
}

// Shared by init() (candidate 0) and set_fallback() (candidate 1): fill in
// cand_addr/cand_ip[slot] from ip/port, or mark the slot invalid on any
// parse failure so it can never be dialled.
bool set_candidate(int slot, const char* ip, int port) {
  if (!ip) { cand_valid[slot] = false; return false; }
  sockaddr_in a;
  memset(&a, 0, sizeof(a));
  a.sin_family = AF_INET;
  a.sin_port = htons((unsigned short)port);
  if (inet_pton(AF_INET, ip, &a.sin_addr) != 1) { cand_valid[slot] = false; return false; }
  cand_addr[slot] = a;
  strncpy(cand_ip[slot], ip, sizeof(cand_ip[slot]) - 1);
  cand_ip[slot][sizeof(cand_ip[slot]) - 1] = '\0';
  cand_valid[slot] = true;
  return true;
}

}  // namespace

int central_tls_sender_init(const char* ip, int port, const char* ca) {
  central_tls_sender_close();
  if (!ip || !ca || !ca[0]) return -1;
  SSL_load_error_strings();
  OpenSSL_add_ssl_algorithms();
  ctx = SSL_CTX_new(TLS_client_method());
  if (!ctx) return -1;
  SSL_CTX_set_min_proto_version(ctx, TLS1_2_VERSION);
  SSL_CTX_set_verify(ctx, SSL_VERIFY_PEER, NULL);
  // IP verification is pinned per-connection now (see advance()'s TLS_PENDING
  // branch), not here -- a fallback candidate may need a different IP pinned
  // than the one passed to this call.
  if (SSL_CTX_load_verify_locations(ctx, ca, NULL) != 1) {
    central_tls_sender_close();
    return -1;
  }
  if (!set_candidate(0, ip, port)) {
    central_tls_sender_close();
    return -1;
  }
  active_candidate = 0;
  last_try = now_ms() - POSE_RECONNECT_MS;  // allow the first attempt immediately
  return 0;
}

int central_tls_sender_set_fallback(const char* ip) {
  if (!cand_valid[0]) return -1;  // no primary/port to reuse yet
  return set_candidate(1, ip, ntohs(cand_addr[0].sin_port)) ? 0 : -1;
}

const char* central_tls_sender_active_ip(void) {
  return cand_valid[active_candidate] ? cand_ip[active_candidate] : "";
}

int central_tls_sender_send_pos(int ch, const float c[4][2]) {
  if (!c) return -1;
  advance();
  if (state != ONLINE) return -1;
  char j[352];
  int n = snprintf(j, sizeof(j),
      "{\"type\":\"POS\",\"seq\":%lu,\"payload\":{\"ch\":%d,"
      "\"corners\":[[%.2f,%.2f],[%.2f,%.2f],[%.2f,%.2f],[%.2f,%.2f]]}}",
      seq++, ch, c[0][0], c[0][1], c[1][0], c[1][1], c[2][0], c[2][1], c[3][0], c[3][1]);
  return (n > 0 && n < (int)sizeof(j)) ? write_line(j) : -1;
}

int central_tls_sender_send_typed(const char* type, const char* payload_json) {
  if (!type || !payload_json) return -1;
  advance();
  if (state != ONLINE) return -1;
  char j[CENTRAL_TLS_MAX_LINE];
  int n = snprintf(j, sizeof(j), "{\"type\":\"%s\",\"seq\":%lu,\"payload\":%s}",
                   type, seq++, payload_json);
  return (n > 0 && n < (int)sizeof(j)) ? write_line(j) : -1;
}

int central_tls_sender_poll_command(char* out, int out_len) {
  if (!out || out_len < 2) return 0;
  advance();
  if (state != ONLINE) return 0;

  int n = SSL_read(ssl, read_buf + read_len, (int)sizeof(read_buf) - 1 - read_len);
  if (n > 0) {
    read_len += n;
  } else {
    int e = SSL_get_error(ssl, n);
    if (e != SSL_ERROR_WANT_READ && e != SSL_ERROR_WANT_WRITE) { close_link(); return 0; }
  }

  read_buf[read_len] = '\0';
  char* nl = strchr(read_buf, '\n');
  if (!nl) return 0;
  *nl = '\0';

  // Parse before consuming: `out` and everything central_cmd_parse() reads come
  // from read_buf, which the memmove below shifts. Whichever command matches (or
  // none), the consume happens exactly once at the end — see
  // docs/08.06/CCTV_ACTION_ITEMS_20260806.md C-2's warning about a consume
  // that leaks past a branch: that re-reads the same line forever.
  //
  // The parsing itself moved to central_cmd_parse.cc (2026-08-12) so it can be
  // tested on a development PC: this file pulls in openssl, so anything living
  // here can only be built for the camera and only verified with a real server
  // attached. The parser has no such dependency.
  int result = central_cmd_parse(read_buf, out, out_len);

  int used = (int)(nl - read_buf) + 1;
  memmove(read_buf, read_buf + used, read_len - used);
  read_len -= used;
  return result;
}

void central_tls_sender_set_enabled(int on) {
  bool want = (on != 0);
  if (want == link_enabled) return;
  link_enabled = want;
  // Turning off must tear the session down, not just stop retrying, or the
  // server would keep seeing a registered CCTV that never sends anything.
  // Not a failed attempt, so suppress rotation (see close_link()).
  if (!want) { reached_online = true; close_link(); }
  else last_try = now_ms() - POSE_RECONNECT_MS;
}

int central_tls_sender_enabled(void) { return link_enabled ? 1 : 0; }

const char* central_tls_sender_state(void) {
  if (!link_enabled) return "disabled";
  if (!cand_valid[0]) return "offline";
  switch (state) {
    case ONLINE:      return "online";
    case TLS_PENDING: return "handshaking";
    case TCP_PENDING: return "connecting";
    default:          return "offline";
  }
}

void central_tls_sender_close() {
  reached_online = true;  // teardown, not a failed attempt -- suppress rotation
  close_link();
  if (ctx) { SSL_CTX_free(ctx); ctx = NULL; }
  cand_valid[0] = false;
  cand_valid[1] = false;
  active_candidate = 0;
  seq = 0;
}
