#include "central_tls_sender.h"
#include "app_config.h"
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
static State state = OFFLINE; static int fd = -1; static SSL_CTX* ctx = NULL;
static SSL* ssl = NULL; static sockaddr_in addr; static bool have_addr = false;
static long last_try = 0; static unsigned long seq = 0; static char read_buf[1024];
static int read_len = 0;
// When the current connect()/SSL_connect() attempt started. Neither call has a
// timeout of its own: without this a SYN that is silently dropped parks the
// state machine in TCP_PENDING/TLS_PENDING forever, and because a pending
// state never returns to OFFLINE the 2 s retry below never runs again. The
// link then stays down until the app restarts, with no socket to show for it.
static long phase_start = 0;
// Operator kill switch (CENTRAL_LINK). Deliberately NOT reset by init() so a
// disabled link stays disabled if anything re-inits the sender.
static bool link_enabled = true;
static long now_ms() { timeval tv; gettimeofday(&tv, NULL); return (long)tv.tv_sec * 1000 + tv.tv_usec / 1000; }
static void close_link() { if (ssl) { SSL_free(ssl); ssl=NULL; } if (fd>=0) { close(fd); fd=-1; } state=OFFLINE; read_len=0; }
static int write_line(const char* text) {
    char line[CENTRAL_TLS_MAX_LINE]; int n=snprintf(line,sizeof(line),"%s\n",text);
    if (!ssl || n<=0 || n >= (int)sizeof(line)) return -1;
    int sent=SSL_write(ssl,line,n); if (sent==n) return 0;
    close_link(); return -1; // partial JSON must not contaminate a new stream
}
static void advance() {
    if (!link_enabled) return;
    if (state==OFFLINE) {
        long now=now_ms(); if (!have_addr || now-last_try<POSE_RECONNECT_MS) return;
        last_try=now; fd=socket(AF_INET,SOCK_STREAM,0); if (fd<0) return;
        fcntl(fd,F_SETFL,fcntl(fd,F_GETFL,0)|O_NONBLOCK); int one=1; setsockopt(fd,IPPROTO_TCP,TCP_NODELAY,&one,sizeof(one));
        phase_start=now;
        int rc=connect(fd,(sockaddr*)&addr,sizeof(addr)); if (!rc) state=TLS_PENDING; else if (errno==EINPROGRESS) state=TCP_PENDING; else close_link();
    }
    // A stalled handshake is indistinguishable from a slow one, so bound both
    // phases and let the OFFLINE branch start a clean attempt.
    if ((state==TCP_PENDING || state==TLS_PENDING) && now_ms()-phase_start>CENTRAL_TLS_HANDSHAKE_MS) { close_link(); return; }
    if (state==TCP_PENDING) {
        fd_set w; FD_ZERO(&w); FD_SET(fd,&w); timeval z={0,0}; int rc=select(fd+1,NULL,&w,NULL,&z); if (rc<=0) return;
        int e=0; socklen_t s=sizeof(e); if (getsockopt(fd,SOL_SOCKET,SO_ERROR,&e,&s)||e) { close_link(); return; } state=TLS_PENDING;
    }
    if (state==TLS_PENDING) {
        if (!ssl) { ssl=SSL_new(ctx); if (!ssl) { close_link(); return; } SSL_set_fd(ssl,fd); SSL_set_connect_state(ssl); }
        int rc=SSL_connect(ssl); if (rc==1) {
            if (SSL_get_verify_result(ssl)!=X509_V_OK) { close_link(); return; }
            X509* peer=SSL_get_peer_certificate(ssl); if (!peer) { close_link(); return; } X509_free(peer); state=ONLINE;
            char hello[128]; snprintf(hello,sizeof(hello),"{\"type\":\"HELLO\",\"seq\":%lu,\"payload\":{\"role\":\"CCTV\"}}",seq++); write_line(hello);
        } else { int e=SSL_get_error(ssl,rc); if (e!=SSL_ERROR_WANT_READ && e!=SSL_ERROR_WANT_WRITE) close_link(); }
    }
}
}

int central_tls_sender_init(const char* ip,int port,const char* ca) {
    central_tls_sender_close(); if (!ip||!ca||!ca[0]) return -1;
    SSL_load_error_strings(); OpenSSL_add_ssl_algorithms(); ctx=SSL_CTX_new(TLS_client_method()); if (!ctx) return -1;
    SSL_CTX_set_min_proto_version(ctx,TLS1_2_VERSION); SSL_CTX_set_verify(ctx,SSL_VERIFY_PEER,NULL);
    if (SSL_CTX_load_verify_locations(ctx,ca,NULL)!=1 || X509_VERIFY_PARAM_set1_ip_asc(SSL_CTX_get0_param(ctx),ip)!=1) { central_tls_sender_close(); return -1; }
    memset(&addr,0,sizeof(addr)); addr.sin_family=AF_INET; addr.sin_port=htons((unsigned short)port);
    if (inet_pton(AF_INET,ip,&addr.sin_addr)!=1) { central_tls_sender_close(); return -1; }
    have_addr=true; last_try=now_ms()-POSE_RECONNECT_MS; return 0;
}
int central_tls_sender_send_pos(const float c[4][2]) {
    if (!c) return -1; advance(); if (state!=ONLINE) return -1; char j[320];
    int n=snprintf(j,sizeof(j),"{\"type\":\"POS\",\"seq\":%lu,\"payload\":{\"corners\":[[%.2f,%.2f],[%.2f,%.2f],[%.2f,%.2f],[%.2f,%.2f]]}}",seq++,c[0][0],c[0][1],c[1][0],c[1][1],c[2][0],c[2][1],c[3][0],c[3][1]);
    return (n>0 && n<(int)sizeof(j)) ? write_line(j) : -1;
}
int central_tls_sender_send_typed(const char* type,const char* payload_json) {
    if (!type||!payload_json) return -1; advance(); if (state!=ONLINE) return -1;
    char j[CENTRAL_TLS_MAX_LINE];
    int n=snprintf(j,sizeof(j),"{\"type\":\"%s\",\"seq\":%lu,\"payload\":%s}",type,seq++,payload_json);
    return (n>0 && n<(int)sizeof(j)) ? write_line(j) : -1;
}
int central_tls_sender_poll_command(char* out,int out_len) {
    if (!out||out_len<2) return 0; advance(); if (state!=ONLINE) return 0;
    int n=SSL_read(ssl,read_buf+read_len,(int)sizeof(read_buf)-1-read_len);
    if (n>0) read_len+=n; else { int e=SSL_get_error(ssl,n); if(e!=SSL_ERROR_WANT_READ&&e!=SSL_ERROR_WANT_WRITE) { close_link(); return 0; } }
    read_buf[read_len]='\0'; char* nl=strchr(read_buf,'\n'); if(!nl) return 0; *nl='\0';
    bool ok=strstr(read_buf,"\"type\":\"CMD\"")&&strstr(read_buf,"\"cmd\":\"CALIB_START\""); int used=(int)(nl-read_buf)+1; memmove(read_buf,read_buf+used,read_len-used); read_len-=used;
    if(!ok) return 0; snprintf(out,(size_t)out_len,"CALIB_START"); return 1;
}
void central_tls_sender_set_enabled(int on) {
    bool want = (on != 0); if (want==link_enabled) return; link_enabled=want;
    // Turning off must tear the session down, not just stop retrying, or the
    // server would keep seeing a registered CCTV that never sends anything.
    if (!want) close_link(); else last_try=now_ms()-POSE_RECONNECT_MS;
}
int central_tls_sender_enabled(void) { return link_enabled ? 1 : 0; }
const char* central_tls_sender_state(void) {
    if (!link_enabled) return "disabled";
    if (!have_addr) return "offline";
    switch (state) {
    case ONLINE:      return "online";
    case TLS_PENDING: return "handshaking";
    case TCP_PENDING: return "connecting";
    default:          return "offline";
    }
}
void central_tls_sender_close() { close_link(); if(ctx) { SSL_CTX_free(ctx); ctx=NULL; } have_addr=false; seq=0; }
