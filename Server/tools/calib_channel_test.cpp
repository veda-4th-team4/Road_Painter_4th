// 채널별 캘리브레이션 저장(프로토콜 v0.4)의 위험 지점을 찍어보는 테스트.
//   make calib_channel_test && ./tools/calib_channel_test
//
// 여기서 보는 것들은 전부 "틀려도 에러가 안 나는" 종류다 - 그래서 테스트로 못박는다:
//  1) 예전 형식(번들 하나)이 채널 1로 읽히는가.
//     ⚠️ 이미 현장에 배포된 users.json / calib_latest.json 이 예전 형식이다.
//     이게 깨지면 서버는 잘 뜨는데 캘리브레이션만 조용히 사라진다.
//  2) 채널 하나를 저장할 때 다른 채널이 날아가지 않는가.
//     맵 전체를 대입하는 실수를 하면 채널 2를 캘리하는 순간 채널 1이 없어진다.
//  3) "계정 → 없으면 전역" 폴백이 채널 단위로 도는가 (맵 단위로 돌면 안 된다).
//  4) channelOf 의 하위호환 (ch 없음/범위 밖 -> 1). 단일 채널 카메라(PNO)와
//     v0.3 클라이언트가 ch를 한 번도 안 실어도 그대로 돌아야 한다.
#include "calib.hpp"
#include "user_store.hpp"
#include <cstdio>
#include <cstdlib>

static int fails = 0;
#define CHECK(cond, ...)                                                   \
    do {                                                                   \
        if (cond) { printf("  ok   "); } else { printf("  FAIL "); ++fails; } \
        printf(__VA_ARGS__); printf("\n");                                 \
    } while (0)

// 매 실행이 빈 상태에서 시작하게 한다 (앞 실행이 남긴 파일을 읽으면 통과가 거짓말이 된다)
static void freshDir(const char* path) {
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "rm -rf %s && mkdir -p %s", path, path);
    if (system(cmd) != 0) {
        printf("  FAIL 테스트 디렉터리 준비 실패: %s\n", path);
        exit(1);
    }
}

static json bundle(const char* tag) {
    return json{{"version", 1}, {"tag", tag},
                {"H_floor", {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}};
}
static std::string tagOf(const json& j) {
    return j.is_object() ? j.value("tag", "<none>") : "<not-object>";
}

int main() {
    printf("[1] 예전 형식(번들 하나) -> 채널 1 승격\n");
    {
        json old = bundle("legacy");
        CHECK(!isCalibChannelMap(old), "번들은 채널 맵으로 오인되지 않는다");
        json m = asCalibChannelMap(old);
        CHECK(m.size() == 1 && tagOf(m["1"]) == "legacy", "채널 1의 번들이 된다");
        CHECK(tagOf(calibOfChannel(old, 1)) == "legacy", "calibOfChannel(_,1) 로 꺼내진다");
        CHECK(calibOfChannel(old, 2).is_null(), "채널 2는 null");
    }
    {   // 레거시 배열 H (v0.2)
        json legacyArr = json{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        CHECK(!isCalibChannelMap(legacyArr), "레거시 배열도 맵이 아니다");
        CHECK(asCalibChannelMap(legacyArr)["1"].is_array(), "배열도 채널 1로 승격");
    }
    {
        json m = json{{"1", bundle("a")}, {"2", bundle("b")}};
        CHECK(isCalibChannelMap(m), "채널 맵은 맵으로 인식된다");
        CHECK(tagOf(asCalibChannelMap(m)["2"]) == "b", "맵은 그대로 통과");
    }

    printf("[2] channelOf 하위호환\n");
    CHECK(channelOf(json::object()) == 1, "ch 없으면 1");
    CHECK(channelOf(json{{"ch", 3}}) == 3, "ch=3 -> 3");
    CHECK(channelOf(json{{"ch", 0}}) == 1, "범위 밖(0) -> 1");
    CHECK(channelOf(json{{"ch", 99}}) == 1, "범위 밖(99) -> 1");
    CHECK(channelOf(json{{"ch", "2"}}) == 1, "문자열 ch -> 1 (정수만 인정)");

    printf("[3] UserStore 채널별 저장/폴백\n");
    freshDir("/tmp/rp_chtest");
    {
        UserStore us("/tmp/rp_chtest/users.json");
        std::string err;
        CHECK(us.registerUser("u1", "pw", "", err), "사용자 등록");

        us.setCalib("u1", 1, bundle("ch1"));
        us.setCalib("u1", 2, bundle("ch2"));
        CHECK(tagOf(us.getCalib("u1", 1)) == "ch1", "채널 1 저장 후 조회");
        CHECK(tagOf(us.getCalib("u1", 2)) == "ch2", "채널 2 를 저장해도 채널 1 은 그대로");

        us.setCalib("u1", 1, bundle("ch1-new"));
        CHECK(tagOf(us.getCalib("u1", 1)) == "ch1-new", "채널 1 갱신");
        CHECK(tagOf(us.getCalib("u1", 2)) == "ch2", "채널 1 갱신이 채널 2 를 안 건드린다");

        // 전역 슬롯 폴백: 계정에 없는 채널 3 은 전역에서 와야 한다
        us.setGlobalCalib(3, bundle("global3"));
        us.setGlobalCalib(1, bundle("global1"));
        CHECK(tagOf(us.getCalib("u1", 3)) == "global3", "계정에 없는 채널은 전역에서");
        CHECK(tagOf(us.getCalib("u1", 1)) == "ch1-new", "계정 값이 전역을 이긴다");

        json all = us.getCalibs("u1");
        CHECK(tagOf(all["1"]) == "ch1-new" && tagOf(all["2"]) == "ch2" &&
              tagOf(all["3"]) == "global3", "getCalibs 가 계정+전역을 합친다");
    }

    printf("[4] 재기동 후에도 살아있는가 (파일 영속)\n");
    {
        UserStore us("/tmp/rp_chtest/users.json");
        CHECK(tagOf(us.getCalib("u1", 2)) == "ch2", "채널 2 가 파일에서 복원");
        CHECK(tagOf(us.getCalib("u1", 3)) == "global3", "전역 채널 3 도 복원");
    }

    printf("[5] 배포돼 있던 예전 파일이 그대로 뜨는가\n");
    freshDir("/tmp/rp_chtest2");
    {   // 예전 서버가 남긴 형식: calib 이 번들 하나, calib_latest.json 도 번들 하나
        FILE* f = fopen("/tmp/rp_chtest2/users.json", "w");
        fprintf(f, "{\"old\":{\"salt\":\"aa\",\"hash\":\"bb\","
                   "\"calib\":{\"version\":1,\"tag\":\"oldacct\","
                   "\"H_floor\":[[1,0,0],[0,1,0],[0,0,1]]}}}");
        fclose(f);
        f = fopen("/tmp/rp_chtest2/calib_latest.json", "w");
        fprintf(f, "{\"version\":1,\"tag\":\"oldglobal\","
                   "\"H_floor\":[[1,0,0],[0,1,0],[0,0,1]]}");
        fclose(f);

        UserStore us("/tmp/rp_chtest2/users.json");
        CHECK(tagOf(us.getCalib("old", 1)) == "oldacct",
              "예전 계정 번들이 채널 1 로 읽힌다");
        CHECK(us.getCalib("old", 2).is_null(), "채널 2 는 여전히 비어 있다");
        CHECK(tagOf(us.getGlobalCalib(1)) == "oldglobal",
              "예전 전역 번들도 채널 1 로 읽힌다");

        // 채널 2 를 새로 저장해도 예전 채널 1 이 살아남아야 한다
        us.setCalib("old", 2, bundle("new2"));
        CHECK(tagOf(us.getCalib("old", 1)) == "oldacct",
              "채널 2 추가 후에도 예전 채널 1 유지");
    }

    printf("\n%s (%d fail)\n", fails ? "실패" : "전부 통과", fails);
    return fails ? 1 : 0;
}
