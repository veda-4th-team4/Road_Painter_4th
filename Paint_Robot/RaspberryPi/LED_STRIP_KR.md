# 타워램프(WS2812B) 상태표시 LED — 설치 및 연동 가이드

로봇 상태를 색으로 표시하는 타워램프(SR-MPU-002)입니다. 커널 디바이스 드라이버가
WS2812B 타이밍을 담당하고, 애플리케이션은 `LedStripManager` 클래스로 **상태만
알려주면** 됩니다.

```text
robot_exec (main.cpp)
  └─ LedStripManager     SetState(DRIVING) 한 줄. 렌더 스레드가 30fps로 프레임 생성
       └─ /dev/led_strip  LED당 R,G,B 3바이트 write
            └─ led_strip_driver.ko   PWM 시리얼라이저 + DMA
                 └─ GPIO12 (물리 32번) ──> WS2812B 7개
```

---

## 1. 하드웨어 배선

| 스트립 선 | 물리 핀 | 역할 |
| ------- | ----- | ---- |
| 갈색 | **4** | 5V |
| 흰색 | **6** | GND |
| 초록 | **32** | 데이터 (GPIO12 / PWM0_0) |

### GPIO12를 쓰는 이유

PWM0_0은 GPIO18(ALT5)과 GPIO12(ALT0) 두 곳에 나옵니다. 드라이버 원본 기본값은
GPIO18이었지만, **이 로봇에는 `max98357a` I2S 앰프가 GPIO18을 `PCM_CLK`로 점유**하고
있어 GPIO12로 옮겼습니다. 오버레이 기본값이 이미 GPIO12이므로 별도 파라미터는 필요
없습니다.

현재 40핀 헤더 점유 현황 (배선 변경 전 반드시 확인):

| GPIO | 물리 핀 | 기능 | 장치 |
| ---- | ----- | ---- | ---- |
| 2, 3 | 3, 5 | SDA1 / SCL1 | MPU6050 |
| 9, 10, 11 | 21, 19, 23 | SPI0 | SPI 장치 |
| 14, 15 | 8, 10 | TXD1 / RXD1 | STM32 |
| 18, 19, 20, 21 | 12, 35, 38, 40 | PCM_* | max98357a 앰프 |
| **12** | **32** | **PWM0_0** | **타워램프** |

### 전기적 주의

- **데이터 레벨**: WS2812B는 5V 전원 기준 VIH가 약 3.5V인데 Pi GPIO는 3.3V라 스펙
  밖입니다. LED 7개 정도는 대체로 동작하지만, 첫 LED가 색을 틀리게 받거나 깜빡이면
  레벨 시프터(74AHCT125)를 넣거나 스트립 VDD를 다이오드 1개로 ~4.4V까지 낮춥니다.
- **전류**: 7개 풀화이트 = 약 420mA. 기본 밝기를 25%로 낮춰둔 이유입니다. 모터와
  전원을 공유하면 전압 강하로 STM32/IMU가 리셋될 수 있습니다.

---

## 2. 최초 1회 설치

### 2-1. 커널 모듈 + 오버레이 빌드

```bash
cd ~/Road_Painter_4th/Paint_Robot/RaspberryPi/driver && make
```

`led_strip_driver.ko` 와 `led_strip_overlay.dtbo` 가 생성됩니다. 커널을 올렸다면
`.ko`를 반드시 다시 빌드해야 합니다 (vermagic 불일치 시 `insmod`가 거부).

### 2-2. 오버레이 등록 (재부팅 후에도 유지)

```bash
sudo cp ~/Road_Painter_4th/Paint_Robot/RaspberryPi/driver/led_strip_overlay.dtbo /boot/overlays/
```

```bash
echo 'dtoverlay=led_strip_overlay' | sudo tee -a /boot/config.txt
```

배선을 다른 핀으로 바꿨다면 파라미터로 덮어씁니다 (ALT0 = `4`, ALT5 = `2`):

```bash
# 예: GPIO18(물리 12번)로 되돌릴 때 — 단, I2S 앰프를 내려야 함
dtoverlay=led_strip_overlay,pin=18,func=2
```

LED 개수가 7개가 아니면 `led-count=` 로 바꿉니다.

### 2-3. 모듈 자동 로드

```bash
sudo cp ~/Road_Painter_4th/Paint_Robot/RaspberryPi/driver/led_strip_driver.ko /lib/modules/$(uname -r)/extra/ && sudo depmod -a && echo led_strip_driver | sudo tee /etc/modules-load.d/led-strip.conf
```

### 2-4. 권한 (중요)

`/dev/led_strip`은 기본이 `root` 전용입니다. **udev 규칙이 없으면 `robot_exec`를
일반 사용자로 실행할 때 램프가 조용히 비활성화됩니다.**

```bash
echo 'KERNEL=="led_strip", MODE="0660", GROUP="gpio"' | sudo tee /etc/udev/rules.d/99-led-strip.rules && sudo udevadm control --reload && sudo udevadm trigger
```

### 2-5. 재부팅

```bash
sudo reboot
```

---

## 3. 설치 확인

```bash
raspi-gpio get 12
```
→ `func=PWM0_0` 이어야 합니다. `INPUT`이면 오버레이가 안 걸린 것입니다.

```bash
tr -d '\0' < /proc/device-tree/soc/pwm@7e20c000/compatible
```
→ `roadpainter,led-strip` 이어야 합니다. `brcm,bcm2835-pwm`이면 오버레이 미적용.

```bash
ls -l /dev/led_strip && lsmod | grep led_strip
```

```bash
sudo dmesg | grep led_strip
```
→ `7 LEDs, N DMA words`, `PWM clock 2400000 Hz`, `ready on /dev/led_strip` 3줄이
정상입니다. probe 끝에 블랭크 프레임을 쏘므로, 전원 인가 시 랜덤 색으로 켜져 있던
스트립이 **이 시점에 꺼지면 배선이 맞다는 신호**입니다.

---

## 4. 점등 테스트

```bash
cd ~/Road_Painter_4th/Paint_Robot/RaspberryPi/build && cmake .. && make led_test
```

```bash
./led_test --brightness 20
```

인자 없이 실행하면 4개 상태를 6초씩 순환합니다.

```text
usage: led_test [state] [--leds N] [--brightness P]

  state         idle | driving | manual | estop
  --leds        LED 개수 (기본 7)
  --brightness  0-100 % (기본 25)
```

Ctrl+C로 종료하며, 종료 시 소등됩니다.

---

## 5. 애플리케이션 연동 (담당자가 볼 부분)

드라이버를 직접 만질 일은 없습니다. **부를 함수는 4개뿐**입니다.

```cpp
#include "LedStripManager.h"

LedStripManager led_strip;              // 기본 7개, /dev/led_strip

led_strip.Open();                       // ① 최초 1회. 렌더 스레드 시작
led_strip.SetBrightness(20);            // ② 선택 (기본 25%)
led_strip.SetState(LedState::DRIVING);  // ③ 상태가 바뀔 때마다
led_strip.Close();                      // ④ 종료 시. 소등 + 스레드 정리
```

| 함수 | 설명 |
| ---- | ---- |
| `Open()` | 디바이스 열고 렌더 스레드 시작. 실패 시 `false` |
| `SetState(LedState)` | **매 제어 루프에서 무조건 호출해도 됨.** atomic store 1회 |
| `SetBrightness(int)` | 0~100 %. 전원 예산 초과 방지 |
| `Close()` | 소멸자에서도 자동 호출됨 |

### 상태 정의

| `LedState` | 표시 | 의미 |
| ---------- | ---- | ---- |
| `IDLE` | 흰색 고정 | 대기 |
| `DRIVING` | 주황 chase | 자율 주행 중 |
| `MANUAL` | 초록 고정 | 수동 조작 |
| `ESTOP` | 빨강↔흰색 그라데이션 | 비상 정지 |

### 성능 걱정 안 해도 되는 이유

- `SetState()`는 atomic 저장만 하고 즉시 반환합니다. 제어 루프(50Hz)에서 매번 불러도
  됩니다.
- 렌더 스레드가 33ms(~30fps) 주기로 프레임을 만들고, **직전 프레임과 픽셀이 같으면
  `write()` 자체를 건너뜁니다.** 정지 상태(IDLE/MANUAL)에서 초당 30번씩 DMA를 돌리지
  않습니다.
- `Open()`이 실패해도 이후 `SetState()` 호출은 전부 무해하게 무시됩니다. **램프가
  없거나 모듈이 안 올라간 환경에서도 로봇 본체는 그대로 동작합니다.**

### `main.cpp` 적용 위치

**(1) 헤더 + 객체 선언**

```cpp
#include "LedStripManager.h"
...
LedStripManager led_strip;
```

**(2) IMU 초기화 직후**

```cpp
// 3-1. Initialize tower lamp status LED (SR-MPU-002)
if (!led_strip.Open()) {
    std::cout << "[MAIN] Warning: Tower lamp not available. Continuing without status LED." << std::endl;
}
```

**(3) ESTOP 구분용 플래그.** ESTOP은 `manual_override`도 같이 올리기 때문에 별도
플래그가 필요합니다. `ESTOP`에서 `true`, `RESUME` / `ABORT_DRAW` / 새 PATH 수신 시
`false`로 내립니다.

```cpp
bool estop_active = false;
```

**(4) 제어 루프에서 속도 결정 직전.** 우선순위 순서가 중요합니다.

```cpp
if (estop_active) {
    led_strip.SetState(LedState::ESTOP);
} else if (manual_override) {
    led_strip.SetState(LedState::MANUAL);
} else if (!path_follower.IsPathFinished()) {
    led_strip.SetState(LedState::DRIVING);
} else {
    led_strip.SetState(LedState::IDLE);
}
```

> HOLD 상태(`hold_active`, 위치 소실로 정지)를 어떻게 표시할지는 미정입니다. 현재
> 로직에서는 `DRIVING`으로 남습니다. 별도 색이 필요하면 `LedState`에 항목을 추가하고
> `LedStripManager::Render()`에 렌더 함수를 붙이면 됩니다.

**(5) 종료부**

```cpp
led_strip.Close();
robot_comm.Close();
```

**(6) `CMakeLists.txt`** — `SOURCES`에 `src/LedStripManager.cpp` 추가.

---

## 6. 드라이버 직접 제어 (다른 언어 / 빠른 확인용)

`/dev/led_strip`은 `write()` 하나뿐이고 ioctl은 없습니다. **LED당 R,G,B 3바이트**를
순서대로 쓰면 그 write 한 번이 DMA 전송 한 번(= 프레임 하나)입니다.

규칙 2가지:
1. 매 프레임 **`lseek(0)` 먼저** — 오프셋이 누적돼 `-ENOSPC`가 납니다.
2. 21바이트(7 × 3)를 한 번에 씁니다.

```bash
sudo sh -c 'printf "\xff\x00\x00\xff\x00\x00\xff\x00\x00\xff\x00\x00\xff\x00\x00\xff\x00\x00\xff\x00\x00" > /dev/led_strip'
```

---

## 7. 트러블슈팅

| 증상 | 원인 / 조치 |
| ---- | --------- |
| `raspi-gpio get 12` 가 `INPUT` | 오버레이 미적용. `/boot/config.txt` 확인 후 재부팅 |
| `insmod: Invalid module format` | 커널 버전 불일치. `driver/`에서 `make` 재실행 |
| `[MAIN] Warning: Tower lamp not available.` | 노드 없음 또는 권한. §2-4 udev 규칙 확인 |
| `open /dev/led_strip failed: Permission denied` | 위와 동일 |
| LED가 전혀 안 켜짐 | GND(6번) 공통 확인, 데이터선이 스트립 **DIN** 쪽인지 확인 |
| 첫 LED만 색이 틀림 | 3.3V 레벨 문제. §1 전기적 주의 참고 |
| 색이 랜덤하게 깜빡임 | 전원 부족 또는 클럭 이탈. `dmesg`에 `PWM clock is ... Hz, wanted` 경고 확인 |
| `dmesg`에 `DMA timed out` | DMA 채널 충돌. 다른 PWM 사용 오버레이가 있는지 확인 |

---

## 8. 동작 원리 (참고)

WS2812B는 클럭 선 없이 펄스 폭으로 0/1을 구분하고 허용 오차가 ±150ns입니다.
preemptible 커널에서 CPU가 GPIO를 토글하면 인터럽트 한 번에 프레임이 깨지므로,
**PWM 블록을 시리얼라이저 모드로 두고 DMA가 FIFO를 채우는** 방식을 씁니다. 타이밍이
PWM 클럭에서 나오므로 CPU 지터가 선에 도달할 경로 자체가 없습니다.

색 비트 1개(1.25µs)를 PWM 서브비트 3개로 인코딩합니다 → 클럭 2.4MHz.

| 색 비트 | 서브비트 | 파형 | 스펙 |
| ----- | ------ | ---- | ---- |
| `1` | `0b110` | 833ns high / 417ns low | T1H 800±150, T1L 450±150 |
| `0` | `0b100` | 417ns high / 833ns low | T0H 400±150, T0L 850±150 |

유저스페이스는 R,G,B로 주지만 스트립은 G,R,B로 받으므로 드라이버가 순서를 바꿉니다.
프레임 끝 latch(기본 300µs)는 별도 delay가 아니라 **DMA 버퍼 뒤쪽의 0 워드**로
만듭니다.

모듈 파라미터: `led_count`(0이면 DT의 `led-count` 사용), `reset_us`(기본 300).

---

## 9. 파일 목록

| 파일 | 내용 |
| ---- | ---- |
| `driver/led_strip_driver.c` | 커널 드라이버 본체 (PWM 시리얼라이저 + DMA + 캐릭터 디바이스) |
| `driver/led_strip_regs.h` | BCM2711 PWM 레지스터 맵, WS2812B 인코딩 상수 |
| `driver/led_strip_overlay.dts` | 디바이스 트리 오버레이 (PWM0 선점, GPIO12 ALT0, DMA) |
| `driver/Makefile` | 모듈 + dtbo 빌드 |
| `include/LedStripManager.h` | 애플리케이션 API |
| `src/LedStripManager.cpp` | 렌더 스레드, 상태별 애니메이션, 프레임 write |
| `src/led_test.cpp` | 단독 점등 테스트 툴 |

---

## 10. 이 브랜치의 범위 (담당자 인수인계)

**`src/main.cpp` 변경은 이 브랜치에 포함되어 있지 않습니다.** 로봇 제어 루프는 담당
팀원의 영역이라 건드리지 않았습니다. 이 브랜치가 제공하는 것은 다음뿐이며, 모두
독립적으로 빌드·동작합니다.

- 커널 드라이버 + 디바이스 트리 오버레이 (`driver/led_strip_*`)
- 애플리케이션 API (`include/LedStripManager.h`, `src/LedStripManager.cpp`)
- 단독 테스트 툴 (`src/led_test.cpp`)
- 빌드 설정 (`CMakeLists.txt`, `driver/Makefile`) 및 본 문서

`CMakeLists.txt`의 `SOURCES`에 `LedStripManager.cpp`가 이미 들어 있으므로, **§5의
6단계를 `main.cpp`에 붙이면 바로 동작합니다.** 빌드 설정은 손댈 필요 없습니다.

### `main.cpp`에 넣을 때 주의 (최신 main 구조 기준)

최신 `main`의 `main.cpp`는 Protocol v2(op_index 매칭, HOLD, ops 배열 파싱), READY/GO
통합, `ABORT_DRAW` 추가로 구조가 바뀌어 있습니다. §5를 적용할 때 아래를 반영하세요.

| 지점 | 주의 |
| ---- | ---- |
| PATH 수신 | `pending_path` 버퍼링 구조이므로, `estop_active = false;` 는 **PATH를 실제로 적용하는 블록**(`if (has_pending_path)` 안)에 넣습니다 |
| `ABORT_DRAW` 핸들러 | `manual_override`를 내리므로 `estop_active = false;` 도 함께 넣습니다 |
| 속도 결정 분기 | 조건이 `if (manual_override \|\| hold_active)` 로 바뀌었습니다. 타워램프 분기는 그 **앞에** 둡니다 |
| HOLD 표시 | §5의 인용 블록 참고. 현재 로직에서는 `DRIVING`으로 남습니다 |

> **별건 — main 브랜치 컴파일 오류**: 현재 `main`의 `src/main.cpp`는 `ABORT_DRAW`
> 핸들러(86~87행)가 `has_pending_path` / `pending_path`를 쓰는데 선언이 123~124행에
> 있어 컴파일되지 않습니다. 병합 시 `static` 선언 2줄을 `while` 루프 위로 올려야
> 합니다. LED 작업과 무관한 기존 이슈이므로 담당자에게 별도 공유가 필요합니다.
