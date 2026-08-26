# MAX98357A 오디오 알림 설치 및 연동 가이드

로봇 상태와 안전 이벤트를 WAV 음성으로 알리는 오디오 모듈입니다. LED 스트립과
같이 커널 계층이 정확한 하드웨어 타이밍을 담당하고, `AudioStripManager`는 제어
루프를 막지 않는 백그라운드 스레드에서 음원을 재생합니다.

```text
robot_exec
  +-- AudioStripManager::Play(SoundId)
        +-- audio/wav_files/*.wav
        +-- /dev/audio_strip       4096-byte PCM period + DRAIN/DROP ioctl
              +-- audio_strip_driver.ko
                    +-- PCM/I2S + DMA -> GPIO18/19/21 -> MAX98357A
```

## 1. 보장하는 동작

- 로봇 제어 루프에서는 `Play()`만 호출하며 파일 및 장치 I/O를 기다리지 않습니다.
- 각 WAV의 마지막 DMA/FIFO 데이터까지 출력한 후 I2S를 정지합니다.
- 새 재생은 이전 링 데이터를 반복하지 않고 깨끗한 스트림에서 시작합니다.
- 긴급정지와 사람 진입 경보는 낮은 우선순위 안내를 즉시 중단하고 재생합니다.
- 오디오 초기화나 음원에 문제가 있어도 UART, 주행, 노즐, LED 동작은
  계속됩니다.
- 음원 경로는 `$HOME`에 의존하지 않습니다. 기본은 저장소의
  `audio/wav_files`, 설치 후에는 `/opt/road-painter/audio/wav_files`입니다.

## 2. 배선

| Raspberry Pi 4 | BCM GPIO | MAX98357A |
| --- | --- | --- |
| 물리 2 또는 4 | 5V | VIN |
| 물리 6 | GND | GND |
| 물리 12 | GPIO18 / PCM_CLK | BCLK |
| 물리 35 | GPIO19 / PCM_FS | LRC |
| 물리 40 | GPIO21 / PCM_DOUT | DIN |

GPIO18은 오디오가 사용하므로 WS2812B LED는 GPIO12를 사용해야 합니다.

## 3. 빌드

```bash
cd Paint_Robot/RaspberryPi
cmake -S . -B build
cmake --build build --target robot_exec audio_strip_test
make -C driver audio
```

생성물은 `build/robot_exec`, `build/audio_strip_test`,
`driver/audio_strip_driver.ko`, `driver/audio_strip_overlay.dtbo`입니다.

## 4. 최초 설치

현재 ALSA `hifiberry-dac`와 raw 오디오 드라이버는 같은 I2S 컨트롤러를 사용하므로
동시에 활성화할 수 없습니다. 설치 스크립트가 기존 overlay 행을 주석 처리하고 원본
부팅 설정을 백업합니다.

```bash
cd Paint_Robot/RaspberryPi/audio
chmod +x install_audio.sh play.sh
sudo ./install_audio.sh
sudo reboot
```

스크립트가 수행하는 작업은 다음과 같습니다.

1. 커널 모듈과 DT overlay 빌드
2. `/lib/modules`와 `/boot/overlays`에 설치
3. `audio_strip_overlay` 등록 및 자동 모듈 로드 설정
4. `/dev/audio_strip`의 `audio` 그룹 권한 설정
5. WAV 10개를 `/opt/road-painter/audio/wav_files`에 설치
6. `/etc/default/road-painter-audio`에 WAV 경로 설치

## 5. 재부팅 후 확인

```bash
ls -l /dev/audio_strip
lsmod | grep audio_strip
dmesg | grep audio_strip
tr -d '\0' < /proc/device-tree/soc/i2s@7e203000/compatible
```

정상 compatible 값은 `roadpainter,audio-strip`입니다.

로봇 데몬을 중지한 상태에서 독립 테스트를 실행합니다. 장치는 동시 open을 거부하므로
`robot_exec`와 `audio_strip_test`를 동시에 실행하면 안 됩니다.

```bash
sudo systemctl stop robot_exec
./build/audio_strip_test --list
./build/audio_strip_test task_start
./build/audio_strip_test -v 50 -n 2 emergency_stop
./audio/play.sh task_complete
```

## 6. 음원 카탈로그

모든 파일은 44.1kHz, 16-bit, stereo PCM WAV입니다.

| `SoundId` | 파일 | 연결된 이벤트 |
| --- | --- | --- |
| `EMERGENCY_STOP` | `emergency_stop.wav` | 서버 ESTOP, 최우선 선점 |
| `LOW_BATTERY` | `low_battery.wav` | API 제공, 배터리 텔레메트리 연결 대기 |
| `PERSON_IN_ZONE` | `person_in_zone.wav` | 중앙 서버 TLS `ZONE_EVENT Enter` |
| `POWER_OFF` | `power_off.wav` | API 제공, 종료 이벤트 연결 대기 |
| `POWER_ON` | `power_on.wav` | `robot_exec` 오디오 초기화 완료 |
| `SIGNAL_LOST` | `signal_lost.wav` | 중앙 서버 연결 유실 |
| `SNAPSHOT` | `snapshot.wav` | API 제공, 스냅샷 이벤트 연결 대기 |
| `SYSTEM_ERROR` | `system_error.wav` | 위치 유실 HOLD 진입 |
| `TASK_COMPLETE` | `task_complete.wav` | PATH 완료 |
| `TASK_START` | `task_start.wav` | 새 PATH 적용 |

배터리 값, 전원 종료, 스냅샷 이벤트는 현재 로봇 프로토콜로 들어오지 않으므로 임의로
추정하지 않습니다. 해당 이벤트가 추가되면 `audio.Play(...)` 한 줄로 연결할 수
있으며 음원과 API는 이미 포함돼 있습니다.

## 7. Zone alarm

기존 `/home/sky/audio/zone_alarm.py`와 UDP 9999 우회 경로는 사용하지 않습니다.
사람 진입 경보도 수동·안전 명령과 동일한 중앙 TLS 프로토콜을 통과합니다.

```text
카메라 role=CCTV -> 중앙 서버 192.168.0.8:9000
                   -> ROBOT TLS 세션 -> NetworkManager
                   -> AudioStripManager::Play(PERSON_IN_ZONE)
```

카메라는 `ZONE_EVENT` payload에 `action:"Enter"`를 담아 서버로 전송합니다. 서버는
형식을 검사한 뒤 Enter만 로봇에 중계하고, 로봇은 일반 `CMD`와 별도 latch로 받아
ESTOP이나 수동 명령을 덮지 않습니다. `IVA_EVENT`와 Exit는 음성에 연결하지 않습니다.

## 8. 우선순위

```text
EMERGENCY_STOP
  > PERSON_IN_ZONE
  > LOW_BATTERY / SIGNAL_LOST / SYSTEM_ERROR
  > POWER_OFF
  > TASK_START / TASK_COMPLETE / SNAPSHOT
  > POWER_ON
```

재생 중 더 높은 우선순위 이벤트가 들어오면 `DROP`으로 기존 DMA 큐를 폐기하고 새
음원을 재생합니다. 같거나 낮은 우선순위 이벤트는 중복 방송을 막기 위해 버립니다.

## 9. 원복

설치 전 부팅 설정은 아래 파일에 한 번만 백업됩니다.

```text
/boot/config.txt.road-painter-audio.bak
```

배포판에 따라 `/boot/firmware/config.txt.road-painter-audio.bak`일 수 있습니다.
하드웨어 시험에서 문제가 생기면 백업과 현재 설정을 비교하여
`dtoverlay=audio_strip_overlay`를 제거하고 `dtoverlay=hifiberry-dac`를 다시
활성화한 뒤 재부팅합니다.

## 10. 하드웨어 시험 전 상태

커널 모듈, overlay, C++ 매니저와 CLI 빌드 및 WAV 포맷 검증까지는 호스트에서 할 수
있습니다. 실제 BCLK/LRCLK 극성, FIFO 샘플 정렬, 스피커 출력은 overlay 전환 후 실제
MAX98357A에서 확인해야 합니다.
