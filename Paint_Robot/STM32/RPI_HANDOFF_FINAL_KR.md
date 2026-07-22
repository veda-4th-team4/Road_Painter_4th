# 도장 로봇 RPi 연동 

> **로봇 RPi가 만들/보낼 것**.

## 1. 역할


| 장치         | 할 일                                               |
| ---------- | ------------------------------------------------- |
| 서버         | PATH / POSE / CMD, DONE 수신                        |
| **로봇 RPi** | 추종·P/PD -> **left_sps, right_sps**, 노즐, 서버 STATUS |
| **STM32**  | SPS->STEP, STATUS steps/flags                     |


STM에 m/도/PATH 보내지 말 것. 최종 출력은 **left_sps, right_sps**만.

---

## 2. 데이터 흐름

```
CCTV --POSE--> 서버 --PATH/POSE/CMD--> 로봇 RPi
                              left_sps,right_sps |
                                                v
                                             STM32 -> 모터
                     STATUS(steps,flags) <------+
서버 <-- DONE/STATUS/FAULT <-- 로봇 RPi
```

---

## 3. RPi <-> STM32 와이어

- 115200-8-N-1, AA|LEN|CMD|PAYLOAD|CRC8|55, poly 0x07, LE


| CMD              | 방향       | 주기          | 내용               |
| ---------------- | -------- | ----------- | ---------------- |
| 0x01 SET_SPEED   | RPi->STM | 주행중 <=100ms | int16 L/R sps    |
| 0x02 NOZZLE      | RPi->STM | 변경시         | 0/1              |
| 0x03 ESTOP       | RPi->STM | 비상          | reason           |
| 0x04 CLEAR_ESTOP | RPi->STM | 재개          | 5A A5            |
| 0x81 STATUS      | STM->RPi | ~100ms      | stepsL/R + flags |




### SET_SPEED 규칙 (중요)


| 페이로드        | STM 동작                     |
| ----------- | -------------------------- |
| NOT 0       | 주행, 300ms watchdog ON      |
| (0,0)       | 정지, watchdog OFF. IR중이면 무시 |
| NOT 0 중 선끊김 | 300ms ESTOP                |


꼭지점: (0,0) → 계산 → 다음 SPS OK.  
작업 종료: (0,0) + 서버 DONE. **ESTOP으로 끝내지 말 것**.  
멈춘 뒤(유휴)에는 리모컨 가능. **움직이는 중에만** 리모컨 금지.

### STATUS 0x81 — STM이 RPi에게 보내는 상태 (100ms마다)

**방향: STM → 로봇 RPi 전용.** RPi가 0x81을 보내면 안 됨

전체 프레임 예:

```
AA 09 81 [left_steps 4B] [right_steps 4B] [flags 1B] [CRC] 55
```

PAYLOAD만 보면 **9바이트**.


| 바이트 위치 | 타입            | 이름          | 의미                        |
| ------ | ------------- | ----------- | ------------------------- |
| 0~3    | uint32, 리틀엔디안 | left_steps  | 왼쪽 모터가 지금까지 만든 STEP 누적 개수 |
| 4~7    | uint32, 리틀엔디안 | right_steps | 오른쪽 모터 STEP 누적 개수         |
| 8      | uint8         | flags       | 아래 비트 플래그                 |


**스텝 카운터 읽는 법**

- 전진이면 값이 증가, 후진이면 감소(실제로는 uint32라서 wrap)
- RPi에서는 같은 4바이트를 **signed int32로 재해석**한 뒤, 이전 값과의 차이로 “이번에 얼마나 돌았는지” 계산
- 이 값은 **드라이버에 넣은 펄스 수**이지, 바퀴가 땅에서 실제로 미끄러지지 않았다는 보장은 아님

flags **한 바이트 (bit = 1이면 해당 상태)**


| bit | 이름          | 의미 (1일 때)                                            |
| --- | ----------- | ---------------------------------------------------- |
| 0   | MOVING      | 모터가 아직 움직이거나 목표 속도가 0이 아님                            |
| 1   | ESTOP       | 비상정지 latch 중 (서버 ESTOP / 리모컨 POWER / UART timeout 등) |
| 2   | TIMEOUT     | SET_SPEED가 300ms 이상 안 와서 걸린 UART watchdog ESTOP      |
| 3   | NOZZLE      | 노즐(서보) ON                                            |
| 4   | RX_ERROR    | STM이 잘못된 UART 프레임을 받은 적 있음                           |
| 5   | TX_OVERFLOW | STATUS를 보내다 STM TX 큐가 넘친 적 있음                        |
| 6   | (사용 안 함)    | 항상 0                                                 |
| 7   | (사용 안 함)    | 항상 0                                                 |


## 예: flags == 0x09 → bit0+bit3 → MOVING + NOZZLE ON.

---

## 4. 기구 상수 (통일)


| 항목              | 값                     |
| --------------- | --------------------- |
| full step       | 200                   |
| microstep       | 16 (3200/rev)         |
| D               | 66 mm                 |
| W               | 166 mm                |
| steps_per_meter | ~15434                |
| MAX_SPS         | 4000                  |
| 0.05 m/s        | ~772 sps              |
| 이론 90도          | 2012 pulse (x K_slip) |


---

## 5. 제어 (로봇 RPi) _ 그냥 참고용

- 접근 감속, 횡오차/헤딩 P/PD, TURN 각도 PD
- 최종 출력: left_sps, right_sps
- 정본 위치: CCTV(+IMU). STEP은 보조

```
d, ey, e_theta, yaw_rate <- PATH + POSE + IMU
-> left_sps, right_sps
-> SET_SPEED(left_sps, right_sps)
```

---

## 6. 시나리오

1. PATH MOVE/TURN 수신
2. (필요시) CLEAR ESTOP
3. <=100ms SET_SPEED 루프 + STATUS
4. 꼭지점: (0,0) -> 계산 -> 다음 SPS
5. 완료: (0,0), NOZZLE OFF, 서버 DONE

---

## 7. 장애


| 상황                 | 처리                            |
| ------------------ | ----------------------------- |
| STATUS 유실          | (0,0), NOZZLE OFF, FAULT      |
| SET_SPEED 끊김(비제로중) | STM 300ms ESTOP               |
| CCTV 장기 유실         | 노즐 OFF, 정지, LOCALIZATION_LOST |
| 서버 ESTOP           | STM 0x03 즉시                   |


## 8. 체크리스트

- [ ] 기구 200x16 / 66mm / W=166
- [ ] POSE로 d/ey/e_theta + 감속/PD
- [ ] 출력 left_sps/right_sps만
- [ ] 주행중 SET_SPEED <=100ms, 꼭지점/DONE시 (0,0)
- [ ] ESTOP/CLEAR, 0x05 불필요
- [ ] STATUS flags + DONE/FAULT
- [ ] 주행중 리모컨 미사용

---





