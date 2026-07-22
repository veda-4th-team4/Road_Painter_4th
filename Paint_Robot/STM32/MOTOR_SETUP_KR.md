# 1. 역할

- left_sps/right_sps -> 가감속 STEP, 노즐, ESTOP, STATUS
- m/도/PATH/PD 안 함
- 물리 ESTOP 없음. ESTOP = 서버 UART + IR POWER
- AUTO/MANUAL 없음. ESTOP만 아니면 서버·IR 둘 다 가능
- IR: 키 1회 = 펄스 후 정지

---

## 2. 타이머 / IRQ


| 타이머     | 용도       | 설정                   |
| ------- | -------- | -------------------- |
| TIM1    | 서보 PA8   | 50 Hz                |
| TIM2    | STEP     | 20 kHz Motor_TickISR |
| TIM3    | IR PB4   | 1 us NEC             |
| TIM5    | HAL tick | 1 ms                 |
| SysTick | FreeRTOS | 1 kHz                |



| pri | IRQ                   |
| --- | --------------------- |
| 0   | TIM2                  |
| 6   | TIM3, USART1          |
| 7   | USART2                |
| 15  | SysTick, PendSV, TIM5 |


MAX_SYSCALL_INTERRUPT_PRIORITY=5. TIM2는 ISR 전용.

---



## 3. 제어 정책

1. 서버 ESTOP/CLEAR
2. IR POWER (REMOTE)
3. SET_SPEED/NOZZLE 또는 IR 방향·노즐


| SET_SPEED        | 동작                             |
| ---------------- | ------------------------------ |
| 좌·우 중 하나라도 0이 아님 | 주행, watchdog ON (300ms)        |
| (0,0)            | 정지, watchdog OFF. IR 펄스 중이면 무시 |
| 주행 중 선 끊김        | 300ms timeout ESTOP            |
| watchdog OFF     | 리모컨 OK                         |


작업 종료: 서버에 DONE, STM에는 (0,0).

 **ESTOP으로 끝내지 말 것**. 

꼭지점마다 (0,0) 후 계산 -> 다음 SPS: 문제 없음.

---

## 4. UART

- 115200-8-N-1, 3.3V
- PA9->RPi RXD(GPIO15), PA10<-RPi TXD(GPIO14), GND
- USART2: VCP / [IR ...]
- AA|LEN|CMD|PAYLOAD|CRC8|55, CRC=LEN+CMD+PAYLOAD, poly 0x07
- 블로킹 DONE/ERR 없음. STATUS로 판단


| CMD              | 방향       | LEN | 내용                    |
| ---------------- | -------- | --- | --------------------- |
| 0x01 SET_SPEED   | RPi->STM | 4   | int16 L/R sps,        |
| 0x02 NOZZLE      | RPi->STM | 1   | 0 OFF / 1 ON          |
| 0x03 ESTOP       | RPi->STM | 1   | reason (비상)           |
| 0x04 CLEAR_ESTOP | RPi->STM | 2   | 5A A5, 완전정지 후         |
| 0x81 STATUS      | STM->RPi | 9   | steps + flags, ~100ms |


예) +500,+500: AA 04 01 F4 01 F4 01 B1 55

### 0x81 STATUS PAYLOAD


| 바이트 | 타입        | 이름          | 의미         |
| --- | --------- | ----------- | ---------- |
| 0~3 | uint32 LE | left_steps  | 좌측 누적 STEP |
| 4~7 | uint32 LE | right_steps | 우측 누적 STEP |
| 8   | uint8     | flags       | 상태 비트      |


스텝: 전진 시 +1, 후진 시 -1(uint32 wrap).

 RPi는 int32로 보고 차분 사용.(= 출력 펄스 수. 실제 미끄러짐은 미포함)

스텝: 전진 시 +1, 후진 시 -1(uint32 wrap). RPi는 int32로 보고 차분 사용.  
(= 출력 펄스 수. 실제 미끄러짐은 미포함)


| bit | 이름          | 1이면                      |
| --- | ----------- | ------------------------ |
| 0   | MOVING      | 움직이는 중                   |
| 1   | ESTOP       | 비상정지 latch               |
| 2   | TIMEOUT     | SET_SPEED 300ms 끊김 ESTOP |
| 3   | NOZZLE      | 노즐 ON                    |
| 4   | RX_ERROR    | UART 수신 오류               |
| 5   | TX_OVERFLOW | STATUS 송신 큐 넘침           |
| 6~7 | -           | 사용 안 함(0)                |


---

## 5. IR

- PB4 TIM3_CH1, 3.3V. 키=ir_remote.h


| 키              | 값         | 동작              |
| -------------- | --------- | --------------- |
| POWER          | 0x28      | REMOTE ESTOP 토글 |
| UP/DOWN        | 0xC0/0x40 | 전/후 펄스          |
| LEFT/RIGHT     | 0x70/0x58 | 90도 펄스          |
| NOZZLE_UP/DOWN | 0xD0/0x1A | 노즐 OFF/ON       |


NEC repeat 무시. ESTOP중 방향 거부.

---

## 6. 핀


| 핀       | 용도            |
| ------- | ------------- |
| PB0/1/6 | L STEP/DIR/EN |
| PB2/5/7 | R STEP/DIR/EN |
| PA8     | 서보            |
| PB4     | IR            |
| PA9/10  | USART1        |
| PA2/3   | USART2        |


---

## 7. FreeRTOS

- 정적 할당, heap/timer 없음
- CommunicationTask: UART + STATUS 100ms
- ControlTask: 명령/IR/watchdog/펄스 종료
- ESTOP = queue front

---

## 8. DRV8825

- 1/16 (M0=L M1=L M2=H), 200 step/rev, D=66mm, W=166mm
- VMOT 100uF+, GND 공통, 서보 5V 별도

---

## 9. robot_config.h


| 항목                    | 의미                      |
| --------------------- | ----------------------- |
| FORWARD_LEVEL         | DIR 극성                  |
| ROBOT_MAX_SPS         | **4000**                |
| ACCEL / ESTOP_DECEL   | 가감속                     |
| UART_WATCHDOG_MS      | 300                     |
| MANUAL_DRIVE/TURN_SPS | IR 속도                   |
| IR_FWD/REV_STEPS      | IR 전후 펄스                |
| TURN_PULSE_THEORY_90  | **2012** 고정             |
| TURN_K_SLIP_MILLI_*   | 슬립 K (milli, +-10=0.01) |
| SERVO_OFF/ON_US       | 노즐                      |


steps/m ~15434. RPi와 통일.

---



## 10. 파일

`control_arbiter.*` `motor.*` `uart_protocol.*` `uart_transport.*` `ir_remote.*` `app_rtos.*` `robot_config.h` `Robot_Painter.ioc` `HARDWARE_WIRING_KR.md` `RPI_HANDOFF_FINAL_KR.md`