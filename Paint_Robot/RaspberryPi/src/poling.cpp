#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include <wiringPi.h>
#include <wiringSerial.h>

int main ()
{
  int fd;
  // 루프백 검증에 성공했던 물리 매핑 포트 경로 지정
  const char* device = "/dev/ttyAMA0";

  // 115200 Baudrate로 시리얼 포트 오픈
  if ((fd = serialOpen (device, 115200)) < 0)
  {
    fprintf (stderr, "시리얼 포트 오픈 실패 (%s): %s\n", device, strerror (errno));
    return 1;
  }

  if (wiringPiSetup () == -1)
  {
    fprintf (stdout, "wiringPi 초기화 실패: %s\n", strerror (errno));
    return 1;
  }

  printf("====================================================\n");
  printf(" STM32 (USART2) -> RPi4 (Physical 8,10) 수신 테스트\n");
  printf(" 포트 경로: %s | 보레이트: 115200 bps\n", device);
  printf("====================================================\n\n");

  while (true)
  {
    // 리눅스 커널 수신 버퍼에 데이터가 유입되었는지 체크
    while (serialDataAvail (fd))
    {
      int raw_data = serialGetchar (fd);
      
      if (raw_data != -1) {
        unsigned char rx_byte = (unsigned char)raw_data;
        // STM32에서 넘어온 카운트 값을 10진수 정수 포맷으로 출력
        printf ("[수신 성공] STM32 Data: %3d (0x%02X)\n", rx_byte, rx_byte);
        fflush (stdout);
      }
    }

    // CPU 과점유를 낮추기 위해 수신 루프 사이에 미세 딜레이(1밀리초) 부여
    delay (1);
  }

  serialClose (fd);
  return 0;
}