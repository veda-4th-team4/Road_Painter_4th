#include "SerialManager.h"
#include <iostream>
#include <unistd.h>

int main() {
    // 통신 객체 인스턴스 생성
    SerialManager robot_comm("/dev/ttyAMA0", 115200);

    // 통신 채널 및 하드웨어 ALT 모드 개방
    if (!robot_comm.Init()) {
        std::cerr << "로봇 시스템 통신망 구축 실패. 프로세스를 종료합니다." << std::endl;
        return 1;
    }

    std::cout << "로봇 메인 시퀀스 콘트롤러 제어 루프 진입." << std::endl;

    // 시나리오 제어 시뮬레이션
    while (true) {
        // Scenario 1: 주행 속도 명령 전송 (좌: 500, 우: -500)
        std::cout << "\n[MAIN] 모터 지령 전달 시도..." << std::endl;
        robot_comm.SendSetSpeed(500, -500);
        sleep(2);

        // Scenario 2: 페인팅 노즐 분사 명령 전송 (ON)
        std::cout << "\n[MAIN] 노즐 분사 제어 전달 시도..." << std::endl;
        robot_comm.SendControlNozzle(1);
        sleep(2);

        // Scenario 3: 비상 정지 트리거 시뮬레이션
        std::cout << "\n[MAIN] 테스트 목적 비상 정지 송신..." << std::endl;
        robot_comm.SendEmergencyStop(0x01); // 0x01: S/W 비상 해제
        sleep(5);
    }

    robot_comm.Close();
    return 0;
}