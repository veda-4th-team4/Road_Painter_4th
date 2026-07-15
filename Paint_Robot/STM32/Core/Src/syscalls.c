/**
 ******************************************************************************
 * @file    syscalls.c
 * @brief   Newlib nano의 미사용 POSIX I/O에 대한 최소 bare-metal stub
 * @details printf/파일 I/O를 구현하는 파일이 아닙니다. GCC 14 nosys.specs가
 *          발생시키는 미구현 system call 경고를 제거하기 위한 네 함수만
 *          제공합니다.
 ******************************************************************************
 */

#include <errno.h>

/**
 * @brief 파일 descriptor close 요청을 지원하지 않음으로 반환합니다.
 * @return 항상 -1입니다.
 */
int _close(int file) {
  (void)file;
  errno = ENOSYS;
  return -1;
}

/**
 * @brief seek 요청을 지원하지 않음으로 반환합니다.
 * @return 항상 -1입니다.
 */
int _lseek(int file, int offset, int whence) {
  (void)file;
  (void)offset;
  (void)whence;
  errno = ENOSYS;
  return -1;
}

/**
 * @brief 표준 입력 read 요청을 지원하지 않음으로 반환합니다.
 * @return 항상 -1입니다.
 */
int _read(int file, char *buffer, int length) {
  (void)file;
  (void)buffer;
  (void)length;
  errno = ENOSYS;
  return -1;
}

/**
 * @brief 표준 출력 write 요청을 지원하지 않음으로 반환합니다.
 * @return 항상 -1입니다.
 */
int _write(int file, char *buffer, int length) {
  (void)file;
  (void)buffer;
  (void)length;
  errno = ENOSYS;
  return -1;
}
