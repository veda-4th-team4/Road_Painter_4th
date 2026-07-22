#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>

int main() {
  std::cout << "[User Space Test] Opening /dev/imu..." << std::endl;

  // Open the character device for both reading and writing
  int fd = open("/dev/imu", O_RDWR);
  if (fd < 0) {
    std::cerr << "[User Space Test] Error: Failed to open /dev/imu. Have you "
                 "run 'sudo chmod 666 /dev/imu'?"
              << std::endl;
    return 1;
  }

  std::cout
      << "[User Space Test] /dev/imu opened successfully! Reading values..."
      << std::endl;
  std::cout << "Rotate the sensor. After 5 seconds, a reset write command (0 "
               "mdeg) will be simulated."
            << std::endl;
  std::cout
      << "------------------------------------------------------------------"
      << std::endl;

  int32_t yaw_mdeg = 0;
  int count = 0;
  bool reset_simulated = false;

  while (count < 250) { // Test run for ~5 seconds (250 * 20ms)
    // Seek to the beginning of the virtual file for clean read
    lseek(fd, 0, SEEK_SET);

    // Read 4-byte signed integer Yaw angle from driver (in millidegrees)
    ssize_t bytes_read = read(fd, &yaw_mdeg, sizeof(int32_t));
    if (bytes_read < 0) {
      std::cerr << "\n[User Space Test] Read error!" << std::endl;
      close(fd);
      return 1;
    }

    // Convert millidegree integer to float degree for visualization
    float yaw_deg = static_cast<float>(yaw_mdeg) * 0.001f;

    printf("\r[User Read] Raw Yaw: %8d mdeg | Deg Yaw: %7.2f deg (Read: %ld "
           "bytes)",
           yaw_mdeg, yaw_deg, bytes_read);
    fflush(stdout);

    count++;
    usleep(20000); // 20ms interval

    // Simulate external Vision reset at 100 loops (~2 seconds)
    if (count == 100 && !reset_simulated) {
      int32_t reset_angle = 0; // Write 0 mdeg (reset)
      std::cout << "\n[User Write] Simulating Vision Server Reset -> Writing 0 "
                   "mdeg to /dev/imu..."
                << std::endl;

      ssize_t bytes_written = write(fd, &reset_angle, sizeof(int32_t));
      if (bytes_written < 0) {
        std::cerr << "[User Write] Error writing reset command!" << std::endl;
      } else {
        std::cout << "[User Write] Successfully wrote " << bytes_written
                  << " bytes to reset driver." << std::endl;
      }
      reset_simulated = true;
    }
  }

  std::cout << "\n[User Space Test] Demo run complete. Closing fd."
            << std::endl;
  close(fd);
  return 0;
}
