#ifndef IMU_PACKET_HH
#define IMU_PACKET_HH

#include <cstdint>

struct IMUPacket {
    uint16_t accel_x;
    uint16_t accel_y;
    uint16_t accel_z;
    uint16_t checksum;

    IMUPacket() : accel_x(0), accel_y(0), accel_z(0), checksum(0) {}

    IMUPacket(uint16_t x, uint16_t y, uint16_t z, uint16_t checksum)
        : accel_x(x), accel_y(y), accel_z(z), checksum(checksum) {}
};

#endif // IMU_PACKET_HH