#pragma once

#include <cstdint>

namespace hydrolib::crc
{
inline uint8_t CountCRC8(const uint8_t *data, unsigned length)
{
    uint8_t crc = 0; // Initial value 0x00
    for (unsigned i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x80)
            {
                // Polynomial 0x07
                crc = (uint8_t)((crc << 1) ^ 0x07);
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}
} // namespace hydrolib::crc
