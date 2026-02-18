#pragma once

#include <cstdint>

namespace hydrolib::bus::application {

// ──────────────────────────────────────────────────────────────────────────────
// Application-layer commands and data structures
// ──────────────────────────────────────────────────────────────────────────────

enum class Command : uint8_t
{
    WRITE,     // Master → Slave: write data to memory
    READ,      // Master → Slave: read data from memory
    RESPONSE,  // Slave → Master: successful read response
    ERROR      // Slave → Master: operation failed
};

// Payload structure for READ / WRITE commands
struct MemoryAccessInfo
{
    uint8_t address;   // starting address in slave memory
    uint8_t length;    // number of bytes to read/write
} __attribute__((__packed__));

// Full header sent over the wire (command + access info)
struct MemoryAccessHeader
{
    Command          command;
    MemoryAccessInfo info;
} __attribute__((__packed__));

// ──────────────────────────────────────────────────────────────────────────────
// Constants (used in slave/master implementations)
// ──────────────────────────────────────────────────────────────────────────────

constexpr unsigned kMaxDataLength  = UINT8_MAX;                           // theoretical max payload
constexpr unsigned kMaxMessageLength = sizeof(MemoryAccessHeader) + kMaxDataLength;

} // namespace hydrolib::bus::application