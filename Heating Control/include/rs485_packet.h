#pragma once
#include <stdint.h>
#include <string.h>

// ============================================================
//  RS485 binary packet framing
//
//  Frame layout:
//    [0xAA] [0x55] [DIR] [SEQ] [LEN_LO] [LEN_HI] [PAYLOAD…] [CRC_LO] [CRC_HI]
//
//  DIR: 0x01 = W→H, 0x02 = H→W
//  SEQ: 0–255 wrapping sequence number (miss detection)
//  LEN: payload length in bytes (little-endian uint16_t)
//  CRC: CRC-16/Modbus over DIR+SEQ+LEN+PAYLOAD
// ============================================================

#define PKT_HEADER_A  0xAA
#define PKT_HEADER_B  0x55
#define PKT_DIR_WH    0x01
#define PKT_DIR_HW    0x02
#define PKT_OVERHEAD  8      // 2 header + 1 dir + 1 seq + 2 len + 2 CRC

// Maximum raw frame size (must hold largest packet struct + overhead)
#define PKT_MAX_FRAME 128

// CRC-16/Modbus (poly 0x8005, init 0xFFFF, no reflect override needed for AVR)
static inline uint16_t crc16Modbus(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x8005;
            else              crc <<= 1;
        }
    }
    return crc;
}

// Encode a packet into buf[]. Returns total frame length, or 0 on overflow.
static inline uint16_t pktEncode(uint8_t *buf, uint16_t bufSize,
                                  uint8_t dir, uint8_t seq,
                                  const void *payload, uint16_t payloadLen)
{
    uint16_t frameLen = PKT_OVERHEAD + payloadLen;
    if (frameLen > bufSize) return 0;

    buf[0] = PKT_HEADER_A;
    buf[1] = PKT_HEADER_B;
    buf[2] = dir;
    buf[3] = seq;
    buf[4] = (uint8_t)(payloadLen & 0xFF);
    buf[5] = (uint8_t)(payloadLen >> 8);
    memcpy(&buf[6], payload, payloadLen);

    uint16_t crc = crc16Modbus(&buf[2], 4 + payloadLen); // DIR..PAYLOAD
    buf[6 + payloadLen]     = (uint8_t)(crc & 0xFF);
    buf[6 + payloadLen + 1] = (uint8_t)(crc >> 8);

    return frameLen;
}

// Receive state machine — call pktFeed() with each incoming byte.
// When a valid, CRC-verified frame is complete, returns true and fills
// dir, seq, payload pointer, and payloadLen.  payloadBuf must be at
// least PKT_MAX_FRAME bytes; it is overwritten on each complete frame.

struct PktReceiver {
    uint8_t  buf[PKT_MAX_FRAME];
    uint16_t pos;
    uint16_t expectedLen; // full frame length once LEN is known

    void reset() { pos = 0; expectedLen = 0; }

    bool feed(uint8_t byte, uint8_t &outDir, uint8_t &outSeq,
              uint8_t *&outPayload, uint16_t &outPayloadLen)
    {
        // Sync on header pair
        if (pos == 0 && byte != PKT_HEADER_A) return false;
        if (pos == 1 && byte != PKT_HEADER_B) { pos = 0; return false; }

        if (pos < PKT_MAX_FRAME) buf[pos] = byte;
        pos++;

        // Once we have the LEN field (bytes 4-5), compute expected frame size
        if (pos == 6) {
            uint16_t payLen = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
            expectedLen = PKT_OVERHEAD + payLen;
            if (expectedLen > PKT_MAX_FRAME) { reset(); return false; }
        }

        if (pos < 6 || pos < expectedLen) return false;

        // Full frame received — verify CRC
        uint16_t payLen  = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
        uint16_t crcCalc = crc16Modbus(&buf[2], 4 + payLen);
        uint16_t crcRecv = (uint16_t)buf[6 + payLen] | ((uint16_t)buf[7 + payLen] << 8);

        bool valid = (crcCalc == crcRecv);
        if (valid) {
            outDir        = buf[2];
            outSeq        = buf[3];
            outPayload    = &buf[6];
            outPayloadLen = payLen;
        }
        reset();
        return valid;
    }
};
