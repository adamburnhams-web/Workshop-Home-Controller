// ============================================================
//  Growatt Monitor — standalone Mega 2560
//  Same pins as W controller. Polls Growatt inverter over
//  Modbus RS485 (Serial2/D16-D17, DE on pin 47) and prints
//  all data to USB serial once per complete 4-phase read cycle.
// ============================================================

#include <Arduino.h>
#include <avr/wdt.h>

// UART2 (Serial2 D16/D17): Growatt Modbus RS485
#define PIN_RS485_DE_GROWATT 47

struct GrowattData {
    int16_t  pv1W;
    int16_t  pv2W;
    int16_t  pvOutputW;
    int16_t  loadW;
    int16_t  pvExportW;
    int16_t  gridImportW;
    int16_t  battVoltage_dV;
    uint8_t  battSocPct;
    int16_t  battChargeW;       // positive = charging, negative = discharging
    int16_t  dailyGenDeciKwh;
    unsigned long lastGoodMs;
    bool     valid;
} growatt;

static uint16_t sdmCRC16(const uint8_t* buf, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

static inline int16_t reg16(const uint8_t* data, uint8_t regOffset) {
    return (int16_t)(((uint16_t)data[regOffset * 2] << 8) | data[regOffset * 2 + 1]);
}

static void printUptime() {
    unsigned long s = millis() / 1000UL;
    uint8_t h   = (uint8_t)(s / 3600UL);
    uint8_t m   = (uint8_t)((s % 3600UL) / 60UL);
    uint8_t sec = (uint8_t)(s % 60UL);
    char buf[14];
    snprintf(buf, sizeof(buf), "[%03u:%02u:%02u]", h, m, sec);
    Serial.print(buf);
}

// Prints signed integer then unit string, no float needed
static void printW(int16_t w, const __FlashStringHelper* label) {
    Serial.print(label);
    Serial.print(w);
    Serial.print(F("W"));
}

// Prints ×0.1 value as X.X
static void printDeciVal(int16_t deci, const __FlashStringHelper* label, const __FlashStringHelper* unit) {
    Serial.print(label);
    if (deci < 0) { Serial.print('-'); deci = -deci; }
    Serial.print(deci / 10);
    Serial.print('.');
    Serial.print(deci % 10);
    Serial.print(unit);
}

static void printGrowattData() {
    printUptime();
    if (!growatt.valid) {
        Serial.println(F(" COMMS FAULT — data stale or never received"));
        return;
    }
    printW(growatt.pv1W,       F(" PV1:"));
    printW(growatt.pv2W,       F(" PV2:"));
    printW(growatt.pvExportW,  F(" Export:"));
    printW(growatt.gridImportW,F(" Import:"));
    printDeciVal(growatt.battVoltage_dV, F(" Batt:"), F("V"));
    Serial.print(F(" SOC:")); Serial.print(growatt.battSocPct); Serial.print('%');
    printW(growatt.battChargeW,F(" Chg:"));
    Serial.println();
}

// ============================================================
//  GROWATT PRODUCTION POLLING  (non-blocking state machine)
//  Phase 0: FC04 @0    x11 → pv1W (r6), pv2W (r10)
//  Phase 1: FC04 @35   x7  → pvOutputW (r36), loadW (r41)
//  Phase 2: FC04 @1000 x25 → batt, gridImport, pvExport
//  Phase 3: FC04 @55   x3  → dailyGenDeciKwh (r57)
//  Prints all data once when phase 3 completes.
// ============================================================

#define GROWATT_STALE_MS 60000UL

void growattPoll() {
    static uint8_t       phase      = 0;
    static bool          inRecv     = false;
    static unsigned long sentMs     = 0;
    static unsigned long lastByteMs = 0;
    static bool          rxStarted  = false;
    static uint8_t       rxBuf[96];
    static uint8_t       rxN        = 0;

    if (!inRecv) {
        { unsigned long t = millis(); while (millis() - t < 5) if (Serial2.available()) Serial2.read(); }

        static const uint16_t phaseReg[4]   = {    0,  35, 1000, 55 };
        static const uint16_t phaseCount[4] = {   11,   7,   25,  3 };
        uint16_t startReg = phaseReg[phase];
        uint16_t count    = phaseCount[phase];

        uint8_t req[8];
        req[0] = 0x01; req[1] = 0x04;
        req[2] = (uint8_t)(startReg >> 8); req[3] = (uint8_t)startReg;
        req[4] = (uint8_t)(count >> 8);    req[5] = (uint8_t)count;
        uint16_t crc = sdmCRC16(req, 6);
        req[6] = (uint8_t)crc; req[7] = (uint8_t)(crc >> 8);

        digitalWrite(PIN_RS485_DE_GROWATT, HIGH);
        delayMicroseconds(200);
        Serial2.write(req, 8); Serial2.flush();
        delayMicroseconds(200);
        digitalWrite(PIN_RS485_DE_GROWATT, LOW);

        sentMs     = millis();
        lastByteMs = millis();
        rxStarted  = false;
        rxN        = 0;
        inRecv     = true;
        return;
    }

    while (Serial2.available() && rxN < sizeof(rxBuf)) {
        uint8_t b = (uint8_t)Serial2.read();
        if (!rxStarted && b == 0x00) continue;
        rxStarted = true;
        rxBuf[rxN++] = b;
        lastByteMs = millis();
    }

    bool timedOut   = (millis() - sentMs > 120);
    bool packetDone = rxStarted && (millis() - lastByteMs >= 5);
    if (!timedOut && !packetDone) return;

    bool ok = false;
    if (rxN >= 5 && rxBuf[0] == 0x01 && !(rxBuf[1] & 0x80)) {
        uint16_t rxCrc = sdmCRC16(rxBuf, rxN - 2);
        if ((uint8_t)rxCrc == rxBuf[rxN-2] && (uint8_t)(rxCrc >> 8) == rxBuf[rxN-1]) {
            uint8_t* data = rxBuf + 3;
            switch (phase) {
                case 0:
                    growatt.pv1W = (int16_t)((uint16_t)reg16(data, 6) / 10);
                    growatt.pv2W = (int16_t)((uint16_t)reg16(data, 10) / 10);
                    ok = true; break;
                case 1:
                    growatt.pvOutputW = (int16_t)((uint16_t)reg16(data, 1) / 10);
                    growatt.loadW     = (int16_t)((uint16_t)reg16(data, 6) / 10);
                    ok = true; break;
                case 2:
                    growatt.battVoltage_dV = reg16(data, 13);
                    growatt.battSocPct     = (uint8_t)reg16(data, 14);
                    growatt.gridImportW    = (int16_t)((uint16_t)reg16(data, 16) / 10);
                    growatt.pvExportW      = (int16_t)((uint16_t)reg16(data, 24) / 10);
                    { int16_t dis = (int16_t)((uint16_t)reg16(data, 10) / 10);
                      int16_t chg = (int16_t)((uint16_t)reg16(data, 12) / 10);
                      growatt.battChargeW = chg - dis; }
                    ok = true; break;
                case 3:
                    growatt.dailyGenDeciKwh = reg16(data, 2);
                    ok = true; break;
            }
        }
    }

    if (ok) {
        growatt.lastGoodMs = millis();
        growatt.valid      = true;
    }

    if (growatt.valid && millis() - growatt.lastGoodMs > GROWATT_STALE_MS) {
        growatt.valid = false;
    }

    bool lastPhase = (phase == 3);
    if (++phase >= 4) phase = 0;
    inRecv = false;

    if (lastPhase) printGrowattData();
}

void setup() {
    wdt_enable(WDTO_8S);
    Serial.begin(115200);
    Serial2.begin(9600);
    pinMode(PIN_RS485_DE_GROWATT, OUTPUT);
    digitalWrite(PIN_RS485_DE_GROWATT, LOW);
    memset(&growatt, 0, sizeof(growatt));
    Serial.println(F("Growatt monitor started"));
}

void loop() {
    wdt_reset();
    growattPoll();
}
