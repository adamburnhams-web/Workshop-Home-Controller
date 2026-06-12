// ============================================================
//  W Simulator — standalone Mega 2560
//  Sends a fixed neutral WToHPacket to H every 250ms over
//  RS485 (Serial1 D18/D19, DE on pin 40), same wiring as W.
//  Prints every valid H→W packet to USB serial with timestamp.
// ============================================================

#include <Arduino.h>
#include <avr/wdt.h>

#include "shared_types.h"
#include "rs485_packet.h"

#define PIN_RS485_DE_LINK    40
#define INTER_CTRL_POLL_MS   250UL
#define RS485_RX_TIMEOUT_MS  200UL

static uint8_t     txSeqNum = 0;
static PktReceiver pktRx;
static unsigned long lastTxMs = 0;

// All neutral values — nothing at H gets triggered.
// Solar: hot=15°C, below 18°C winter trigger.
// Workshop air: 20°C, above typical UFH stop temp (~13.5°C).
// Growatt: valid flag set, SOC=80%, no PV/import/export.
// Security: locked, door closed, no PIR.
// Winch: fully closed + manual lock reed flags set.
// ufhTargetReached=1 so H knows W-air is already at target.
// wFaultFlags=0, all pumps/valves off.
static void buildPacket(WToHPacket &pkt) {
    memset(&pkt, 0, sizeof(pkt));

    pkt.tempSolarHot    = 150;   // 15.0°C
    pkt.tempSolarCold   = 120;   // 12.0°C
    pkt.tempUFHSupply   = 200;   // 20.0°C
    pkt.tempUFHPostTMV  = 200;   // 20.0°C
    pkt.tempWorkshopAir = 200;   // 20.0°C
    pkt.tempOutsideAir  = 100;   // 10.0°C

    pkt.growattValid    = 1;
    pkt.battVoltage_dV  = 500;   // 50.0V
    pkt.battSocPct      = 80;

    pkt.workshopLocked  = 1;
    pkt.winchReedFlags  = WREED_FULLY_CLOSED | WREED_MANUAL_LOCK;
    pkt.ufhTargetReached = 1;
}

static void sendPacket() {
    WToHPacket pkt;
    buildPacket(pkt);

    uint8_t  frame[PKT_MAX_FRAME];
    uint16_t len = pktEncode(frame, sizeof(frame), PKT_DIR_WH,
                             txSeqNum++, &pkt, sizeof(pkt));

    digitalWrite(PIN_RS485_DE_LINK, HIGH);
    Serial1.write(frame, len);
    Serial1.flush();
    digitalWrite(PIN_RS485_DE_LINK, LOW);

    lastTxMs = millis();
}

// ============================================================
//  H PACKET PRINT
// ============================================================

static void printUptime() {
    unsigned long s = millis() / 1000UL;
    char buf[14];
    snprintf(buf, sizeof(buf), "[%03u:%02u:%02u]",
             (unsigned)(s / 3600UL),
             (unsigned)((s % 3600UL) / 60UL),
             (unsigned)(s % 60UL));
    Serial.print(buf);
}

static void printTemp(int16_t t) {
    if (t == TEMP_FAULT) { Serial.print(F("---")); return; }
    if (t < 0) { Serial.print('-'); t = -t; }
    Serial.print(t / 10);
    Serial.print('.');
    Serial.print(t % 10);
}

static void printHPacket(const HToWPacket &h) {
    printUptime();

    // Tank and pipe temperatures
    Serial.print(F(" Bot:"));    printTemp(h.tempTankBot);
    Serial.print(F(" Mid:"));    printTemp(h.tempTankMid);
    Serial.print(F(" Top:"));    printTemp(h.tempTankTop);
    Serial.print(F(" HPipe:"));  printTemp(h.tempHotPipe);
    Serial.print(F(" CPipe:"));  printTemp(h.tempColdPipe);
    Serial.print(F(" HtrOut:")); printTemp(h.tempHeaterOut);

    // Heater
    Serial.print(F(" Htr:"));    Serial.print(h.heaterPowerPct); Serial.print('%');
    if (h.heaterRestricted)      Serial.print(F(" RESTR"));

    // Mode and heating session
    Serial.print(F(" Mode:"));   Serial.print(h.systemMode == MODE_SUMMER ? F("SUM") : F("WIN"));
    Serial.print(F(" Boost:"));  Serial.print(h.boostMode);
    Serial.print(F(" Morn:"));   Serial.print(h.morningHeatActive);
    Serial.print(F(" UFHStop:")); printTemp(h.ufhStopTemp_dC);

    // Manual heater mode
    if (h.manualHeaterMode != MHM_OFF) {
        Serial.print(F(" ManHtr:"));
        Serial.print(h.manualHeaterMode == MHM_SOC_LIM ? F("SOC") : F("FORCE"));
    }

    // Fan
    Serial.print(F(" Fan:"));    Serial.print(h.fanBaseSpeedPct); Serial.print('%');

    // Two-port valve, pump duty
    Serial.print(F(" 2P:"));     Serial.print(h.twoPortHeaterSide);
    Serial.print(F(" HPump:"));  Serial.print(h.hPumpDutyPct); Serial.print('%');

    // Time sync (only show when H sends it)
    if (h.timeSyncValid) {
        char tbuf[22];
        snprintf(tbuf, sizeof(tbuf), " Time:20%02u-%02u-%02u %02u:%02u:%02u",
                 h.syncYear, h.syncMonth, h.syncDay,
                 h.syncHour, h.syncMinute, h.syncSecond);
        Serial.print(tbuf);
    }

    // Faults
    char fbuf[12];
    snprintf(fbuf, sizeof(fbuf), " Faults:%08lX", (unsigned long)h.hFaultFlags);
    Serial.print(fbuf);

    Serial.println();
}

static void receivePacket() {
    unsigned long deadline = millis() + RS485_RX_TIMEOUT_MS;
    uint8_t  outDir, outSeq;
    uint8_t *payload; uint16_t payLen;

    while (millis() < deadline) {
        if (!Serial1.available()) continue;
        if (pktRx.feed((uint8_t)Serial1.read(), outDir, outSeq, payload, payLen)) {
            if (outDir == PKT_DIR_HW && payLen == sizeof(HToWPacket)) {
                HToWPacket h;
                memcpy(&h, payload, sizeof(h));
                printHPacket(h);
            }
            return;
        }
    }
}

void setup() {
    wdt_enable(WDTO_8S);
    Serial.begin(115200);
    Serial1.begin(9600);
    pinMode(PIN_RS485_DE_LINK, OUTPUT);
    digitalWrite(PIN_RS485_DE_LINK, LOW);
    pktRx.reset();
    Serial.println(F("W simulator ready"));
}

void loop() {
    wdt_reset();
    if (millis() - lastTxMs >= INTER_CTRL_POLL_MS) {
        sendPacket();
        receivePacket();
        wdt_reset();
    }
}
