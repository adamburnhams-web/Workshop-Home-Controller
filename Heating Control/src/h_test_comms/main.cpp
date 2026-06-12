// ============================================================
//  H comms test — standalone Mega 2560
//  Listens on RS485 for W→H packets using the same wiring as
//  the H controller (Serial1 D18/D19, DE on pin 31).
//  Prints every valid packet to USB serial with uptime stamp.
// ============================================================

#include <Arduino.h>
#include <avr/wdt.h>

#include "shared_types.h"
#include "rs485_packet.h"

#define PIN_RS485_DE  31

static PktReceiver pktRx;
static uint8_t     lastSeq    = 0;
static bool        firstPkt   = true;
static uint32_t    totalRx    = 0;
static uint32_t    totalMiss  = 0;

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
    Serial.print(t / 10); Serial.print('.'); Serial.print(t % 10);
}

static void printPacket(uint8_t seq, const WToHPacket &p) {
    printUptime();

    uint8_t skipped = firstPkt ? 0 : (uint8_t)(seq - lastSeq - 1);
    if (skipped) {
        char buf[16];
        snprintf(buf, sizeof(buf), " MISS+%u", skipped);
        Serial.print(buf);
        totalMiss += skipped;
    }
    lastSeq  = seq;
    firstPkt = false;
    totalRx++;

    Serial.print(F(" seq=")); Serial.print(seq);

    // Temperatures
    Serial.print(F(" SolH:"));   printTemp(p.tempSolarHot);
    Serial.print(F(" SolC:"));   printTemp(p.tempSolarCold);
    Serial.print(F(" UFH:"));    printTemp(p.tempUFHSupply);
    Serial.print(F(" WkAir:"));  printTemp(p.tempWorkshopAir);
    Serial.print(F(" Out:"));    printTemp(p.tempOutsideAir);

    // Growatt
    if (p.growattValid) {
        Serial.print(F(" PV:"));  Serial.print(p.pvOutputW); Serial.print('W');
        Serial.print(F(" Exp:")); Serial.print(p.pvExportW); Serial.print('W');
        Serial.print(F(" SOC:")); Serial.print(p.battSocPct); Serial.print('%');
    } else {
        Serial.print(F(" Growatt:NO"));
    }

    // Solar pump
    Serial.print(F(" Pump:"));
    if (p.solarPumpActive) { Serial.print(p.solarPumpDutyPct); Serial.print('%'); }
    else                     Serial.print(F("off"));

    // Faults
    if (p.wFaultFlags) {
        char fbuf[14];
        snprintf(fbuf, sizeof(fbuf), " WF:%08lX", (unsigned long)p.wFaultFlags);
        Serial.print(fbuf);
    }

    // Running miss ratio
    if (totalRx % 20 == 0) {
        char sbuf[32];
        snprintf(sbuf, sizeof(sbuf), " [rx=%lu miss=%lu]", totalRx, totalMiss);
        Serial.print(sbuf);
    }

    Serial.println();
}

void setup() {
    wdt_enable(WDTO_8S);
    Serial.begin(115200);
    Serial1.begin(9600);
    pinMode(PIN_RS485_DE, OUTPUT);
    digitalWrite(PIN_RS485_DE, LOW);   // always receive
    pktRx.reset();
    Serial.println(F("h_test_comms ready — listening for W packets"));
}

void loop() {
    wdt_reset();
    while (Serial1.available()) {
        uint8_t  outDir, outSeq;
        uint8_t *payload; uint16_t payLen;
        if (!pktRx.feed((uint8_t)Serial1.read(), outDir, outSeq, payload, payLen)) continue;
        if (outDir != PKT_DIR_WH || payLen != sizeof(WToHPacket)) continue;
        WToHPacket pkt;
        memcpy(&pkt, payload, sizeof(pkt));
        printPacket(outSeq, pkt);
    }
}
