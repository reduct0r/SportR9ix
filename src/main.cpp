#include <Arduino.h>
#include "config.h"
#include "SportParser.h"
#include "PassthroughDecoder.h"
#include "MavlinkSender.h"
#include "SportDiag.h"

SportPort sportPort;
PassthroughDecoder decoder;
MavlinkSender mavlink;
SportDiag sportDiag;

TelemetryState telemetry;
uint32_t lastPollMs = 0;
bool sportPollActive = SPORT_ACTIVE_POLL;
uint32_t bootMs = 0;
uint32_t lastScanMs = 0;
bool startupMsgSent = false;

#if DEBUG_USB
uint32_t lastStatusMs = 0;
uint32_t lastRawBytes = 0;
#endif

void setup() {
    bootMs = millis();
    sportPort.begin(SPORT_PIN, SPORT_BAUD, SPORT_ACTIVE_POLL != 0);
    sportPollActive = SPORT_ACTIVE_POLL != 0;
    mavlink.begin();

#if DEBUG_USB
    Serial.println();
    Serial.println("SKAT SmartPort -> MAVLink bridge");
    Serial.printf("Sport GPIO%d invert=sw, poll=%s, WiFi=%s\n", SPORT_PIN,
                  SPORT_ACTIVE_POLL ? "on" : "off", WIFI_ENABLED ? "on" : "off");
#endif
}

void processSportPackets() {
    SportPacket packet;
    while (sportPort.readPacket(packet)) {
        sportDiag.notePacket(packet);
        decoder.handlePacket(packet, telemetry);
        telemetry.sportPackets++;
    }
}

void loop() {
    const uint32_t now = millis();

#if SPORT_AUTO_POLL_FALLBACK
    // Only when radio is OFF — polling with TX on the bus blocks listen while ER9X polls.
    if (!sportPollActive && sportPort.rawByteCount() < 10 && (now - bootMs) > 15000) {
        sportPollActive = true;
        sportPort.begin(SPORT_PIN, SPORT_BAUD, true);
        mavlink.sendStatusText("SKAT: poll mode (turn radio OFF!)");
    }
#endif

    if (sportPollActive && now - lastPollMs >= SPORT_POLL_INTERVAL_MS) {
        lastPollMs = now;
#if SPORT_OPENTX_ROTATE_POLL
        sportPort.pollOpenTxNext();
#else
        sportPort.pollSensor(SPORT_DEFAULT_SENSOR_ID);
#endif
        const uint32_t rxUntil = millis() + SPORT_POLL_RX_WINDOW_MS;
        while (millis() < rxUntil) {
            processSportPackets();
        }
    }

    if (!startupMsgSent && (now - bootMs) > 2000) {
        startupMsgSent = true;
#if MAVLINK_DIAG
#if SPORT_OPENTX_ROTATE_POLL && SPORT_ACTIVE_POLL
        mavlink.sendStatusText("SKAT: OpenTX rotate poll x28");
#elif SPORT_ACTIVE_POLL
        mavlink.sendStatusText("SKAT: poll 0x1B only");
#else
        mavlink.sendStatusText("SKAT: listen-only ER9X master");
#endif
#endif
    }

    processSportPackets();

#if SPORT_PERIODIC_SCAN
    if (!sportDiag.scanActive() && (now - bootMs) > SPORT_SCAN_BOOT_DELAY_MS &&
        (now - lastScanMs) > SPORT_SCAN_INTERVAL_MS) {
        lastScanMs = now;
        const bool wasPolling = sportPollActive;
        sportPort.begin(SPORT_PIN, SPORT_BAUD, true);
        sportPollActive = true;
        sportDiag.beginScan();
        mavlink.sendStatusText("SKAT: scan poll (radio OFF best)");
        (void)wasPolling;
    }

    if (sportDiag.scanActive()) {
        sportDiag.scanStep(sportPort, now);
    }

    if (sportDiag.scanReportPending()) {
        sportDiag.clearScanReportPending();
        char scanMsg[50];
        sportDiag.formatScanResult(scanMsg, sizeof(scanMsg), 27, sportDiag.scanResponseCount());
        mavlink.sendStatusText(scanMsg);
        if (!SPORT_ACTIVE_POLL) {
            sportPollActive = false;
            sportPort.begin(SPORT_PIN, SPORT_BAUD, false);
        }
    }
#endif

    telemetry.rawSportBytes = sportPort.rawByteCount();
    telemetry.sportCrcOk = sportPort.parserCrcOk();
    telemetry.sportCrcFail = sportPort.parserCrcFail();
    telemetry.sportPollActive = sportPollActive;
    mavlink.update(telemetry);

    static uint32_t lastIdsMs = 0;
#if MAVLINK_DIAG
    if (now - lastIdsMs >= 10000) {
        lastIdsMs = now;
        char idsMsg[50];
        sportDiag.formatSummary(idsMsg, sizeof(idsMsg));
        mavlink.sendStatusText(idsMsg);

        char pollMsg[50];
        snprintf(pollMsg, sizeof(pollMsg), "poll slot=%u wire=%02X f8=%lu f9=%lu",
                 static_cast<unsigned>(sportPort.currentOpenTxPollIndex()),
                 SportPort::openTxPollWireId(sportPort.currentOpenTxPollIndex()),
                 static_cast<unsigned long>(sportPort.parserFrames8()),
                 static_cast<unsigned long>(sportPort.parserFrames9()));
        mavlink.sendStatusText(pollMsg);

        sportDiag.resetWindow();
    }

#if SPORT_RAW_DIAG_INTERVAL_MS
    static uint32_t lastRawMs = 0;
    if (now - lastRawMs >= SPORT_RAW_DIAG_INTERVAL_MS) {
        lastRawMs = now;
        SportRawScan scan{};
        sportPort.scanRecentRaw(scan);
        char syncMsg[50];
        sportPort.formatSyncReport(syncMsg, sizeof(syncMsg), scan);
        mavlink.sendStatusText(syncMsg);

        char rawMsg[50];
        sportPort.formatRecentSampleHexAt(rawMsg, sizeof(rawMsg), 0, 14);
        mavlink.sendStatusText(rawMsg);
        sportPort.formatRecentSampleHexAt(rawMsg, sizeof(rawMsg), 14, 14);
        mavlink.sendStatusText(rawMsg);
        sportPort.formatRecentSampleHexAt(rawMsg, sizeof(rawMsg), 28, 14);
        mavlink.sendStatusText(rawMsg);
    }
#endif
#endif

#if DEBUG_USB
    if (now - lastStatusMs >= 2000) {
        lastStatusMs = now;
        const uint32_t raw = sportPort.rawByteCount();
        const uint32_t deltaRaw = raw - lastRawBytes;
        lastRawBytes = raw;
        Serial.printf("raw=%lu(+%lu) ok=%lu sport=%lu pass=%lu last=%lums\n", raw, deltaRaw,
                      sportPort.parserCrcOk(), telemetry.sportPackets, telemetry.passthroughPackets,
                      telemetry.lastUpdateMs ? now - telemetry.lastUpdateMs : 9999);
    }
#endif
}
