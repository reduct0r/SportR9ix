#include "MavlinkSender.h"
#include <stdio.h>
#include <string.h>

namespace {
uint16_t crcAccumulate(uint8_t data, uint16_t crc) {
    uint8_t tmp = data ^ (crc & 0xFF);
    tmp ^= (tmp << 4);
    return (crc >> 8) ^ (static_cast<uint16_t>(tmp) << 8) ^ (static_cast<uint16_t>(tmp) << 3) ^
           (static_cast<uint16_t>(tmp) >> 4);
}

uint16_t crcMavlink(const uint8_t *data, uint8_t len, uint8_t crcExtra) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; ++i) {
        crc = crcAccumulate(data[i], crc);
    }
    crc = crcAccumulate(crcExtra, crc);
    return crc;
}

void putFloat(uint8_t *buf, float value) {
    memcpy(buf, &value, sizeof(float));
}

void putInt32(uint8_t *buf, int32_t value) {
    buf[0] = value & 0xFF;
    buf[1] = (value >> 8) & 0xFF;
    buf[2] = (value >> 16) & 0xFF;
    buf[3] = (value >> 24) & 0xFF;
}

void putUint32(uint8_t *buf, uint32_t value) {
    putInt32(buf, static_cast<int32_t>(value));
}

void putUint16(uint8_t *buf, uint16_t value) {
    buf[0] = value & 0xFF;
    buf[1] = (value >> 8) & 0xFF;
}

void putInt16(uint8_t *buf, int16_t value) {
    putUint16(buf, static_cast<uint16_t>(value));
}

uint8_t crcExtraForMessage(uint16_t msgId) {
    switch (msgId) {
    case 0:
        return 50;
    case 1:
        return 124;
    case 20:
        return 214; // PARAM_REQUEST_READ
    case 21:
        return 159; // PARAM_REQUEST_LIST
    case 22:
        return 220;
    case 24:
        return 24;
    case 30:
        return 39;
    case 33:
        return 104;
    case 42:
        return 141;
    case 44:
        return 221; // MISSION_COUNT
    case 45:
        return 132; // MISSION_REQUEST_LIST
    case 47:
        return 221; // MISSION_ACK
    case 66:
        return 148; // REQUEST_DATA_STREAM
    case 74:
        return 20;
    case 147:
        return 154; // BATTERY_STATUS
    case 76:
        return 152; // COMMAND_LONG
    case 253:
        return 83;
    default:
        return 0;
    }
}

bool isKnownGcsMessage(uint16_t msgId) {
    switch (msgId) {
    case 20:
    case 21:
    case 45:
    case 66:
    case 76:
        return true;
    default:
        return false;
    }
}

bool validateFrame(const uint8_t *frame, uint16_t msgId, uint8_t payloadLen) {
    const uint8_t crcExtra = crcExtraForMessage(msgId);
    if (crcExtra == 0 && !isKnownGcsMessage(msgId)) {
        return false;
    }
    if (crcExtra == 0) {
        // Known GCS message without crc in table — accept without CRC check.
        return true;
    }
    const uint16_t crc = crcMavlink(&frame[1], payloadLen + 9, crcExtra);
    const uint16_t recv = frame[10 + payloadLen] | (static_cast<uint16_t>(frame[11 + payloadLen]) << 8);
    return crc == recv;
}

#if WIFI_ENABLED
void sendUdpFrame(WiFiUDP &udp, const IPAddress &ip, uint16_t port, const uint8_t *frame, size_t len) {
    udp.beginPacket(ip, port);
    udp.write(frame, len);
    udp.endPacket();
}
#endif
} // namespace

void MavlinkSender::begin() {
    Serial.setTxBufferSize(1024);
    Serial.begin(MAVLINK_BAUD);

#if WIFI_ENABLED
    WiFi.mode(WIFI_AP);
    const IPAddress apIp(WIFI_AP_IP_OCTETS);
    WiFi.softAPConfig(apIp, apIp, IPAddress(255, 255, 255, 0));
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
    udp_.begin(WIFI_UDP_PORT);
#endif
}

void MavlinkSender::emit(const uint8_t *frame, size_t len) {
    Serial.write(frame, len);

#if WIFI_ENABLED
    // One destination only — dual send (bcast+unicast) caused out-of-order duplicates in MP.
    if (udpClientConnected_) {
        sendUdpFrame(udp_, remoteIp_, WIFI_UDP_PORT, frame, len);
    } else {
        sendUdpFrame(udp_, IPAddress(WIFI_AP_BCAST_OCTETS), WIFI_UDP_PORT, frame, len);
    }
#endif
}

void MavlinkSender::sendBuffer(const uint8_t *payload, uint8_t payloadLen, uint16_t msgId,
                               uint8_t crcExtra) {
    uint8_t frame[280];
    const uint8_t headerLen = 10;

    frame[0] = MAVLINK_V2;
    frame[1] = payloadLen;
    frame[2] = 0;
    frame[3] = 0;
    frame[4] = seq_++;
    frame[5] = MAVLINK_SYSTEM_ID;
    frame[6] = MAVLINK_COMPONENT_ID;
    frame[7] = msgId & 0xFF;
    frame[8] = (msgId >> 8) & 0xFF;
    frame[9] = (msgId >> 16) & 0xFF;
    memcpy(&frame[headerLen], payload, payloadLen);

    const uint16_t crc = crcMavlink(&frame[1], payloadLen + 9, crcExtra);
    frame[headerLen + payloadLen] = crc & 0xFF;
    frame[headerLen + payloadLen + 1] = (crc >> 8) & 0xFF;

    emit(frame, headerLen + payloadLen + 2);
}

void MavlinkSender::sendMissionCountZero() {
    uint8_t payload[4]{};
    sendBuffer(payload, sizeof(payload), MSG_MISSION_COUNT, 221);
}

void MavlinkSender::sendParamComplete() {
    uint8_t payload[25]{};
    putFloat(&payload[0], 1.0f);
    putUint16(&payload[4], 1);
    putUint16(&payload[6], 0);
    memcpy(&payload[8], "SKAT_TELEM", 10);
    payload[24] = 9; // MAV_PARAM_TYPE_REAL32
    sendBuffer(payload, sizeof(payload), MSG_PARAM_VALUE, 220);
    paramsSent_ = true;
}

void MavlinkSender::sendStatusText(const char *text) {
    uint8_t payload[51]{};
    payload[0] = 6; // MAV_SEVERITY_INFO
    strncpy(reinterpret_cast<char *>(&payload[1]), text, 50);
    sendBuffer(payload, sizeof(payload), MSG_STATUSTEXT, 83);
}

void MavlinkSender::dispatchMessage(uint16_t msgId, const uint8_t *payload, uint8_t len) {
    (void)payload;
    (void)len;
    gcsMessages_++;

    switch (msgId) {
    case MSG_PARAM_REQUEST_LIST:
    case MSG_PARAM_REQUEST_READ:
        sendParamComplete();
        sendStatusText("SKAT: params sent (telemetry bridge)");
        break;
    case MSG_MISSION_REQUEST_LIST:
        sendMissionCountZero();
        sendStatusText("SKAT: no mission (telemetry bridge)");
        break;
    default:
        break;
    }
}

void MavlinkSender::handleIncomingByte(uint8_t byte) {
    if (rxLen_ >= sizeof(rxBuf_)) {
        rxLen_ = 0;
    }
    rxBuf_[rxLen_++] = byte;

    if (rxLen_ < 2) {
        return;
    }

    if (rxBuf_[0] == MAVLINK_V2) {
        const uint8_t payloadLen = rxBuf_[1];
        const uint16_t totalLen = 12 + payloadLen;
        if (rxLen_ < totalLen) {
            return;
        }
        const uint16_t msgId = rxBuf_[7] | (static_cast<uint16_t>(rxBuf_[8]) << 8) |
                               (static_cast<uint16_t>(rxBuf_[9]) << 16);
        if (validateFrame(rxBuf_, msgId, payloadLen)) {
            dispatchMessage(msgId, &rxBuf_[10], payloadLen);
        }
        rxLen_ = 0;
        return;
    }

    if (rxBuf_[0] == MAVLINK_V1) {
        const uint8_t payloadLen = rxBuf_[1];
        const uint16_t totalLen = 8 + payloadLen;
        if (rxLen_ < totalLen) {
            return;
        }
        const uint16_t msgId = rxBuf_[5];
        const uint8_t crcExtra = crcExtraForMessage(msgId);
        const uint16_t crc = crcMavlink(&rxBuf_[1], payloadLen + 5, crcExtra);
        const uint16_t recv = rxBuf_[6 + payloadLen] | (static_cast<uint16_t>(rxBuf_[7 + payloadLen]) << 8);
        if (crc == recv) {
            dispatchMessage(msgId, &rxBuf_[6], payloadLen);
        }
        rxLen_ = 0;
        return;
    }

    rxLen_ = 0;
}

void MavlinkSender::pollIncoming() {
    while (Serial.available()) {
        handleIncomingByte(static_cast<uint8_t>(Serial.read()));
    }

#if WIFI_ENABLED
    const int packetSize = udp_.parsePacket();
    if (packetSize > 0) {
        remoteIp_ = udp_.remoteIP();
        remotePort_ = udp_.remotePort();
        udpClientConnected_ = true;
        while (udp_.available()) {
            handleIncomingByte(static_cast<uint8_t>(udp_.read()));
        }
    }
#endif
}

void MavlinkSender::sendHeartbeat(const TelemetryState &state) {
    uint8_t payload[9]{};
    putUint32(&payload[0], state.customMode);
    payload[4] = 2;  // MAV_TYPE_QUADROTOR — MP HUD expects vehicle type
    payload[5] = 3;  // MAV_AUTOPILOT_ARDUPILOTMEGA
    payload[6] = state.baseMode; // armed bit from passthrough 0x5001 (display only)
    payload[7] = 4;  // MAV_STATE_ACTIVE
    payload[8] = 3;
    sendBuffer(payload, sizeof(payload), MSG_HEARTBEAT, 50);
}

void MavlinkSender::sendTelemetryStatus(const TelemetryState &state) {
    char text[50];
    snprintf(text, sizeof(text), "%s raw=%lu ok=%lu fl=%lu pt=%lu 6=%lu id=%04X",
             state.sportPollActive ? "P" : "L",
             static_cast<unsigned long>(state.rawSportBytes),
             static_cast<unsigned long>(state.sportCrcOk),
             static_cast<unsigned long>(state.sportCrcFail),
             static_cast<unsigned long>(state.passthroughPackets),
             static_cast<unsigned long>(state.cnt5006),
             state.lastAppId);
    sendStatusText(text);
}

void MavlinkSender::sendAttitude(const TelemetryState &state) {
    uint8_t payload[28]{};
    putUint32(&payload[0], millis());
    putFloat(&payload[4], state.rollRad);
    putFloat(&payload[8], state.pitchRad);
    putFloat(&payload[12], state.yawDeg * 0.0174532925f);
    sendBuffer(payload, sizeof(payload), MSG_ATTITUDE, 39);
}

void MavlinkSender::sendGlobalPosition(const TelemetryState &state) {
    uint8_t payload[28]{};
    putUint32(&payload[0], millis());
    putInt32(&payload[4], state.latE7);
    putInt32(&payload[8], state.lonE7);
    const int32_t amslMm =
        state.altCm != 0 ? state.altCm * 10 : static_cast<int32_t>(state.homeAltM * 1000.0f);
    putInt32(&payload[12], amslMm);
    putInt32(&payload[16], state.relativeAltCm * 10);
    putInt16(&payload[20], static_cast<int16_t>(state.groundspeedMs * 100));
    putInt16(&payload[22], 0);
    const float climb = state.effectiveClimbMs();
    putInt16(&payload[24], static_cast<int16_t>(climb * 100));
    putUint16(&payload[26], static_cast<uint16_t>(state.yawDeg * 100));
    sendBuffer(payload, sizeof(payload), MSG_GLOBAL_POSITION_INT, 104);
}

void MavlinkSender::sendGpsRaw(const TelemetryState &state) {
    uint8_t payload[30]{};
    putInt32(&payload[8], state.latE7);
    putInt32(&payload[12], state.lonE7);
    putInt32(&payload[16], state.altCm * 10);
    payload[28] = state.gpsFix >= 3 ? 3 : state.gpsFix;
    payload[29] = state.gpsSats;
    sendBuffer(payload, sizeof(payload), MSG_GPS_RAW_INT, 24);
}

void MavlinkSender::sendVfrHud(const TelemetryState &state) {
    uint8_t payload[20]{};
    const float climb = state.effectiveClimbMs();
    putFloat(&payload[0], state.displayAirspeedMs());
    putFloat(&payload[4], state.groundspeedMs);
    putInt16(&payload[8], static_cast<int16_t>(state.yawDeg));
    putUint16(&payload[10], static_cast<uint16_t>(state.throttlePct));
    putFloat(&payload[12], state.displayAltM());
    putFloat(&payload[16], climb);
    sendBuffer(payload, sizeof(payload), MSG_VFR_HUD, 20);
}

void MavlinkSender::sendSysStatus(const TelemetryState &state) {
    uint8_t payload[31]{};
    constexpr uint32_t kBattSensor = 262144;
    putUint32(&payload[0], kBattSensor);
    putUint32(&payload[4], kBattSensor);
    putUint32(&payload[8], kBattSensor);
    putUint16(&payload[14], static_cast<uint16_t>(state.voltageV * 1000));
    const int16_t currentCa =
        state.currentA != 0.0f ? static_cast<int16_t>(state.currentA * 100) : 0;
    putInt16(&payload[16], currentCa);
    const int8_t remain = state.batteryRemainingPct();
    payload[18] = remain >= 0 ? static_cast<uint8_t>(remain) : 0xFF;
    sendBuffer(payload, sizeof(payload), MSG_SYS_STATUS, 124);
}

void MavlinkSender::sendBatteryStatus(const TelemetryState &state) {
    // MAVLink BATTERY_STATUS wire layout (MIN_LEN 36):
    // 0: current_consumed, 4: energy_consumed, 8: temperature,
    // 10: voltages[10], 30: current_battery, 32: id, 35: battery_remaining
    uint8_t payload[36]{};
    putInt32(&payload[0], state.consumedMah > 0 ? state.consumedMah : -1);
    putInt32(&payload[4], -1);
    putInt16(&payload[8], 32767); // INT16_MAX = temperature unknown
    putUint16(&payload[10], static_cast<uint16_t>(state.voltageV * 1000));
    for (int i = 1; i < 10; ++i) {
        putUint16(&payload[10 + i * 2], 0xFFFF);
    }
    const int16_t currentCa =
        state.currentA != 0.0f ? static_cast<int16_t>(state.currentA * 100) : -1;
    putInt16(&payload[30], currentCa);
    payload[32] = 0;
    const int8_t remain = state.batteryRemainingPct();
    payload[35] = remain >= 0 ? static_cast<uint8_t>(remain) : static_cast<uint8_t>(-1);
    sendBuffer(payload, sizeof(payload), MSG_BATTERY_STATUS, 154);
}

void MavlinkSender::sendMissionCurrent(const TelemetryState &state) {
    uint8_t payload[18]{};
    putUint16(&payload[0], state.wpNumber);
    sendBuffer(payload, sizeof(payload), MSG_MISSION_CURRENT, 141);
}

void MavlinkSender::sendFcStatusText(const TelemetryState &state) {
    if (!state.statusPending || state.statusChunkBuf[0] == '\0') {
        return;
    }
    uint8_t payload[51]{};
    payload[0] = 6;
    strncpy(reinterpret_cast<char *>(&payload[1]), state.statusChunkBuf, 50);
    sendBuffer(payload, sizeof(payload), MSG_STATUSTEXT, 83);
}

void MavlinkSender::update(TelemetryState &state) {
    pollIncoming();

    const uint32_t now = millis();
    const bool attRecent = state.last5006Ms != 0 && (now - state.last5006Ms) < 5000;
    const bool velRecent = state.last5005Ms != 0 && (now - state.last5005Ms) < 5000;
    const bool battRecent = state.last5003Ms != 0 && (now - state.last5003Ms) < 5000;
    const bool altRecent = (state.last5004Ms != 0 && (now - state.last5004Ms) < 5000) ||
                           (state.last5002Ms != 0 && (now - state.last5002Ms) < 5000);
    const bool apRecent = state.last5001Ms != 0 && (now - state.last5001Ms) < 5000;
    const bool anyRecent = state.lastUpdateMs != 0 && (now - state.lastUpdateMs) < 5000;
    const bool hasAltData = altRecent || state.gpsValid;

    if (gcsMessages_ > 0 && !paramsSent_ && now > 3000) {
        sendParamComplete();
        sendStatusText("SKAT: params sent");
    }

    if (!bootMsgSent_ && now > 2000) {
#if MAVLINK_DIAG
        sendStatusText("SKAT: read-only telem, no arm cmd");
#else
        sendStatusText("SKAT telem bridge ready");
#endif
        bootMsgSent_ = true;
    }

#if WIFI_ENABLED
    if (!wifiBootMsgSent_ && now > 2500) {
        sendStatusText("SKAT WiFi: SKAT-TELEM UDP 14550");
        wifiBootMsgSent_ = true;
    }
#endif

    if (!noPassthroughWarnSent_ && now > 15000 && state.sportCrcOk > 50 && !anyRecent) {
        sendStatusText("SKAT: no FC passthrough - aircraft+RF?");
        noPassthroughWarnSent_ = true;
    } else if (noPassthroughWarnSent_ && anyRecent) {
        noPassthroughWarnSent_ = false;
    }

#if MAVLINK_DIAG
    if (now - lastStatusTextMs_ >= 5000) {
        lastStatusTextMs_ = now;
        sendTelemetryStatus(state);
    }
#endif

    if (now - lastHeartbeatMs_ >= 1000) {
        sendHeartbeat(state);
        lastHeartbeatMs_ = now;
    }

    // Attitude: needs 0x5006 for roll/pitch; yaw can come from 0x5005
    if (anyRecent && (attRecent || velRecent) && now - lastAttitudeMs_ >= 50) {
        sendAttitude(state);
        lastAttitudeMs_ = now;
    }

    if (anyRecent && hasAltData && now - lastPositionMs_ >= 200) {
        sendGlobalPosition(state);
        if (state.gpsValid) {
            sendGpsRaw(state);
        }
        lastPositionMs_ = now;
    }

    if (anyRecent && (velRecent || attRecent || altRecent) && now - lastHudMs_ >= 100) {
        sendVfrHud(state);
        lastHudMs_ = now;
    }

    if (state.statusPending) {
        sendFcStatusText(state);
        state.statusPending = false;
    }

    if (anyRecent && state.last5009Ms != 0 && (now - state.last5009Ms) < 5000 &&
        now - lastMissionMs_ >= 1000) {
        sendMissionCurrent(state);
        lastMissionMs_ = now;
    }

    if (anyRecent && (state.batteryValid || battRecent) && now - lastSysStatusMs_ >= 500) {
        sendSysStatus(state);
        lastSysStatusMs_ = now;
    }

    if (anyRecent && (state.batteryValid || battRecent) && now - lastBatteryMs_ >= 500) {
        sendBatteryStatus(state);
        lastBatteryMs_ = now;
    }

    (void)apRecent;
}
