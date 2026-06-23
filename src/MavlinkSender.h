#pragma once

#include <Arduino.h>
#include "config.h"
#include "PassthroughDecoder.h"

#if WIFI_ENABLED
#include <WiFi.h>
#include <WiFiUdp.h>
#endif

class MavlinkSender {
public:
    void begin();
    void update(TelemetryState &state);
    void sendStatusText(const char *text);

private:
    static constexpr uint8_t MAVLINK_V2 = 0xFD;
    static constexpr uint8_t MAVLINK_V1 = 0xFE;
    static constexpr uint16_t MSG_HEARTBEAT = 0;
    static constexpr uint16_t MSG_SYS_STATUS = 1;
    static constexpr uint16_t MSG_PARAM_REQUEST_READ = 20;
    static constexpr uint16_t MSG_PARAM_REQUEST_LIST = 21;
    static constexpr uint16_t MSG_PARAM_VALUE = 22;
    static constexpr uint16_t MSG_MISSION_COUNT = 44;
    static constexpr uint16_t MSG_MISSION_REQUEST_LIST = 45;
    static constexpr uint16_t MSG_GPS_RAW_INT = 24;
    static constexpr uint16_t MSG_ATTITUDE = 30;
    static constexpr uint16_t MSG_GLOBAL_POSITION_INT = 33;
    static constexpr uint16_t MSG_MISSION_CURRENT = 42;
    static constexpr uint16_t MSG_VFR_HUD = 74;
    static constexpr uint16_t MSG_BATTERY_STATUS = 147;
    static constexpr uint16_t MSG_STATUSTEXT = 253;

    uint8_t seq_ = 0;
    uint32_t lastHeartbeatMs_ = 0;
    uint32_t lastAttitudeMs_ = 0;
    uint32_t lastPositionMs_ = 0;
    uint32_t lastHudMs_ = 0;
    uint32_t lastSysStatusMs_ = 0;
    uint32_t lastBatteryMs_ = 0;
    uint32_t lastMissionMs_ = 0;
    uint32_t lastStatusTextMs_ = 0;
    bool paramsSent_ = false;
    bool bootMsgSent_ = false;
    bool noPassthroughWarnSent_ = false;
    bool wifiBootMsgSent_ = false;
    uint32_t gcsMessages_ = 0;

    uint8_t rxBuf_[280]{};
    uint16_t rxLen_ = 0;

#if WIFI_ENABLED
    WiFiUDP udp_;
    IPAddress remoteIp_;
    uint16_t remotePort_ = 0;
    bool udpClientConnected_ = false;
    bool wifiParamsSent_ = false;
    void emitWifi(const uint8_t *frame, size_t len);
#endif

    void sendBuffer(const uint8_t *payload, uint8_t payloadLen, uint16_t msgId, uint8_t crcExtra);
    void sendHeartbeat(const TelemetryState &state);
    void sendAttitude(const TelemetryState &state);
    void sendGlobalPosition(const TelemetryState &state);
    void sendGpsRaw(const TelemetryState &state);
    void sendVfrHud(const TelemetryState &state);
    void sendSysStatus(const TelemetryState &state);
    void sendBatteryStatus(const TelemetryState &state);
    void sendMissionCurrent(const TelemetryState &state);
    void sendFcStatusText(const TelemetryState &state);
    void sendParamComplete();
    void sendMissionCountZero();
    void sendTelemetryStatus(const TelemetryState &state);
    void pollIncoming();
    void handleIncomingByte(uint8_t byte);
    void dispatchMessage(uint16_t msgId, const uint8_t *payload, uint8_t len);
    void emit(const uint8_t *frame, size_t len);
};
