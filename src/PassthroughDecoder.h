#pragma once

#include "SportParser.h"
#include "config.h"
#include <math.h>

struct TelemetryState {
    bool attitudeValid = false;
    bool gpsValid = false;
    bool batteryValid = false;
    bool velocityValid = false;
    bool apStatusValid = false;

    float rollRad = 0;
    float pitchRad = 0;
    float yawDeg = 0;
    float groundspeedMs = 0;
    float airspeedMs = 0;
    bool xyIsAirspeed = false;
    float climbMs = 0;
    float climbDerivedMs = 0;
    float prevHomeAltM = 0;
    uint32_t prevHomeAltMs = 0;
    float voltageV = 0;
    float currentA = 0;
    int32_t consumedMah = 0;
    int32_t battCapacityMah = 0;
    int32_t batt2CapacityMah = 0;
    float homeAltM = 0;
    float homeDistM = 0;
    float homeBearingDeg = 0;
    float rangefinderM = 0;
    float gpsHdop = 0;
    float terrainHeightM = 0;
    bool terrainUnhealthy = false;
    float trueWindSpeedMs = 0;
    float trueWindAngleDeg = 0;
    float apparentWindSpeedMs = 0;
    float apparentWindAngleDeg = 0;
    int32_t rpm1 = 0;
    int32_t rpm2 = 0;
    uint16_t wpNumber = 0;
    float wpDistanceM = 0;
    float wpBearingDeg = 0;
    int32_t frameType = -1;
    uint8_t simpleMode = 0;
    bool landComplete = false;
    bool battFailsafe = false;
    uint8_t ekfFailsafe = 0;
    bool failsafe = false;
    bool fencePresent = false;
    bool fenceBreached = false;
    float imuTempC = 0;
    char statusChunkBuf[52]{};
    uint8_t statusChunkLen = 0;
    bool statusPending = false;

    int32_t latE7 = 0;
    int32_t lonE7 = 0;
    int32_t altCm = 0;
    int32_t relativeAltCm = 0;
    uint8_t gpsFix = 0;
    uint8_t gpsSats = 0;
    uint8_t baseMode = 0;
    uint32_t customMode = 0;
    int16_t throttlePct = 0;
    uint32_t sportPackets = 0;
    uint32_t passthroughPackets = 0;
    uint32_t lastUpdateMs = 0;

    uint32_t cnt5000 = 0;
    uint32_t cnt5001 = 0;
    uint32_t cnt5005 = 0;
    uint32_t cnt5006 = 0;
    uint32_t cnt5002 = 0;
    uint32_t cnt5003 = 0;
    uint32_t cnt5004 = 0;
    uint32_t cnt5007 = 0;
    uint32_t cnt5009 = 0;
    uint32_t cnt500A = 0;
    uint32_t cnt500B = 0;
    uint32_t cnt500C = 0;
    uint32_t cnt0800 = 0;
    uint32_t last5006Ms = 0;
    uint32_t last5005Ms = 0;
    uint32_t last5003Ms = 0;
    uint32_t last5004Ms = 0;
    uint32_t last5002Ms = 0;
    uint32_t last5007Ms = 0;
    uint32_t last5009Ms = 0;
    uint32_t last5001Ms = 0;
    uint16_t lastAppId = 0;
    uint32_t cntUnknown = 0;
    uint32_t cnt50Other = 0;
    uint8_t lastPrimId = 0;

    uint32_t rawSportBytes = 0;
    uint32_t sportCrcOk = 0;
    uint32_t sportCrcFail = 0;
    bool sportPollActive = false;

    float displayAltM() const {
        if (last5004Ms != 0) {
            return homeAltM;
        }
        if (gpsFix >= 2 && altCm != 0) {
            return altCm * 0.01f;
        }
        return relativeAltCm * 0.01f;
    }

    float effectiveClimbMs() const {
        if (fabsf(climbMs) >= 0.05f) {
            return climbMs;
        }
        if (fabsf(climbDerivedMs) >= 0.02f) {
            return climbDerivedMs;
        }
        return climbMs;
    }

    float displayAirspeedMs() const {
        if (airspeedMs > 0.01f) {
            return airspeedMs;
        }
        return groundspeedMs;
    }

    int32_t effectiveCapacityMah() const {
#if BATT_CAPACITY_MAH
        if (battCapacityMah > 0) {
            return battCapacityMah;
        }
        return BATT_CAPACITY_MAH;
#else
        return battCapacityMah;
#endif
    }

    int8_t batteryRemainingPct() const {
        const int32_t cap = effectiveCapacityMah();
        if (cap <= 0) {
            return -1;
        }
        const int32_t used = consumedMah < 0 ? 0 : consumedMah;
        if (used >= cap) {
            return 0;
        }
        return static_cast<int8_t>(((cap - used) * 100) / cap);
    }
};

class PassthroughDecoder {
public:
    void handlePacket(const SportPacket &packet, TelemetryState &state);

private:
    static float decodeRollRad(uint32_t data);
    static float decodePitchRad(uint32_t data);
    static float decodeYawDeg(uint32_t data);
    static void decodeGpsCoord(uint32_t data, bool isLatitude, int32_t &latE7, int32_t &lonE7);
};
