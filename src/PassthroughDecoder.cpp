#include "PassthroughDecoder.h"
#include "config.h"
#include <math.h>

namespace {
constexpr float kDegToRad = 0.0174532925f;

uint16_t extractBits(uint32_t value, uint8_t offset, uint16_t mask) {
    return static_cast<uint16_t>((value >> offset) & mask);
}

float pow10u(uint8_t exp) {
    switch (exp) {
    case 0:
        return 1.0f;
    case 1:
        return 10.0f;
    case 2:
        return 100.0f;
    case 3:
        return 1000.0f;
    default:
        return 1000.0f;
    }
}

// Inverse of AP_Frsky_SPort::prep_number(number, 2, 1) — vertical speed, currents, speeds.
float decodePrepNumber21Signed(uint32_t data, uint8_t offset) {
    const uint16_t field = static_cast<uint16_t>((data >> offset) & 0x1FF);
    const bool negative = (field & 0x100) != 0;
    const bool exp = field & 0x1;
    const uint8_t digits = static_cast<uint8_t>((field >> 1) & 0x7F);
    float value = static_cast<float>(digits) * (exp ? 10.0f : 1.0f) * 0.1f;
    return negative ? -value : value;
}

// Inverse of prep_number(number, 3, 2) — home altitude, terrain height.
float decodePrepNumber32Signed(uint32_t data, uint8_t offset) {
    const uint16_t field = static_cast<uint16_t>((data >> offset) & 0xFFF);
    const bool negative = (field & 0x800) != 0;
    const uint8_t exp = field & 0x3;
    const uint16_t digits = static_cast<uint16_t>((field >> 2) & 0x3FF);
    float value = static_cast<float>(digits) * pow10u(exp) * 0.1f;
    return negative ? -value : value;
}

void noteHomeAltSample(TelemetryState &state, float relAltM, uint32_t nowMs) {
    state.homeAltM = relAltM;
    state.relativeAltCm = static_cast<int32_t>(relAltM * 100.0f);
    if (state.prevHomeAltMs != 0 && nowMs > state.prevHomeAltMs) {
        const float dt = (nowMs - state.prevHomeAltMs) / 1000.0f;
        if (dt >= 0.08f && dt <= 5.0f) {
            state.climbDerivedMs = (relAltM - state.prevHomeAltM) / dt;
        }
    }
    state.prevHomeAltM = relAltM;
    state.prevHomeAltMs = nowMs;
}

void ingestStatusTextChunk(TelemetryState &state, uint32_t data) {
    if (state.statusChunkLen >= sizeof(state.statusChunkBuf) - 1) {
        state.statusChunkLen = 0;
    }
    for (int i = 3; i >= 0; --i) {
        const char ch = static_cast<char>((data >> (i * 8)) & 0xFF);
        if (ch == '\0') {
            if (state.statusChunkLen > 0) {
                state.statusChunkBuf[state.statusChunkLen] = '\0';
                state.statusPending = true;
                state.statusChunkLen = 0;
            }
            return;
        }
        if (state.statusChunkLen < sizeof(state.statusChunkBuf) - 1) {
            state.statusChunkBuf[state.statusChunkLen++] = ch;
        }
    }
    const bool endMarker =
        ((data >> 21) & 0x4) || ((data >> 14) & 0x2) || ((data >> 7) & 0x1);
    if (endMarker && state.statusChunkLen > 0) {
        state.statusChunkBuf[state.statusChunkLen] = '\0';
        state.statusPending = true;
        state.statusChunkLen = 0;
    }
}
} // namespace

float PassthroughDecoder::decodeRollRad(uint32_t data) {
    const int16_t rollCdeg = static_cast<int16_t>(extractBits(data, 0, 0x7FF) * 20 - 18000);
    return rollCdeg * 0.01f * kDegToRad;
}

float PassthroughDecoder::decodePitchRad(uint32_t data) {
    const int16_t pitchCdeg = static_cast<int16_t>(extractBits(data, 11, 0x3FF) * 20 - 9000);
    return pitchCdeg * 0.01f * kDegToRad;
}

float PassthroughDecoder::decodeYawDeg(uint32_t data) {
    return extractBits(data, 17, 0x7FF) * 0.2f;
}

void PassthroughDecoder::decodeGpsCoord(uint32_t data, bool isLatitude, int32_t &latE7,
                                        int32_t &lonE7) {
    const uint32_t raw = data & 0x3FFFFFFF;
    int32_t coord = static_cast<int32_t>((raw / 6) * 100);

    if (isLatitude) {
        if (data & 0x40000000) {
            coord = -coord;
        }
        latE7 = coord * 1000;
    } else {
        if (data & 0xC0000000) {
            coord = -coord;
        }
        lonE7 = coord * 1000;
    }
}

void PassthroughDecoder::handlePacket(const SportPacket &packet, TelemetryState &state) {
    if (packet.frame != 0x10 && packet.frame != 0x32) {
        return;
    }

    const uint32_t now = millis();
    state.lastUpdateMs = now;
    state.lastAppId = packet.appId;
    state.lastPrimId = packet.frame;

    switch (packet.appId) {
    case 0x5000: {
        ingestStatusTextChunk(state, packet.data);
        state.cnt5000++;
        state.passthroughPackets++;
        break;
    }
    case 0x5006: {
        state.rollRad = decodeRollRad(packet.data);
        state.pitchRad = decodePitchRad(packet.data);
        const int rangeExp = (packet.data >> 21) & 0x1;
        const int rangeMantissa = (packet.data >> 22) & 0x3FF;
        state.rangefinderM = static_cast<float>(rangeMantissa) * pow10u(rangeExp) / 100.0f;
        state.attitudeValid = true;
        state.cnt5006++;
        state.last5006Ms = now;
        state.passthroughPackets++;
        break;
    }
    case 0x5005: {
        state.climbMs = decodePrepNumber21Signed(packet.data, 0);
        const float xySpeed = decodePrepNumber21Signed(packet.data, 9);
        state.xyIsAirspeed = (packet.data & (1U << 28)) != 0;
        if (state.xyIsAirspeed) {
            state.airspeedMs = xySpeed;
        } else {
            state.groundspeedMs = xySpeed;
            state.airspeedMs = xySpeed;
        }
        state.yawDeg = decodeYawDeg(packet.data);
        state.velocityValid = true;
        state.cnt5005++;
        state.last5005Ms = now;
        state.passthroughPackets++;
        break;
    }
    case 0x5003:
    case 0x5008: {
        const float volts = (packet.data & 0x1FF) * 0.1f;
        const float amps = decodePrepNumber21Signed(packet.data, 9);
        const int32_t mah = static_cast<int32_t>((packet.data >> 17) & 0x7FFF);
        if (volts > 0.5f) {
            state.voltageV = volts;
            state.currentA = amps;
            state.consumedMah = mah;
            state.batteryValid = true;
            state.cnt5003++;
            state.last5003Ms = now;
            state.passthroughPackets++;
        }
        break;
    }
    case 0x5002: {
        state.gpsSats = packet.data & 0xF;
        state.gpsFix = static_cast<uint8_t>(((packet.data >> 4) & 0x3) + ((packet.data >> 14) & 0x3));
        const int hdopMantissa = (packet.data >> 7) & 0x7F;
        const int hdopExp = (packet.data >> 6) & 0x1;
        state.gpsHdop = static_cast<float>(hdopMantissa) * pow10u(hdopExp);
        const int altMantissa = (packet.data >> 24) & 0x7F;
        const int altExp = (packet.data >> 22) & 0x3;
        const int altSign = (packet.data >> 31) & 0x1;
        const float gpsAltM =
            (altSign ? -1.0f : 1.0f) * static_cast<float>(altMantissa) * pow10u(altExp) / 10.0f;
        state.altCm = static_cast<int32_t>(gpsAltM * 100.0f);
        if (state.gpsFix >= 2) {
            state.gpsValid = true;
        }
        state.cnt5002++;
        state.last5002Ms = now;
        state.passthroughPackets++;
        break;
    }
    case 0x5004: {
        const int distExp = packet.data & 0x3;
        const int distMantissa = (packet.data >> 2) & 0x3FF;
        state.homeDistM = static_cast<float>(distMantissa) * pow10u(distExp);
        const int homeAltSign = (packet.data >> 24) & 0x1;
        const float relAltM =
            (homeAltSign ? -1.0f : 1.0f) * decodePrepNumber32Signed(packet.data, 12);
        state.homeBearingDeg = static_cast<float>(((packet.data >> 25) & 0x7F) * 3);
        noteHomeAltSample(state, relAltM, now);
        state.cnt5004++;
        state.last5004Ms = now;
        state.passthroughPackets++;
        break;
    }
    case 0x5001: {
        state.customMode = packet.data & 0x1F;
        state.simpleMode = static_cast<uint8_t>((packet.data >> 5) & 0x3);
        state.landComplete = ((packet.data >> 7) & 0x1) != 0;
        const bool armed = (packet.data >> 8) & 0x1;
        state.baseMode = armed ? 0x80 : 0;
        state.battFailsafe = ((packet.data >> 9) & 0x1) != 0;
        state.ekfFailsafe = static_cast<uint8_t>((packet.data >> 10) & 0x3);
        state.failsafe = ((packet.data >> 12) & 0x1) != 0;
        state.fencePresent = ((packet.data >> 13) & 0x1) != 0;
        state.fenceBreached = state.fencePresent ? (((packet.data >> 14) & 0x1) != 0) : false;
        const int throttleRaw = (packet.data >> 19) & 0x3F;
        const int throttleSign = (packet.data >> 25) & 0x1;
        state.throttlePct = static_cast<uint16_t>(
            (throttleSign ? -1 : 1) * throttleRaw * 100 / 63);
        state.imuTempC = static_cast<float>(((packet.data >> 26) & 0x3F) + 19);
        state.apStatusValid = true;
        state.cnt5001++;
        state.last5001Ms = now;
        state.passthroughPackets++;
        break;
    }
    case 0x5007: {
        const uint8_t paramId = static_cast<uint8_t>((packet.data >> 24) & 0xFF);
        const int32_t paramValue = static_cast<int32_t>(packet.data & 0xFFFFFF);
        if (paramId == 1) {
            state.frameType = paramValue;
        } else if (paramId == 4 && paramValue > 0) {
            state.battCapacityMah = paramValue;
            state.last5007Ms = now;
        } else if (paramId == 5 && paramValue > 0) {
            state.batt2CapacityMah = paramValue;
        }
        state.cnt5007++;
        state.passthroughPackets++;
        break;
    }
    case 0x5009:
    case 0x500D: {
        state.wpNumber = static_cast<uint16_t>(packet.data & 0x7FF);
        state.wpDistanceM =
            static_cast<float>(((packet.data >> 11) & 0x3FF) * pow10u((packet.data >> 10) & 0x1));
        state.wpBearingDeg = static_cast<float>(((packet.data >> 23) & 0xFF) * 3);
        state.cnt5009++;
        state.last5009Ms = now;
        state.passthroughPackets++;
        break;
    }
    case 0x500A: {
        const int rpm1Raw = packet.data & 0xFFFF;
        const int rpm2Raw = (packet.data >> 16) & 0xFFFF;
        state.rpm1 = (((packet.data >> 15) & 0x1) ? -((~rpm1Raw & 0xFFFF) + 1) : rpm1Raw) * 10;
        state.rpm2 = (((packet.data >> 31) & 0x1) ? -((~rpm2Raw & 0xFFFF) + 1) : rpm2Raw) * 10;
        state.cnt500A++;
        state.passthroughPackets++;
        break;
    }
    case 0x500B: {
        const int heightExp = packet.data & 0x3;
        const int heightMantissa = (packet.data >> 2) & 0x3FF;
        const int heightSign = (packet.data >> 12) & 0x1;
        state.terrainHeightM =
            (heightSign ? -1.0f : 1.0f) * static_cast<float>(heightMantissa) * pow10u(heightExp) * 0.1f;
        state.terrainUnhealthy = ((packet.data >> 13) & 0x1) != 0;
        state.cnt500B++;
        state.passthroughPackets++;
        break;
    }
    case 0x500C: {
        state.trueWindAngleDeg = static_cast<float>((packet.data & 0x7F) * 3);
        state.trueWindSpeedMs = decodePrepNumber21Signed(packet.data, 7);
        const int appAngleSign = (packet.data >> 15) & 0x1;
        state.apparentWindAngleDeg =
            static_cast<float>((appAngleSign ? -1 : 1) * static_cast<int>((packet.data >> 16) & 0x7F) * 3);
        state.apparentWindSpeedMs = decodePrepNumber21Signed(packet.data, 22);
        state.cnt500C++;
        state.passthroughPackets++;
        break;
    }
    case 0x800: {
        const bool isLatitude = (packet.data & 0x80000000) == 0;
        decodeGpsCoord(packet.data, isLatitude, state.latE7, state.lonE7);
        if (state.latE7 != 0 || state.lonE7 != 0) {
            state.gpsValid = true;
        }
        state.cnt0800++;
        state.passthroughPackets++;
        break;
    }
    default:
        if (packet.appId >= 0x5000 && packet.appId <= 0x50FF) {
            state.cnt50Other++;
            state.passthroughPackets++;
        } else {
            state.cntUnknown++;
        }
        break;
    }
}
