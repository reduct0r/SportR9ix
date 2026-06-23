#include "SportParser.h"

#include "config.h"



namespace {

uint8_t getBit(uint8_t value, uint8_t index) {

    return (value >> index) & 0x01;

}



// OpenTX sportPollSerialPorts() wire bytes for physical slots 1..28.

constexpr uint8_t kOpenTxPollWireIds[28] = {

    0x00, 0xA1, 0x22, 0x83, 0xE4, 0x45, 0xC6, 0x67, 0x48, 0xE9, 0x6A, 0xCB, 0xAC, 0x0D,

    0x8E, 0x2F, 0xD0, 0x71, 0xF2, 0x53, 0x34, 0x95, 0x16, 0xB7, 0x98, 0x39, 0xBA, 0x1B,

};



uint8_t openTxPollIndex = 0;

bool isPrimId(uint8_t b) {
    return b == 0x10 || b == 0x32 || b == 0x30 || b == 0x31;
}

bool crcBytes1toN(const uint8_t *pkt, uint8_t crcIndex) {
    uint16_t sum = 0;
    for (uint8_t i = 1; i <= crcIndex; ++i) {
        sum += pkt[i];
        sum += sum >> 8;
        sum &= 0xFF;
    }
    return sum == 0xFF;
}

bool tryOk8(const uint8_t *p, uint16_t &appId) {
    if (!isPrimId(p[0]) || !crcBytes1toN(p, 7)) {
        return false;
    }
    appId = static_cast<uint16_t>(p[1] | (static_cast<uint16_t>(p[2]) << 8));
    return true;
}

bool tryOk9(const uint8_t *p, uint16_t &appId) {
    if (!isPrimId(p[1]) || !crcBytes1toN(p, 8)) {
        return false;
    }
    appId = static_cast<uint16_t>(p[2] | (static_cast<uint16_t>(p[3]) << 8));
    return true;
}

void invertBuf(uint8_t *dst, const uint8_t *src, size_t n) {
    for (size_t i = 0; i < n; ++i) {
        dst[i] = static_cast<uint8_t>(src[i] ^ 0xFF);
    }
}

void fillPacket8(const uint8_t *p, SportPacket &out) {
    out.sensor = 0;
    out.frame = p[0];
    out.appId = static_cast<uint16_t>(p[1] | (static_cast<uint16_t>(p[2]) << 8));
    out.data = p[3] | (static_cast<uint32_t>(p[4]) << 8) | (static_cast<uint32_t>(p[5]) << 16) |
               (static_cast<uint32_t>(p[6]) << 24);
}

void fillPacket9(const uint8_t *p, SportPacket &out) {
    out.sensor = p[0] & 0x1F;
    out.frame = p[1];
    out.appId = static_cast<uint16_t>(p[2] | (static_cast<uint16_t>(p[3]) << 8));
    out.data = p[4] | (static_cast<uint32_t>(p[5]) << 8) | (static_cast<uint32_t>(p[6]) << 16) |
               (static_cast<uint32_t>(p[7]) << 24);
}

} // namespace



uint8_t SportPort::calcSensorId(uint8_t physicalId) {

    uint8_t result = physicalId;

    result += (getBit(physicalId, 0) ^ getBit(physicalId, 1) ^ getBit(physicalId, 2)) << 5;

    result += (getBit(physicalId, 2) ^ getBit(physicalId, 3) ^ getBit(physicalId, 4)) << 6;

    result += (getBit(physicalId, 0) ^ getBit(physicalId, 2) ^ getBit(physicalId, 4)) << 7;

    return result;

}



uint8_t SportPort::openTxPollWireId(uint8_t slotIndex) {

    return kOpenTxPollWireIds[slotIndex % 28];

}



uint8_t SportPort::currentOpenTxPollIndex() const {

    return openTxPollIndex;

}



void SportPort::pollPhysicalId(uint8_t physicalId) {

    pollSensor(calcSensorId(physicalId));

}



void SportPort::pollOpenTxNext() {

    pollSensor(kOpenTxPollWireIds[openTxPollIndex]);

    openTxPollIndex = static_cast<uint8_t>((openTxPollIndex + 1) % 28);

}



bool SportParser::isDataPrimId(uint8_t primId) {

    return primId == DATA_FRAME || primId == 0x32 || primId == 0x30 || primId == 0x31;

}



bool SportParser::sportCrcOk(const uint8_t *pkt, uint8_t crcIndex) {

    uint16_t sum = 0;

    for (uint8_t i = 1; i <= crcIndex; ++i) {

        sum += pkt[i];

        sum += sum >> 8;

        sum &= 0xFF;

    }

    return sum == 0xFF;

}



void SportParser::restartIdle() {

    mode_ = Mode::Idle;

    count_ = 0;

    pendingPhys_ = 0;

}



bool SportParser::validateAndExtract8(SportPacket &out) {

    if (!isDataPrimId(buffer_[0])) {

        return false;

    }



#if !SPORT_SKIP_CRC

    if (!sportCrcOk(buffer_, 7)) {

        crcFail_++;

        return false;

    }

#endif



    crcOk_++;

    frames8_++;

    out.sensor = 0;

    out.frame = buffer_[0];

    out.appId = buffer_[1] | (static_cast<uint16_t>(buffer_[2]) << 8);

    out.data = buffer_[3] | (static_cast<uint32_t>(buffer_[4]) << 8) |

               (static_cast<uint32_t>(buffer_[5]) << 16) |

               (static_cast<uint32_t>(buffer_[6]) << 24);

    return true;

}



bool SportParser::validateAndExtract9(SportPacket &out) {

    if (!isDataPrimId(buffer_[1])) {

        return false;

    }



#if !SPORT_SKIP_CRC

    if (!sportCrcOk(buffer_, 8)) {

        crcFail_++;

        return false;

    }

#endif



    crcOk_++;

    frames9_++;

    out.sensor = buffer_[0] & 0x1F;

    out.frame = buffer_[1];

    out.appId = buffer_[2] | (static_cast<uint16_t>(buffer_[3]) << 8);

    out.data = buffer_[4] | (static_cast<uint32_t>(buffer_[5]) << 8) |

               (static_cast<uint32_t>(buffer_[6]) << 16) |

               (static_cast<uint32_t>(buffer_[7]) << 24);

    return true;

}



void SportParser::reset() {

    restartIdle();

    skipPollId_ = false;

    inEscape_ = false;

}



void SportParser::noteValidFrame8() {
    crcOk_++;
    frames8_++;
}

void SportParser::noteValidFrame9() {
    crcOk_++;
    frames9_++;
}



bool SportParser::feed(uint8_t byte, SportPacket &out) {

    if (byte == FRAME_HEAD) {

        skipPollId_ = true;

        restartIdle();

        return false;

    }



    if (skipPollId_) {

        skipPollId_ = false;

        return false;

    }



    if (byte == FRAME_DLE) {

        inEscape_ = true;

        return false;

    }



    if (inEscape_) {

        byte ^= STUFF_MASK;

        inEscape_ = false;

    }



    switch (mode_) {

    case Mode::Idle:

        if (isDataPrimId(byte)) {

            buffer_[0] = byte;

            count_ = 1;

            mode_ = Mode::Frame8;

            break;

        }

        pendingPhys_ = byte;

        mode_ = Mode::WaitPrim;

        break;



    case Mode::WaitPrim:

        if (isDataPrimId(byte)) {

            buffer_[0] = pendingPhys_;

            buffer_[1] = byte;

            count_ = 2;

            mode_ = Mode::Frame9;

        } else {

            const uint8_t saved = byte;

            restartIdle();

            if (isDataPrimId(saved)) {

                buffer_[0] = saved;

                count_ = 1;

                mode_ = Mode::Frame8;

            } else if (isDataPrimId(pendingPhys_)) {

                buffer_[0] = pendingPhys_;

                count_ = 1;

                mode_ = Mode::Frame8;

            }

        }

        break;



    case Mode::Frame8:

        if (count_ < sizeof(buffer_)) {

            buffer_[count_++] = byte;

        }

        break;



    case Mode::Frame9:

        if (count_ < sizeof(buffer_)) {

            buffer_[count_++] = byte;

        }

        break;

    }



    if (mode_ == Mode::Frame8 && count_ >= FRAME8_SIZE) {

        const bool ok = validateAndExtract8(out);

        restartIdle();

        return ok;

    }



    if (mode_ == Mode::Frame9 && count_ >= FRAME9_SIZE) {

        const bool ok = validateAndExtract9(out);

        restartIdle();

        return ok;

    }



    return false;

}



void SportPort::begin(uint8_t pin, uint32_t baud, bool activePoll, bool invert) {

    pin_ = pin;

    activePoll_ = activePoll;

    serial_ = &Serial2;

    serial_->end();

    delay(5);

    serial_->begin(baud, SERIAL_8N1, pin, -1, invert);

    parser_.reset();

    openTxPollIndex = 0;

    sampleLen_ = 0;

    samplePos_ = 0;

    streamTotalBytes_ = 0;

    lastEmittedEndPos_ = 0;

}



void SportPort::pollSensor(uint8_t sensorId) {

    if (!serial_) {

        return;

    }

    const uint8_t pollFrame[] = {0x7E, sensorId};



    if (activePoll_) {

        serial_->end();

        delayMicroseconds(300);

        serial_->begin(SPORT_BAUD, SERIAL_8N1, pin_, pin_, SPORT_UART_INVERT);

        serial_->write(pollFrame, sizeof(pollFrame));

        serial_->flush();

        delayMicroseconds(1500);

        serial_->end();

        delayMicroseconds(300);

        serial_->begin(SPORT_BAUD, SERIAL_8N1, pin_, -1, SPORT_UART_INVERT);

        return;

    }



    serial_->write(pollFrame, sizeof(pollFrame));

    serial_->flush();

}



void SportPort::pushSampleByte(uint8_t b) {

    sample_[samplePos_] = b;

    samplePos_ = static_cast<uint8_t>((samplePos_ + 1) % kSampleSize);

    if (sampleLen_ < kSampleSize) {

        sampleLen_++;

    }

}



void SportPort::formatRecentSampleHex(char *out, size_t len) const {
    formatRecentSampleHexAt(out, len, 0, 16);
}

void SportPort::formatRecentSampleHexAt(char *out, size_t len, size_t start, size_t count) const {
    if (!out || len == 0) {
        return;
    }
    const size_t avail = sampleLen_ < kSampleSize ? sampleLen_ : kSampleSize;
    if (start >= avail) {
        snprintf(out, len, "hex empty");
        return;
    }
    const size_t n = count < (avail - start) ? count : (avail - start);
    int pos = snprintf(out, len, "hex");
    for (size_t i = 0; i < n && pos < static_cast<int>(len) - 4; ++i) {
        const size_t idx =
            sampleLen_ < kSampleSize
                ? start + i
                : (static_cast<size_t>(samplePos_) + start + i) % kSampleSize;
        pos += snprintf(out + pos, len - pos, " %02X", sample_[idx]);
    }
}

void SportPort::scanRecentRaw(SportRawScan &out) const {
    out = {};
    const size_t n = sampleLen_ < kSampleSize ? sampleLen_ : kSampleSize;
    if (n < 8) {
        return;
    }

    uint8_t buf[kSampleSize]{};
    uint8_t inv[kSampleSize]{};
    for (size_t i = 0; i < n; ++i) {
        const size_t idx =
            sampleLen_ < kSampleSize ? i : (static_cast<size_t>(samplePos_) + i) % kSampleSize;
        buf[i] = sample_[idx];
        if (buf[i] != 0) {
            out.nonzero++;
        }
        if (buf[i] == 0x7E) {
            out.count7e++;
        }
    }
    invertBuf(inv, buf, n);

    auto scanBuf = [&](const uint8_t *b, uint16_t &ok8, uint16_t &ok9) {
        for (size_t off = 0; off + 8 <= n; ++off) {
            uint16_t appId = 0;
            if (tryOk8(b + off, appId)) {
                ok8++;
                out.lastAppId = appId;
                out.lastOffset = static_cast<uint8_t>(off);
            }
        }
        for (size_t off = 0; off + 9 <= n; ++off) {
            uint16_t appId = 0;
            if (tryOk9(b + off, appId)) {
                ok9++;
                out.lastAppId = appId;
                out.lastOffset = static_cast<uint8_t>(off);
            }
        }
    };

    scanBuf(buf, out.ok8, out.ok9);
    scanBuf(inv, out.ok8Inv, out.ok9Inv);
}

void SportPort::formatSyncReport(char *out, size_t len, const SportRawScan &r) const {
    snprintf(out, len,
             "sync 8=%u 9=%u i8=%u nz=%u 7E=%u id=%04X @%u",
             static_cast<unsigned>(r.ok8), static_cast<unsigned>(r.ok9),
             static_cast<unsigned>(r.ok8Inv), static_cast<unsigned>(r.nonzero),
             static_cast<unsigned>(r.count7e), r.lastAppId,
             static_cast<unsigned>(r.lastOffset));
}



bool SportPort::trySlidingExtract(SportPacket &out) {
    const size_t n = sampleLen_ < kSampleSize ? sampleLen_ : kSampleSize;
    if (n < 8) {
        return false;
    }

    uint8_t buf[kSampleSize]{};
    for (size_t i = 0; i < n; ++i) {
        const size_t idx =
            sampleLen_ < kSampleSize ? i : (static_cast<size_t>(samplePos_) + i) % kSampleSize;
        buf[i] = sample_[idx];
    }

    size_t bestOff = SIZE_MAX;
    size_t bestLen = 0;
    uint32_t bestEndPos = 0;

    for (size_t off = 0; off + 9 <= n; ++off) {
        uint16_t appId = 0;
        if (!tryOk9(buf + off, appId)) {
            continue;
        }
        const uint32_t endPos = streamTotalBytes_ - static_cast<uint32_t>(n) + static_cast<uint32_t>(off) + 9;
        if (endPos <= lastEmittedEndPos_ || endPos < bestEndPos) {
            continue;
        }
        bestOff = off;
        bestLen = 9;
        bestEndPos = endPos;
        (void)appId;
    }

    for (size_t off = 0; off + 8 <= n; ++off) {
        uint16_t appId = 0;
        if (!tryOk8(buf + off, appId)) {
            continue;
        }
        const uint32_t endPos = streamTotalBytes_ - static_cast<uint32_t>(n) + static_cast<uint32_t>(off) + 8;
        if (endPos <= lastEmittedEndPos_ || endPos < bestEndPos) {
            continue;
        }
        bestOff = off;
        bestLen = 8;
        bestEndPos = endPos;
        (void)appId;
    }

    if (bestOff == SIZE_MAX) {
        return false;
    }

    lastEmittedEndPos_ = bestEndPos;
    if (bestLen == 9) {
        fillPacket9(buf + bestOff, out);
        parser_.noteValidFrame9();
    } else {
        fillPacket8(buf + bestOff, out);
        parser_.noteValidFrame8();
    }
    return true;
}



bool SportPort::readPacket(SportPacket &out) {

    if (!serial_) {

        return false;

    }

    while (serial_->available()) {

        const uint8_t b = static_cast<uint8_t>(serial_->read());

        rawBytes_++;

        streamTotalBytes_++;

        pushSampleByte(b);

        if (activePoll_) {
            if (parser_.feed(b, out)) {
                return true;
            }
        } else if (trySlidingExtract(out)) {
            return true;
        }

    }

    return false;

}


