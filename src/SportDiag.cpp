#include "SportDiag.h"
#include "config.h"

void SportDiag::bump(IdCount *list, uint16_t id) {
    for (int i = 0; i < kMaxIds; ++i) {
        if (list[i].id == id) {
            list[i].count++;
            return;
        }
        if (list[i].count == 0) {
            list[i].id = id;
            list[i].count = 1;
            return;
        }
    }
    // Replace smallest bucket
    int minIdx = 0;
    for (int i = 1; i < kMaxIds; ++i) {
        if (list[i].count < list[minIdx].count) {
            minIdx = i;
        }
    }
    if (list[minIdx].count <= 1) {
        list[minIdx].id = id;
        list[minIdx].count = 1;
    } else {
        list[minIdx].count++;
    }
}

void SportDiag::notePacket(const SportPacket &packet) {
    bump(ids_, packet.appId);
    if (scanActive_) {
        bump(scanIds_, packet.appId);
        scanResponses_++;
    }
}

void SportDiag::resetWindow() {
    for (int i = 0; i < kMaxIds; ++i) {
        ids_[i] = {};
    }
}

void SportDiag::formatSummary(char *out, size_t len) const {
    int pos = 0;
    pos += snprintf(out + pos, len > pos ? len - pos : 0, "ids");
    for (int i = 0; i < kMaxIds && pos < static_cast<int>(len) - 8; ++i) {
        if (ids_[i].count == 0) {
            break;
        }
        pos += snprintf(out + pos, len - pos, " %04X:%lu", ids_[i].id,
                        static_cast<unsigned long>(ids_[i].count));
    }
}

void SportDiag::formatScanResult(char *out, size_t len, uint8_t polledPhysicalId,
                                 uint32_t responses) const {
    int pos = snprintf(out, len, "scan id%02u rsp=%lu", polledPhysicalId,
                       static_cast<unsigned long>(responses));
    for (int i = 0; i < kMaxIds && pos < static_cast<int>(len) - 10; ++i) {
        if (scanIds_[i].count == 0) {
            break;
        }
        pos += snprintf(out + pos, len - pos, " %04X:%lu", scanIds_[i].id,
                        static_cast<unsigned long>(scanIds_[i].count));
    }
}

void SportDiag::beginScan() {
    for (int i = 0; i < kMaxIds; ++i) {
        scanIds_[i] = {};
    }
    scanResponses_ = 0;
    scanActive_ = true;
    scanReportPending_ = false;
    scanIndex_ = 0;
    scanStepMs_ = 0;
    scanEndMs_ = 0;
}

void SportDiag::finishScan() {
    scanActive_ = false;
}

bool SportDiag::scanStep(SportPort &port, uint32_t nowMs) {
    if (!scanActive_) {
        return false;
    }

    if (scanEndMs_ == 0) {
        scanEndMs_ = nowMs + SPORT_SCAN_DURATION_MS;
        scanStepMs_ = nowMs;
    }

    if (nowMs >= scanEndMs_) {
        scanReportPending_ = true;
        finishScan();
        return false;
    }

    if (nowMs - scanStepMs_ >= SPORT_SCAN_STEP_MS) {
        scanStepMs_ = nowMs;
        const uint8_t slot = scanIndex_;
        port.pollSensor(SportPort::openTxPollWireId(slot));
        scanIndex_ = static_cast<uint8_t>((scanIndex_ + 1) % 28);
    }
    return true;
}
