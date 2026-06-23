#pragma once

#include <Arduino.h>
#include "SportParser.h"

// Tracks SmartPort App IDs seen on the bus (listen and/or poll).
class SportDiag {
public:
    void notePacket(const SportPacket &packet);
    void resetWindow();
    void formatSummary(char *out, size_t len) const;
    void formatScanResult(char *out, size_t len, uint8_t polledPhysicalId, uint32_t responses) const;

    void beginScan();
    bool scanActive() const { return scanActive_; }
    bool scanReportPending() const { return scanReportPending_; }
    void clearScanReportPending() { scanReportPending_ = false; }
    bool scanStep(SportPort &port, uint32_t nowMs);
    void finishScan();
    uint32_t scanResponseCount() const { return scanResponses_; }

private:
    static constexpr int kMaxIds = 6;

    struct IdCount {
        uint16_t id = 0;
        uint32_t count = 0;
    };

    IdCount ids_[kMaxIds]{};
    IdCount scanIds_[kMaxIds]{};
    uint32_t scanResponses_ = 0;

    bool scanActive_ = false;
    bool scanReportPending_ = false;
    uint8_t scanIndex_ = 0;
    uint32_t scanStepMs_ = 0;
    uint32_t scanEndMs_ = 0;

    void bump(IdCount *list, uint16_t id);
};
