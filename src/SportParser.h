#pragma once



#include <Arduino.h>

#include "config.h"



struct SportPacket {
    uint8_t sensor = 0;
    uint8_t frame = 0;
    uint16_t appId = 0;
    uint32_t data = 0;
};

// Sliding-window CRC scan over recent raw RX bytes (desync / invert check).
struct SportRawScan {
    uint16_t ok8 = 0;
    uint16_t ok9 = 0;
    uint16_t ok8Inv = 0;
    uint16_t ok9Inv = 0;
    uint8_t count7e = 0;
    uint8_t nonzero = 0;
    uint16_t lastAppId = 0;
    uint8_t lastOffset = 0;
};



class SportParser {

public:

    bool feed(uint8_t byte, SportPacket &out);

    void reset();

    uint32_t crcOk() const { return crcOk_; }

    uint32_t crcFail() const { return crcFail_; }

    uint32_t frames8() const { return frames8_; }

    uint32_t frames9() const { return frames9_; }

    void noteValidFrame8();

    void noteValidFrame9();



private:

    static constexpr uint8_t FRAME_HEAD = 0x7E;

    static constexpr uint8_t FRAME_DLE = 0x7D;

    static constexpr uint8_t STUFF_MASK = 0x20;

    static constexpr uint8_t DATA_FRAME = 0x10;

    static constexpr uint8_t FRAME8_SIZE = 8;

    static constexpr uint8_t FRAME9_SIZE = 9;



    enum class Mode : uint8_t { Idle, WaitPrim, Frame8, Frame9 };



    Mode mode_ = Mode::Idle;

    uint8_t buffer_[16]{};

    uint8_t count_ = 0;

    uint8_t pendingPhys_ = 0;

    bool skipPollId_ = false;

    bool inEscape_ = false;

    uint32_t crcOk_ = 0;

    uint32_t crcFail_ = 0;

    uint32_t frames8_ = 0;

    uint32_t frames9_ = 0;



    static bool isDataPrimId(uint8_t primId);

    static bool sportCrcOk(const uint8_t *pkt, uint8_t crcIndex);

    bool validateAndExtract8(SportPacket &out);

    bool validateAndExtract9(SportPacket &out);

    void restartIdle();

};



class SportPort {

public:

    void begin(uint8_t pin, uint32_t baud, bool activePoll = false,

               bool invert = SPORT_UART_INVERT);

    void pollSensor(uint8_t sensorId);

    void pollPhysicalId(uint8_t physicalId);

    void pollOpenTxNext();

    static uint8_t calcSensorId(uint8_t physicalId);

    static uint8_t openTxPollWireId(uint8_t slotIndex);

    uint8_t currentOpenTxPollIndex() const;

    bool readPacket(SportPacket &out);

    uint32_t rawByteCount() const { return rawBytes_; }

    void resetRawByteCount() { rawBytes_ = 0; }

    uint32_t parserCrcOk() const { return parser_.crcOk(); }

    uint32_t parserCrcFail() const { return parser_.crcFail(); }

    uint32_t parserFrames8() const { return parser_.frames8(); }

    uint32_t parserFrames9() const { return parser_.frames9(); }

    void formatRecentSampleHex(char *out, size_t len) const;
    void formatRecentSampleHexAt(char *out, size_t len, size_t start, size_t count) const;
    void scanRecentRaw(SportRawScan &out) const;
    void formatSyncReport(char *out, size_t len, const SportRawScan &r) const;

private:

    HardwareSerial *serial_ = nullptr;

    SportParser parser_;

    uint8_t pin_ = 0;

    bool activePoll_ = false;

    uint32_t rawBytes_ = 0;

    static constexpr size_t kSampleSize = 128;

    uint8_t sample_[kSampleSize]{};

    uint8_t sampleLen_ = 0;

    uint8_t samplePos_ = 0;



    void pushSampleByte(uint8_t b);

    bool trySlidingExtract(SportPacket &out);

    uint32_t streamTotalBytes_ = 0;

    uint32_t lastEmittedEndPos_ = 0;

};


