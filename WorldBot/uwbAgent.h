#pragma once

#include "dw3000.h"
#include <SPI.h>
#include <math.h>

extern dwt_txconfig_t txconfig_options;

class UWBAgent {
public:
    static constexpr int NUM_ANCHORS = 4;

    UWBAgent(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t ss, uint8_t rst, uint8_t irq)
        : _sck(sck), _miso(miso), _mosi(mosi), _ss(ss), _rst(rst), _irq(irq) {}

    // Set anchor position (meters). Call before begin(). Index 0–3.
    void setAnchorPosition(int idx, float x, float y) {
        if (idx >= 0 && idx < NUM_ANCHORS) { _anchorX[idx] = x; _anchorY[idx] = y; }
    }

    bool begin() {
        SPI.begin(_sck, _miso, _mosi, _ss);
        spiBegin(_irq, _rst);
        spiSelect(_ss);
        delay(10);

        if (!dwt_checkidlerc())                       { Serial.println("UWB: IDLE FAILED");   return false; }
        if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) { Serial.println("UWB: INIT FAILED");   return false; }
        if (dwt_configure(&_config))                  { Serial.println("UWB: CONFIG FAILED");  return false; }

        dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
        dwt_configurestskey(&_cp_key);
        dwt_configurestsiv(&_cp_iv);
        dwt_configurestsloadiv();
        dwt_configuretxrf(&txconfig_options);
        dwt_setrxantennadelay(_RX_ANT_DLY);
        dwt_settxantennadelay(_TX_ANT_DLY);
        dwt_setrxaftertxdelay(_POLL_TX_TO_RESP_RX_DLY_UUS);
        dwt_setrxtimeout(_RESP_RX_TIMEOUT_UUS);
        dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

        _mutex = xSemaphoreCreateMutex();
        xTaskCreate(_taskFn, "UWB", 8192, this, 1, nullptr);
        return true;
    }

    float getX() {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        float v = _px;
        xSemaphoreGive(_mutex);
        return v;
    }

    float getY() {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        float v = _py;
        xSemaphoreGive(_mutex);
        return v;
    }

    bool isPositionValid() {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        bool v = !isnan(_px);
        xSemaphoreGive(_mutex);
        return v;
    }

    float getAnchorDistance(int idx) { return (idx >= 0 && idx < NUM_ANCHORS) ? (float)_distances[idx] : NAN; }
    bool  isAnchorValid(int idx)     { return (idx >= 0 && idx < NUM_ANCHORS) && _valid[idx]; }

private:
    static constexpr uint16_t _TX_ANT_DLY                 = 16350;
    static constexpr uint16_t _RX_ANT_DLY                 = 16350;
    static constexpr uint16_t _POLL_TX_TO_RESP_RX_DLY_UUS = 240;
    static constexpr uint16_t _RESP_RX_TIMEOUT_UUS         = 1500;

    static constexpr int _SN_IDX         = 2;
    static constexpr int _COMMON_LEN     = 10;
    static constexpr int _POLL_RX_TS_IDX = 10;
    static constexpr int _RESP_TX_TS_IDX = 14;

    static constexpr uint8_t _TAG_ADDR  = 0x04;
    static constexpr uint8_t _FUNC_POLL = 0xE0;
    static constexpr uint8_t _FUNC_RESP = 0xE1;

    uint8_t _sck, _miso, _mosi, _ss, _rst, _irq;

    float             _px = NAN, _py = NAN;
    double            _distances[NUM_ANCHORS] = {};
    bool              _valid[NUM_ANCHORS]     = {};
    float             _anchorX[NUM_ANCHORS]   = {};
    float             _anchorY[NUM_ANCHORS]   = {};
    SemaphoreHandle_t _mutex                  = nullptr;

    uint8_t  _frameSeqNb = 0;
    uint8_t  _rxBuf[24]  = {};
    uint32_t _statusReg  = 0;

    dwt_config_t _config = {
        5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1,
        DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD,
        (129 + 8 - 8), DWT_STS_MODE_1, DWT_STS_LEN_64, DWT_PDOA_M0
    };

    // STS credentials — must match anchor firmware exactly
    dwt_sts_cp_key_t _cp_key = {0x14EB220F, 0xF86050A8, 0xD1D336AA, 0x14148674};
    dwt_sts_cp_iv_t  _cp_iv  = {0x1F9A3DE4, 0xD37EC3CA, 0xC44FA8FB, 0x362EEB34};

    // Poll message template — bytes [5] and [_SN_IDX] are overwritten each call
    uint8_t _txPollMsg[12] = {
        0x41, 0x88, 0, 0xCA, 0xDE,
        0x00, 0x00,
        _TAG_ADDR, 0x00,
        _FUNC_POLL, 0, 0
    };

    static void _taskFn(void* arg) {
        UWBAgent* self = static_cast<UWBAgent*>(arg);
        for (;;) self->_update();
    }

    void _update() {
        for (int i = 0; i < NUM_ANCHORS; i++) {
            _valid[i] = _rangeAnchor(i);
            vTaskDelay(1);
        }

        float px, py;
        _trilaterate(px, py);

        xSemaphoreTake(_mutex, portMAX_DELAY);
        _px = px;
        _py = py;
        xSemaphoreGive(_mutex);
        vTaskDelay(1);
    }

    bool _rangeAnchor(int idx) {
        _txPollMsg[5]       = (uint8_t)(idx + 1);
        _txPollMsg[6]       = 0x00;
        _txPollMsg[_SN_IDX] = _frameSeqNb;

        dwt_write32bitreg(SYS_STATUS_ID,
            SYS_STATUS_TXFRS_BIT_MASK | SYS_STATUS_ALL_RX_GOOD |
            SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        dwt_configurestsiv(&_cp_iv);
        dwt_configurestsloadiv();
        dwt_writetxdata(sizeof(_txPollMsg), _txPollMsg, 0);
        dwt_writetxfctrl(sizeof(_txPollMsg), 0, 1);
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        while (!((_statusReg = dwt_read32bitreg(SYS_STATUS_ID)) &
                 (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
        {}

        _frameSeqNb++;

        int16_t stsQual;
        int goodSts = dwt_readstsquality(&stsQual);
        if (!(_statusReg & SYS_STATUS_RXFCG_BIT_MASK) || goodSts < 0) {
            dwt_write32bitreg(SYS_STATUS_ID,
                SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_GOOD);
            return false;
        }

        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD);

        uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        if (frame_len > sizeof(_rxBuf)) return false;

        dwt_readrxdata(_rxBuf, frame_len, 0);
        _rxBuf[_SN_IDX] = 0;

        // Verify: dest=tag, src=anchor(idx+1), function=RESP
        const uint8_t expected[_COMMON_LEN] = {
            0x41, 0x88, 0, 0xCA, 0xDE,
            _TAG_ADDR, 0x00, (uint8_t)(idx + 1), 0x00, _FUNC_RESP
        };
        if (memcmp(_rxBuf, expected, _COMMON_LEN) != 0) return false;

        uint32_t poll_tx_ts = dwt_readtxtimestamplo32();
        uint32_t resp_rx_ts = dwt_readrxtimestamplo32();
        float clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

        uint32_t poll_rx_ts, resp_tx_ts;
        resp_msg_get_ts(&_rxBuf[_POLL_RX_TS_IDX], &poll_rx_ts);
        resp_msg_get_ts(&_rxBuf[_RESP_TX_TS_IDX], &resp_tx_ts);

        int32_t rtd_init = resp_rx_ts - poll_tx_ts;
        int32_t rtd_resp = resp_tx_ts - poll_rx_ts;
        double tof = ((rtd_init - rtd_resp * (1.0 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        _distances[idx] = tof * SPEED_OF_LIGHT;
        return true;
    }

    void _trilaterate(float& outX, float& outY) {
        int ref = -1, n = 0;
        for (int i = 0; i < NUM_ANCHORS; i++) {
            if (_valid[i]) { if (ref < 0) ref = i; n++; }
        }
        if (n < 3) { outX = outY = NAN; return; }

        float x1 = _anchorX[ref], y1 = _anchorY[ref], d1 = (float)_distances[ref];
        float ATA00 = 0, ATA01 = 0, ATA11 = 0, ATb0 = 0, ATb1 = 0;

        for (int i = 0; i < NUM_ANCHORS; i++) {
            if (!_valid[i] || i == ref) continue;
            float xi = _anchorX[i], yi = _anchorY[i], di = (float)_distances[i];
            float ai = 2.0f * (x1 - xi);
            float bi = 2.0f * (y1 - yi);
            float ci = di*di - d1*d1 - xi*xi + x1*x1 - yi*yi + y1*y1;
            ATA00 += ai * ai; ATA01 += ai * bi; ATA11 += bi * bi;
            ATb0  += ai * ci; ATb1  += bi * ci;
        }

        float det = ATA00 * ATA11 - ATA01 * ATA01;
        if (fabsf(det) < 1e-6f) { outX = outY = NAN; return; }
        outX = (ATA11 * ATb0 - ATA01 * ATb1) / det;
        outY = (ATA00 * ATb1 - ATA01 * ATb0) / det;
    }
};
