/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_SERIALRX_SUMD

#include "common/crc.h"
#include "common/utils.h"
#include "common/maths.h"

#include "drivers/time.h"

#include "io/serial.h"

#ifdef USE_TELEMETRY
#include "telemetry/telemetry.h"
#endif

#include "pg/rx.h"

#include "rx/rx.h"
#include "rx/sumd.h"

// driver for SUMD receiver using UART2

// Support for SUMD and SUMD V3
// Tested with 16 channels, SUMD supports up to 16(*), SUMD V3 up to 32 (MZ-32) channels, but limit to MAX_SUPPORTED_RC_CHANNEL_COUNT (currently 8, BF 3.4)
// * According to the original SUMD V1 documentation, SUMD V1 already supports up to 32 Channels?!?

#define SUMD_SYNCBYTE 0xA8
#define SUMD_MAX_CHANNEL 32
#define SUMD_BYTES_PER_CHANNEL 2
#define SUMD_BUFFSIZE (SUMD_MAX_CHANNEL * SUMD_BYTES_PER_CHANNEL) // 2 bytes per channel
#define SUMD_BAUDRATE 115200

#define SUMDV1_FRAME_STATE_OK 0x01
#define SUMDV3_FRAME_STATE_OK 0x03
#define SUMD_FRAME_STATE_FAILSAFE 0x81

static bool sumdFrameDone = false;
static uint16_t sumdChannels[MAX_SUPPORTED_RC_CHANNEL_COUNT];
static uint16_t sumdCrcReceived;
static uint8_t sumdStatus;
static uint16_t crc;

static uint8_t sumd[SUMD_BUFFSIZE] = { 0, };
static uint8_t sumdChannelCount;

typedef enum { SUMD_UNSYNCED, SUMD_READING_STATE, SUMD_READING_LENGTH, SUMD_READING_DATA, SUMD_READING_CRC_HIGH, SUMD_READING_CRC_LOW} state_codes_t;
state_codes_t recvState = SUMD_UNSYNCED;

// Receive ISR callback
static void sumdDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    static uint8_t channelBytesLength = 0;
    static uint8_t channelBytesRead = 0;
    static uint32_t sumdTimeLast = 0;
    state_codes_t nextRecvState = recvState;
    uint32_t sumdTime = micros();

    if ((sumdTime - sumdTimeLast) > 4000) {
        nextRecvState = SUMD_UNSYNCED;
        recvState = SUMD_UNSYNCED;
    }
    sumdTimeLast = sumdTime;

    switch(recvState) {
        case SUMD_UNSYNCED:
            if (c == SUMD_SYNCBYTE) {
                nextRecvState = SUMD_READING_STATE;
                crc = crc16_ccitt(crc, (uint8_t)c);
            }
            break;
        case SUMD_READING_STATE:
             nextRecvState = SUMD_READING_LENGTH;
             sumdStatus = (uint8_t)c;
             crc = crc16_ccitt(crc, (uint8_t)c);
             break;
        case SUMD_READING_LENGTH:
             nextRecvState = SUMD_READING_DATA;
             channelBytesLength = SUMD_BYTES_PER_CHANNEL * (uint8_t)c;
             channelBytesRead = 0;
             crc = crc16_ccitt(crc, (uint8_t)c);
             break;
        case SUMD_READING_DATA:
            crc = crc16_ccitt(crc, (uint8_t)c);
            if (channelBytesRead < SUMD_BUFFSIZE) {
                sumd[channelBytesRead] = (uint8_t)c;
            }
            channelBytesRead++;
            if (channelBytesRead >= channelBytesLength) {
                nextRecvState = SUMD_READING_CRC_HIGH;
            }
            break;
        case SUMD_READING_CRC_HIGH:
            nextRecvState = SUMD_READING_CRC_LOW;
            sumdCrcReceived = c << 8;
            break;
        case SUMD_READING_CRC_LOW:
            nextRecvState = SUMD_UNSYNCED;
            sumdCrcReceived |= (uint8_t)c;
            sumdFrameDone = true;
            break;
        default:
            nextRecvState = SUMD_UNSYNCED;
    }

    recvState = nextRecvState;
}

static uint8_t sumdFrameStatus(rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);

    uint8_t frameStatus = RX_FRAME_PENDING;

    if (!sumdFrameDone) {
        return frameStatus;
    }

    sumdFrameDone = false;

    // verify CRC
    if (crc != sumdCrcReceived) {
        return frameStatus;
    }

    switch (sumdStatus) {
        case SUMD_FRAME_STATE_FAILSAFE:
            frameStatus = RX_FRAME_COMPLETE | RX_FRAME_FAILSAFE;
            break;
        case SUMDV1_FRAME_STATE_OK:
        case SUMDV3_FRAME_STATE_OK:
            frameStatus = RX_FRAME_COMPLETE;
            break;
        default:
            return frameStatus;
    }

    unsigned channelsToProcess = MIN(sumdChannelCount, MAX_SUPPORTED_RC_CHANNEL_COUNT);

    for (unsigned channelIndex = 0; channelIndex < channelsToProcess; channelIndex++) {
        sumdChannels[channelIndex] = (
                (sumd[SUMD_BYTES_PER_CHANNEL * channelIndex] << 8) | sumd[SUMD_BYTES_PER_CHANNEL * channelIndex + 1]
        );
    }
    return frameStatus;
}

static uint16_t sumdReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    UNUSED(rxRuntimeConfig);
    return sumdChannels[chan] / 8;
}

bool sumdInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxConfig);

    rxRuntimeConfig->channelCount = MIN(SUMD_MAX_CHANNEL, MAX_SUPPORTED_RC_CHANNEL_COUNT);
    rxRuntimeConfig->rxRefreshRate = 11000;

    rxRuntimeConfig->rcReadRawFn = sumdReadRawRC;
    rxRuntimeConfig->rcFrameStatusFn = sumdFrameStatus;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

#ifdef USE_TELEMETRY
    bool portShared = telemetryCheckRxPortShared(portConfig);
#else
    bool portShared = false;
#endif

    serialPort_t *sumdPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        sumdDataReceive,
        NULL,
        SUMD_BAUDRATE,
        portShared ? MODE_RXTX : MODE_RX,
        (rxConfig->serialrx_inverted ? SERIAL_INVERTED : 0) | (rxConfig->halfDuplex ? SERIAL_BIDIR : 0)
        );

#ifdef USE_TELEMETRY
    if (portShared) {
        telemetrySharedPort = sumdPort;
    }
#endif

    return sumdPort != NULL;
}
#endif
