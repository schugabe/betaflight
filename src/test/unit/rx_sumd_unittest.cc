/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

extern "C" {
#include "platform.h"
#include "pg/pg.h"
#include "pg/rx.h"
#include "drivers/serial.h"
#include "drivers/time.h"
#include "io/serial.h"
#include "rx/rx.h"
#include "rx/sumd.h"
#include "telemetry/telemetry.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"


extern "C" {
    uint8_t batteryCellCount = 3;
    float rcCommand[4] = {0, 0, 0, 0};
    int16_t telemTemperature1 = 0;
    baro_t baro = { .baroTemperature = 50 };
    telemetryConfig_t telemetryConfig_System;
}


bool telemetryCheckRxPortShared(const serialPortConfig_t *portConfig)
{
    //TODO: implement
    (void) portConfig;
    return false;
}

serialPort_t * telemetrySharedPort = NULL;

static uint16_t vbat = 100;
uint16_t getVbat(void)
{
    return vbat;
}

uint32_t microseconds_stub_value = 0;
uint32_t micros(void)
{
    return microseconds_stub_value;
}

#define SERIAL_BUFFER_SIZE 256
#define SERIAL_PORT_DUMMY_IDENTIFIER  (serialPortIdentifier_e)0x1234

typedef struct serialPortStub_s {
    uint8_t buffer[SERIAL_BUFFER_SIZE];
    int pos = 0;
    int end = 0;
} serialPortStub_t;

static serialPort_t serialTestInstance;
static serialPortConfig_t serialTestInstanceConfig = {
    .identifier = SERIAL_PORT_DUMMY_IDENTIFIER,
    .functionMask = 0
};

static serialReceiveCallbackPtr stub_serialRxCallback;
static serialPortConfig_t *findSerialPortConfig_stub_retval;
static bool openSerial_called = false;
static serialPortStub_t serialWriteStub;
static bool portIsShared = false;



serialPortConfig_t *findSerialPortConfig(serialPortFunction_e function)
{
    EXPECT_EQ(function, FUNCTION_RX_SERIAL);
    return findSerialPortConfig_stub_retval;
}

static portMode_e serialExpectedMode = MODE_RX;
static portOptions_e serialExpectedOptions = SERIAL_UNIDIR;

serialPort_t *openSerialPort(
    serialPortIdentifier_e identifier,
    serialPortFunction_e function,
    serialReceiveCallbackPtr callback,
    void *callbackData,
    uint32_t baudrate,
    portMode_e mode,
    portOptions_e options
)
{
    openSerial_called = true;
    EXPECT_FALSE(NULL == callback);
    EXPECT_TRUE(NULL == callbackData);
    EXPECT_EQ(identifier, SERIAL_PORT_DUMMY_IDENTIFIER);
    EXPECT_EQ(options, serialExpectedOptions);
    EXPECT_EQ(function, FUNCTION_RX_SERIAL);
    EXPECT_EQ(baudrate, 115200);
    EXPECT_EQ(mode, serialExpectedMode);
    stub_serialRxCallback = callback;
    return &serialTestInstance;
}

void serialWrite(serialPort_t *instance, uint8_t ch)
{
    EXPECT_EQ(instance, &serialTestInstance);
    EXPECT_LT(serialWriteStub.pos, sizeof(serialWriteStub.buffer));
    serialWriteStub.buffer[serialWriteStub.pos++] = ch;
}


void serialTestResetPort()
{
    openSerial_called = false;
    stub_serialRxCallback = NULL;
    portIsShared = false;
    serialExpectedMode = MODE_RX;
    serialExpectedOptions = SERIAL_UNIDIR;
}


class SumdRxInitUnitTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        serialTestResetPort();
    }
};


TEST_F(SumdRxInitUnitTest, Test_SumdRxNotEnabled)
{
    const rxConfig_t initialRxConfig = {};
    rxRuntimeConfig_t rxRuntimeConfig = {};
    findSerialPortConfig_stub_retval = NULL;

    EXPECT_FALSE(sumdInit(&initialRxConfig, &rxRuntimeConfig));

    EXPECT_EQ(18, rxRuntimeConfig.channelCount);
    EXPECT_EQ(11000, rxRuntimeConfig.rxRefreshRate);
    EXPECT_FALSE(NULL == rxRuntimeConfig.rcReadRawFn);
    EXPECT_FALSE(NULL == rxRuntimeConfig.rcFrameStatusFn);
}


TEST_F(SumdRxInitUnitTest, Test_SumdRxEnabled)
{
    const rxConfig_t initialRxConfig = {};
    rxRuntimeConfig_t rxRuntimeConfig = {};
    findSerialPortConfig_stub_retval = &serialTestInstanceConfig;

    EXPECT_TRUE(sumdInit(&initialRxConfig, &rxRuntimeConfig));

    EXPECT_EQ(18, rxRuntimeConfig.channelCount);
    EXPECT_EQ(11000, rxRuntimeConfig.rxRefreshRate);
    EXPECT_FALSE(NULL == rxRuntimeConfig.rcReadRawFn);
    EXPECT_FALSE(NULL == rxRuntimeConfig.rcFrameStatusFn);

    EXPECT_TRUE(openSerial_called);
}



class SumdRxProtocollUnitTest : public ::testing::Test
{
protected:
    rxRuntimeConfig_t rxRuntimeConfig = {};
    virtual void SetUp()
    {
        serialTestResetPort();

        const rxConfig_t initialRxConfig = {};
        findSerialPortConfig_stub_retval = &serialTestInstanceConfig;

        EXPECT_TRUE(sumdInit(&initialRxConfig, &rxRuntimeConfig));
        microseconds_stub_value += 5000;
        EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeConfig.rcFrameStatusFn(&rxRuntimeConfig));
    }
};

TEST_F(SumdRxProtocollUnitTest, Test_OnePacketReceived)
{
    uint8_t packet[] = {0xA8, 0x01, 20,
                        0x1c, 0x20, 0x22, 0x60, 0x2e, 0xe0, 0x3b, 0x60, 0x41, 0xa0,
                        0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20,
                        0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20,
                        0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20, 0x1c, 0x20,
                        0x06, 0x3f}; //checksum

    for (size_t i=0; i < sizeof(packet); i++) {
        EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeConfig.rcFrameStatusFn(&rxRuntimeConfig));
        stub_serialRxCallback(packet[i], NULL);
    }

    //report frame complete once
    EXPECT_EQ(RX_FRAME_COMPLETE, rxRuntimeConfig.rcFrameStatusFn(&rxRuntimeConfig));
    EXPECT_EQ(RX_FRAME_PENDING, rxRuntimeConfig.rcFrameStatusFn(&rxRuntimeConfig));

    ASSERT_EQ(900, rxRuntimeConfig.rcReadRawFn(&rxRuntimeConfig, 0));
    ASSERT_EQ(1100, rxRuntimeConfig.rcReadRawFn(&rxRuntimeConfig, 1));
    ASSERT_EQ(1500, rxRuntimeConfig.rcReadRawFn(&rxRuntimeConfig, 2));
    ASSERT_EQ(1900, rxRuntimeConfig.rcReadRawFn(&rxRuntimeConfig, 3));
    ASSERT_EQ(2100, rxRuntimeConfig.rcReadRawFn(&rxRuntimeConfig, 4));
}
