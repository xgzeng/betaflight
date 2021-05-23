#include "drivers/rangefinder/rangefinder.h"
#include "drivers/rangefinder/rangefinder_vl53l0.h"

#include <string.h>

#include "common/strtol.h"
#include "io/serial.h"

#define MAX_FRAME_LEN 32

static serialPort_t* vl53SerialPort = NULL;
static uint8_t vlFrameBuf[MAX_FRAME_LEN];

static int vlFramePos = 0; // current frame buffer read position
static int32_t vlDistance = RANGEFINDER_OUT_OF_RANGE; // distance in cm

static int32_t vlDistanceMM = 0; // distance in mm, value received in STAGE_WAIT_DATA

#define STAGE_WAIT_STATE_1 0
#define STAGE_WAIT_DATA    1
#define STAGE_WAIT_STATE_2 2

static int vlReceiveStage = STAGE_WAIT_STATE_1;

static void lidarVL53Init(rangefinderDev_t *dev)
{
    UNUSED(dev);
    // tfFrameState = TF_FRAME_STATE_WAIT_START1;
    // tfReceivePosition = 0;
}

static int ParseStateLine(const char* line) {
    if (memcmp(line, "State;", 6) != 0) return -1;
    return atoi(line + 6);
}

static int32_t ParseDataLine(const char* line) {
    if (memcmp(line, "d:", 2) != 0) return -1;
    return strtoul(line + 2, NULL, 10);
}

void lidarVL53Update(rangefinderDev_t *dev)
{
    UNUSED(dev);

    if (vl53SerialPort == NULL) return;

    while (serialRxBytesWaiting(vl53SerialPort)) {
        uint8_t c = serialRead(vl53SerialPort);

        if (vlFramePos >= MAX_FRAME_LEN) {
            vlFramePos = 0; // line buffer overflow
        }

        vlFrameBuf[vlFramePos] = c;
        vlFramePos += 1;

        if (c != 0x0A) {
            continue;
        }

        // end of line, parse and process line
        switch (vlReceiveStage) {
        case STAGE_WAIT_STATE_1:
            if (ParseStateLine((const char*)vlFrameBuf) == 0) {
                vlReceiveStage = STAGE_WAIT_DATA;
            }
            break;
        case STAGE_WAIT_DATA:
            vlDistanceMM = ParseDataLine((const char*)vlFrameBuf);
            if (vlDistanceMM >= 0) {
                vlReceiveStage = STAGE_WAIT_STATE_2;
            } else {
                vlReceiveStage = STAGE_WAIT_STATE_1;
            }
            break; 
        case STAGE_WAIT_STATE_2:
            if (ParseStateLine((const char*)vlFrameBuf) == 0) {
                vlDistance = vlDistanceMM / 10;
                vlReceiveStage = STAGE_WAIT_DATA;
            } else {
                vlReceiveStage = STAGE_WAIT_STATE_1;    
            }
            break;
        default:
            vlReceiveStage = STAGE_WAIT_STATE_1;
            break;
        }

        vlFramePos = 0;
    }
}

int32_t lidarVL53GetDistance(rangefinderDev_t *dev)
{
    UNUSED(dev);
    int32_t value = vlDistance;
    vlDistance = RANGEFINDER_NO_NEW_DATA;
    return value;
}

bool lidarVL53Detect(rangefinderDev_t *dev) {
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_LIDAR_TF);
    if (!portConfig) {
        return false;
    }

    vl53SerialPort = openSerialPort(portConfig->identifier, FUNCTION_LIDAR_TF, NULL, NULL, 115200, MODE_RXTX, 0);
    if (vl53SerialPort == NULL) {
        return false;
    }

    dev->delayMs = 10;
    dev->maxRangeCm = 200;
    dev->detectionConeDeciDegrees = 900;
    dev->detectionConeExtendedDeciDegrees = 900;

    dev->init = &lidarVL53Init;
    dev->update = &lidarVL53Update;
    dev->read = &lidarVL53GetDistance;
    return true;
}
