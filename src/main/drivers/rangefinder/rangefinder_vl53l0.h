#pragma once

#include "common/time.h"

int32_t lidarVL53GetDistance(rangefinderDev_t *dev);

bool lidarVL53Detect(rangefinderDev_t *dev);
