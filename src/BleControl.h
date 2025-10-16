#pragma once

#include <Arduino.h>

struct RemoteControlUpdate {
  bool hasAmplitude = false;
  float amplitude = 0.0f;
  bool hasFrequency = false;
  float frequency = 0.0f;
  bool hasPeriod = false;
  float period = 0.0f;
};

using RemoteUpdateCallback = void (*)(const RemoteControlUpdate &update);

void BleControl_begin(RemoteUpdateCallback callback);
void BleControl_poll();
bool BleControl_isConnected();

