#include "BleControl.h"

#if defined(ARDUINO_ARCH_ESP32)

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

namespace {
constexpr const char *kServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
constexpr const char *kCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";

RemoteUpdateCallback s_callback = nullptr;
BLEAdvertising *s_advertising = nullptr;
bool s_connected = false;

uint16_t readLE16(const uint8_t *data) {
  return static_cast<uint16_t>(data[0]) |
         static_cast<uint16_t>(data[1] << 8);
}

RemoteControlUpdate decodePayload(const uint8_t *data, size_t len) {
  RemoteControlUpdate update;
  if (!data || len == 0) {
    return update;
  }

  update.hasAmplitude = true;
  update.amplitude = static_cast<float>(data[0]);

  if (len == 3) {
    update.hasPeriod = true;
    update.period = static_cast<float>(readLE16(&data[1]));
  } else if (len >= 5) {
    update.hasFrequency = true;
    update.frequency = static_cast<float>(readLE16(&data[1]));
    update.hasPeriod = true;
    update.period = static_cast<float>(readLE16(&data[3]));
  } else {
    if (len >= 2) {
      update.hasFrequency = true;
      update.frequency = static_cast<float>(data[1]);
    }
    if (len >= 4) {
      update.hasPeriod = true;
      update.period = static_cast<float>(readLE16(&data[2]));
    } else if (len >= 3) {
      update.hasPeriod = true;
      update.period = static_cast<float>(data[2]);
    }
  }

  return update;
}

struct ControlServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    s_connected = true;
  }

  void onDisconnect(BLEServer *) override {
    s_connected = false;
    if (s_advertising) {
      BLEDevice::startAdvertising();
    }
  }
};

struct ControlCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    if (!s_callback) {
      return;
    }
    std::string value = characteristic->getValue();
    if (value.empty()) {
      return;
    }
    const uint8_t *data = reinterpret_cast<const uint8_t *>(value.data());
    RemoteControlUpdate update = decodePayload(data, value.size());
    if (update.hasAmplitude || update.hasFrequency || update.hasPeriod) {
      s_callback(update);
    }
  }
};

ControlServerCallbacks s_serverCallbacks;
ControlCharacteristicCallbacks s_characteristicCallbacks;

}  // namespace

void BleControl_begin(RemoteUpdateCallback callback) {
  s_callback = callback;
  BLEDevice::init("BOS WFS Controller");

  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(&s_serverCallbacks);

  BLEService *service = server->createService(kServiceUuid);
  BLECharacteristic *characteristic = service->createCharacteristic(
      kCharacteristicUuid,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  characteristic->setCallbacks(&s_characteristicCallbacks);

  service->start();

  s_advertising = BLEDevice::getAdvertising();
  s_advertising->addServiceUUID(kServiceUuid);
  s_advertising->setScanResponse(true);
  BLEDevice::startAdvertising();
}

void BleControl_poll() {
  (void)s_connected;
  (void)s_advertising;
}

bool BleControl_isConnected() {
  return s_connected;
}

#else  // !ARDUINO_ARCH_ESP32

void BleControl_begin(RemoteUpdateCallback) {}
void BleControl_poll() {}
bool BleControl_isConnected() { return false; }

#endif
