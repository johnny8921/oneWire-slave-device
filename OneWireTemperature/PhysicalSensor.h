#pragma once

#include <Arduino.h>
#include <OneWire.h>
#include <optional>
#include "src/OneWireHub/DS18B20.h"

class SensorRegistry {
public:
  struct Reading {
    uint8_t rom[8];
    int16_t rawTemperature;
    uint32_t updatedMs;
    uint8_t sources;
    uint8_t lastChannel;
    std::optional<DS18B20> proxy;
  };

  static constexpr uint8_t maxReadings = 16;

  explicit SensorRegistry(OneWireHub &hub) : hub(hub) {}

  uint8_t count() const { return readingCount; }
  const Reading &reading(uint8_t index) const { return readings[index]; }

  bool update(uint8_t channel, const uint8_t rom[8], int16_t raw, uint32_t now) {
    uint8_t index = 0;
    while (index < readingCount && memcmp(readings[index].rom, rom, 8) != 0) ++index;
    if (index == maxReadings) return false;

    Reading &entry = readings[index];
    if (!entry.proxy) {
      entry.proxy.emplace(rom[0], rom[1], rom[2], rom[3], rom[4], rom[5], rom[6]);
      entry.proxy->setTemperatureRaw(raw);
      if (hub.attach(*entry.proxy) == 255) {
        entry.proxy.reset();
        return false;
      }
    } else {
      entry.proxy->setTemperatureRaw(raw);
    }
    if (index == readingCount) {
      memcpy(entry.rom, rom, sizeof(entry.rom));
      ++readingCount;
    }
    entry.rawTemperature = raw;
    entry.updatedMs = now;
    entry.sources |= 1u << channel;
    entry.lastChannel = channel;
    return true;
  }

  bool disconnect(uint8_t channel) {
    const uint8_t source = 1u << channel;
    for (uint8_t index = 0; index < readingCount; ++index) {
      Reading &entry = readings[index];
      if (!(entry.sources & source)) continue;
      entry.sources &= ~source;
      if (entry.sources == 0) {
        hub.detach(*entry.proxy);
        entry.proxy.reset();
      }
      return true;
    }
    return false;
  }

private:
  OneWireHub &hub;
  Reading readings[maxReadings]{};
  uint8_t readingCount = 0;
};

class PhysicalSensor {
public:
  explicit PhysicalSensor(uint8_t pin) : pin(pin) {}

  void begin() { bus.begin(pin); }

  bool ready(uint32_t now) const {
    return (state != State::Probe && state != State::WaitConversion) ||
           static_cast<int32_t>(now - nextActionMs) >= 0;
  }

  // Returns true when a new measurement arrives or a sensor goes offline.
  bool poll(uint32_t now, SensorRegistry &registry, uint8_t channel) {
    if (!ready(now)) return false;
    if (state == State::Probe) {
      if (!startTransfer(State::ReadRom, 0x33, false, sizeof(rom))) return disconnect(now, registry, channel);
      return false;
    }
    if (state == State::StartConvert) {
      if (!startTransfer(State::Convert, 0x44, true, 0)) return disconnect(now, registry, channel);
      return false;
    }
    if (state == State::WaitConversion) {
      if (!startTransfer(State::ReadScratchpad, 0xBE, true, sizeof(received))) return disconnect(now, registry, channel);
      return false;
    }

    // Service at most one slot before returning to the slave bus.
    if (bitPosition < txBits) {
      bus.write_bit((transmit[bitPosition / 8] >> (bitPosition % 8)) & 1);
      bus.depower();
      ++bitPosition;
      return false;
    }
    if (bitPosition < txBits + rxBits) {
      const uint8_t rxBit = bitPosition++ - txBits;
      received[rxBit / 8] |= bus.read_bit() << (rxBit % 8);
      return false;
    }

    if (state == State::ReadRom) {
      if (received[0] != DS18B20::family_code || OneWire::crc8(received, sizeof(rom)) != 0) {
        return disconnect(now, registry, channel);
      }
      memcpy(rom, received, sizeof(rom));
      state = State::StartConvert;
      return false;
    }
    if (state == State::Convert) {
      nextActionMs = now + 750;
      state = State::WaitConversion;
      return false;
    }

    const int16_t raw = static_cast<int16_t>(uint16_t(received[0]) | (uint16_t(received[1]) << 8));
    if (OneWire::crc8(received, sizeof(received)) != 0 || (received[4] & 0x9F) != 0x1F ||
        raw < -55 * 16 || raw > 125 * 16) {
      return disconnect(now, registry, channel);
    }

    if (!registry.update(channel, rom, raw, now)) return disconnect(now, registry, channel);
    state = State::StartConvert;
    return true;
  }

private:
  enum class State : uint8_t { Probe, ReadRom, StartConvert, Convert, WaitConversion, ReadScratchpad };

  bool startTransfer(State next, uint8_t command, bool addressed, uint8_t readBytes) {
    if (!bus.reset()) return false;

    uint8_t size = 0;
    if (addressed) {
      transmit[size++] = 0x55;
      memcpy(transmit + size, rom, sizeof(rom));
      size += sizeof(rom);
    }
    transmit[size++] = command;
    memset(received, 0, sizeof(received));
    txBits = size * 8;
    rxBits = readBytes * 8;
    bitPosition = 0;
    state = next;
    return true;
  }

  bool disconnect(uint32_t now, SensorRegistry &registry, uint8_t channel) {
    state = State::Probe;
    nextActionMs = now + 500;
    return registry.disconnect(channel);
  }

  uint8_t pin;
  OneWire bus;
  uint8_t rom[8];
  uint8_t transmit[10];
  uint8_t received[9];
  uint8_t txBits = 0;
  uint8_t rxBits = 0;
  uint8_t bitPosition = 0;
  uint32_t nextActionMs = 0;
  State state = State::Probe;
};
