#include <ESP8266WiFi.h>
#include <U8x8lib.h>
#include <Wire.h>
#include "PhysicalSensor.h"
#include "src/OneWireHub/BusTrace.h"
#include "src/OneWireHub/DS18B20.h"
#include "src/OneWireHub/OneWireHub.h"

constexpr uint8_t kOneWirePin = D2;  // NodeMCU Amica: GPIO4
constexpr uint8_t kFlashDioSensorPin = 10;  // SD3 on NodeMCU Amica, available in DIO mode.
constexpr uint8_t kOledSda = D6;
constexpr uint8_t kOledScl = D5;
constexpr bool kTestD2 = false;
constexpr uint32_t kTestHalfPeriodMs = 50;
constexpr uint32_t kLedIntervalMs = 500;

OneWireHub hub(kOneWirePin);
SensorRegistry physicalSensors(hub);
PhysicalSensor sensorBuses[] = {
  PhysicalSensor(D1), PhysicalSensor(D0), PhysicalSensor(D3),
  PhysicalSensor(D4), PhysicalSensor(D7), PhysicalSensor(kFlashDioSensorPin),
};
constexpr uint8_t kSensorBusCount = sizeof(sensorBuses) / sizeof(sensorBuses[0]);
constexpr const char *kSensorBusNames[] = {"D1", "D0", "D3", "D4", "D7", "SD3"};
static_assert(sizeof(kSensorBusNames) / sizeof(kSensorBusNames[0]) == kSensorBusCount,
              "Give each sensor bus a display name");
uint8_t enabledSensorBuses = kSensorBusCount;
uint8_t nextSensorBus = 0;

U8X8_SH1106_128X64_NONAME_HW_I2C oled(U8X8_PIN_NONE, kOledScl, kOledSda);
constexpr uint8_t kOledRowsPerColumn = 5;
constexpr uint8_t kOledColumns = 16;
constexpr uint8_t kOledCells = kOledColumns * 8;
constexpr uint8_t kOledDegreeGlyph = 0xB0;
constexpr uint8_t kSensorsPerTable = 2 * kOledRowsPerColumn;
constexpr uint32_t kOledPageIntervalMs = 5000;
constexpr uint32_t kOledRefreshIntervalMs = 100;
// Low byte is the glyph. High byte selects a quadrant for double-size text.
uint16_t oledTarget[kOledCells];
uint16_t oledDisplayed[kOledCells];
uint8_t oledNextCell = kOledCells;
uint8_t oledPage = 0;
uint32_t lastOledPageMs = 0;
uint32_t lastOledRefreshMs = 0;
bool oledReady = false;

uint32_t lastLedToggleMs = 0;
bool ledOn = false;
uint32_t d2TestStartMs = 0;
bool d2TestHigh = true;

struct TraceEntry {
  BusEvent event;
  uint8_t value;
};

constexpr uint8_t kTraceCapacity = 32;
TraceEntry trace[kTraceCapacity];
uint8_t traceRead = 0;
uint8_t traceWrite = 0;

void recordBusEvent(BusEvent event, uint8_t value) {
  const uint8_t next = (traceWrite + 1) % kTraceCapacity;
  if (next == traceRead) return;
  trace[traceWrite] = {event, value};
  traceWrite = next;
}

void printBusEvent() {
  if (traceRead == traceWrite || Serial.availableForWrite() < 48) return;

  const TraceEntry entry = trace[traceRead];
  traceRead = (traceRead + 1) % kTraceCapacity;
  switch (entry.event) {
    case BusEvent::Presence:
      Serial.println(F("1W RESET -> PRESENCE"));
      break;
    case BusEvent::Rom:
      Serial.printf("1W ROM 0x%02X\r\n", unsigned(entry.value));
      break;
    case BusEvent::SearchComplete:
      Serial.printf("1W SEARCH complete: ROM byte 1 = %02X\r\n", unsigned(entry.value));
      break;
    case BusEvent::SearchMismatch:
      Serial.printf("1W SEARCH branch mismatch at bit %u\r\n", unsigned(entry.value));
      break;
    case BusEvent::Function:
      Serial.printf("1W DS18B20 0x%02X\r\n", unsigned(entry.value));
      break;
    case BusEvent::Error:
      Serial.printf("1W ERROR %u\r\n", unsigned(entry.value));
      break;
  }
}

void setupOled() {
  Wire.begin(kOledSda, kOledScl);
  for (uint8_t address = 0x3C; address <= 0x3D; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() != 0) continue;

    oled.setI2CAddress(address << 1);
    oled.setBusClock(400000);
    oled.begin();
    oled.setFont(u8x8_font_chroma48medium8_r);
    for (uint16_t &cell : oledDisplayed) cell = ' ';
    oledReady = true;
    Serial.printf("OLED: SH1106 at 0x%02X\r\n", unsigned(address));
    return;
  }
  Serial.println(F("OLED: no I2C response at 0x3C or 0x3D"));
}

void putOledText(uint8_t column, uint8_t row, const char *text) {
  while (*text && column < kOledColumns) {
    oledTarget[row * kOledColumns + column++] = uint8_t(*text++);
  }
}

void putOledLargeText(uint8_t column, uint8_t row, const char *text) {
  while (*text && column + 1 < kOledColumns) {
    for (uint8_t quadrant = 0; quadrant < 4; ++quadrant) {
      const uint8_t cell = (row + quadrant / 2) * kOledColumns + column + quadrant % 2;
      oledTarget[cell] = ((quadrant + 1) << 8) | uint8_t(*text);
    }
    column += 2;
    ++text;
  }
}

uint8_t tablePages() {
  const uint8_t count = physicalSensors.count();
  return count == 0 ? 1 : (count + kSensorsPerTable - 1) / kSensorsPerTable;
}

void renderOled(uint32_t now) {
  for (uint16_t &cell : oledTarget) cell = ' ';
  const uint8_t count = physicalSensors.count();
  const uint8_t tables = tablePages();
  char line[kOledColumns + 1];

  if (oledPage < tables) {
    putOledText(0, 0, "DS18B20 C");
    if (count == 0) putOledText(0, 3, "No sensors");
    const size_t first = oledPage * kSensorsPerTable;
    for (uint8_t i = 0; i < kSensorsPerTable && first + i < count; ++i) {
      const uint8_t sensor = first + i;
      const auto &reading = physicalSensors.reading(sensor);
      if (reading.sources == 0 && (now / 500) % 2 != 0) continue;
      const double temperature = reading.rawTemperature / 16.0;
      // A sign or third digit uses the space after the sensor number.
      const char *format = temperature <= -9.995 || temperature >= 99.995 ? "%02u%6.2f" : "%02u %5.2f";
      snprintf(line, sizeof(line), format, unsigned(sensor + 1), temperature);
      putOledText((i / kOledRowsPerColumn) * 8, 2 + i % kOledRowsPerColumn, line);
    }
  } else {
    const uint8_t sensor = oledPage - tables;
    const auto &reading = physicalSensors.reading(sensor);
    const uint32_t ageMs = now - reading.updatedMs;
    snprintf(line, sizeof(line), "SENSOR %02u", unsigned(sensor + 1));
    putOledText(0, 0, line);
    uint8_t channel = reading.lastChannel;
    if (reading.sources != 0 && !(reading.sources & (1u << channel))) {
      // The same ROM is still online on another input.
      channel = __builtin_ctz(static_cast<unsigned>(reading.sources));
    }
    const char *pinName = kSensorBusNames[channel];
    const int temperatureLength = snprintf(line, sizeof(line), "%.2f%cC", reading.rawTemperature / 16.0,
                                           kOledDegreeGlyph);
    putOledLargeText((kOledColumns - 2 * temperatureLength) / 2, 2, line);
    for (uint8_t i = 0; i < 8; ++i) {
      snprintf(line + 2 * i, 3, "%02X", unsigned(reading.rom[i]));
    }
    putOledText(0, 5, line);
    putOledText(0, 7, reading.sources != 0 ? "OK" : "ERROR");
    putOledText((kOledColumns - strlen(pinName)) / 2, 7, pinName);
    const unsigned long ageSeconds = ageMs / 1000;
    const int ageLength = ageSeconds < 1000000
        ? snprintf(line, sizeof(line), "%02lus", ageSeconds)
        : snprintf(line, sizeof(line), "%luh", ageSeconds / 3600);
    putOledText(kOledColumns - ageLength, 7, line);
  }

  snprintf(line, sizeof(line), "%02u/%02u", unsigned(oledPage + 1), unsigned(tables + count));
  putOledText(11, 0, line);
  oledNextCell = 0;
  lastOledRefreshMs = now;
}

void updateOled() {
  // Send at most one changed tile before returning to the 1-Wire poll.
  while (oledNextCell < kOledCells) {
    const uint8_t cell = oledNextCell++;
    const uint16_t value = oledTarget[cell];
    if (oledDisplayed[cell] == value) continue;

    const uint8_t x = cell % kOledColumns;
    const uint8_t y = cell / kOledColumns;
    const uint8_t glyph = value & 0xFF;
    if (value >> 8) {
      const uint8_t quadrant = (value >> 8) - 1;
      const uint8_t sourceColumn = 4 * (quadrant % 2);
      const uint8_t shift = 8 * (quadrant / 2);
      uint8_t source[8];
      uint8_t tile[8];
      if (glyph == kOledDegreeGlyph) {
        // The bold font contains only ASCII. Supply an 8x8 degree glyph.
        constexpr uint8_t degree[8] = {0, 0, 0x06, 0x09, 0x09, 0x06, 0, 0};
        memcpy(source, degree, sizeof(source));
      } else {
        oled.setFont(u8x8_font_torussansbold8_r);
        u8x8_get_glyph_data(oled.getU8x8(), glyph, source, 0);
        oled.setFont(u8x8_font_chroma48medium8_r);
      }
      for (uint8_t i = 0; i < 8; ++i) {
        tile[i] = u8x8_upscale_byte(source[sourceColumn + i / 2]) >> shift;
      }
      oled.drawTile(x, y, 1, tile);
    } else {
      oled.drawGlyph(x, y, glyph);
    }
    oledDisplayed[cell] = value;
    return;
  }
}

void setup() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);

  Serial.begin(115200);
  if (kTestD2) {
    digitalWrite(LED_BUILTIN, HIGH);  // The built-in LED is active-low.
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(kOneWirePin, INPUT_PULLUP);
    Serial.println(F("D2 (GPIO4) test: 10 Hz, 50 ms high / 50 ms low"));
    d2TestStartMs = lastLedToggleMs = millis();
    return;
  }
  Serial.println(F("1W trace ready on D2 (GPIO4)"));
  if (ESP.getFlashChipMode() != FM_DIO) {
    --enabledSensorBuses;
    Serial.println(F("DS18B20: SD3 disabled; flash must use DIO"));
  }
  for (uint8_t channel = 0; channel < enabledSensorBuses; ++channel) {
    sensorBuses[channel].begin();
  }
  Serial.printf("DS18B20 input buses: %u (D1 D0 D3 D4 D7 SD3)\r\n", unsigned(enabledSensorBuses));

  setupOled();
  lastLedToggleMs = lastOledPageMs = millis();
  if (oledReady) renderOled(lastOledPageMs);
}

bool pollSensorBuses(uint32_t now) {
  for (uint8_t i = 0; i < enabledSensorBuses; ++i) {
    const uint8_t channel = nextSensorBus;
    nextSensorBus = (nextSensorBus + 1) % enabledSensorBuses;
    if (sensorBuses[channel].ready(now)) {
      return sensorBuses[channel].poll(now, physicalSensors, channel);
    }
  }
  return false;
}

void loop() {
  if (!kTestD2) {
    hub.poll();
    const Error busError = hub.getError();
    if (busError != Error::NO_ERROR && busError != Error::FIRST_TIMESLOT_TIMEOUT) {
      recordBusEvent(BusEvent::Error, static_cast<uint8_t>(busError));
    }
    printBusEvent();
  }

  const uint32_t now = millis();
  if (kTestD2 && now - lastLedToggleMs >= kLedIntervalMs) {
    lastLedToggleMs = now;
    ledOn = !ledOn;
    digitalWrite(LED_BUILTIN, ledOn ? LOW : HIGH);
  }

  if (kTestD2) {
    if (now - d2TestStartMs >= kTestHalfPeriodMs) {
      d2TestStartMs = now;
      d2TestHigh = !d2TestHigh;
      digitalWrite(kOneWirePin, LOW);
      pinMode(kOneWirePin, d2TestHigh ? INPUT_PULLUP : OUTPUT_OPEN_DRAIN);
    }
    return;
  }

  const uint8_t previousCount = physicalSensors.count();
  const bool sensorChanged = pollSensorBuses(now);
  const uint8_t count = physicalSensors.count();
  if (oledReady) {
    const uint32_t displayNow = millis();
    const uint32_t elapsedPages = (displayNow - lastOledPageMs) / kOledPageIntervalMs;
    if (count != previousCount) {
      oledPage = 0;
      lastOledPageMs = displayNow;
      renderOled(displayNow);
    } else if (elapsedPages != 0) {
      lastOledPageMs += elapsedPages * kOledPageIntervalMs;
      oledPage = (oledPage + elapsedPages) % (tablePages() + count);
      renderOled(displayNow);
    } else if (sensorChanged || (count != 0 && oledNextCell == kOledCells &&
                                 displayNow - lastOledRefreshMs >= kOledRefreshIntervalMs)) {
      renderOled(displayNow);
    }
    updateOled();
  }
}
