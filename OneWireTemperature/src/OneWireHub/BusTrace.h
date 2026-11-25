#pragma once

#include <stdint.h>

enum class BusEvent : uint8_t { Presence, Rom, SearchComplete, SearchMismatch, Function, Error };

// Record an event without writing to UART during a 1-Wire transaction.
void recordBusEvent(BusEvent event, uint8_t value = 0);
