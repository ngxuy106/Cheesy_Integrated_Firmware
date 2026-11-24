#ifndef RAM_MONITOR_H
#define RAM_MONITOR_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define RAM_MONITOR_MAX_VARIABLES 8U
#define RAM_MONITOR_SRAM_START    0x20000000UL
#define RAM_MONITOR_SRAM_END      0x20020000UL
#define RAM_MONITOR_PERIOD_MS     50U

typedef bool (*RAM_MonitorTx_t)(const uint8_t *data, uint16_t length);

void RAM_Monitor_Init(RAM_MonitorTx_t tx);

/* Returns true when the byte belongs to a binary subscription frame. */
bool RAM_Monitor_ProcessByte(uint8_t byte);

/* Call from the main loop; sends at most one frame when due. */
void RAM_Monitor_StreamDue(uint32_t now);

#endif
