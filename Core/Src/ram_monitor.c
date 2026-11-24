#include "ram_monitor.h"
#include <string.h>

#define RAM_MONITOR_CONFIG_HEADER 0x56U
#define RAM_MONITOR_STREAM_HEADER 0x55U
#define RAM_MONITOR_CONFIG_MAX_PAYLOAD (1U + RAM_MONITOR_MAX_VARIABLES * 5U)
#define RAM_MONITOR_PAYLOAD_MAX (4U + RAM_MONITOR_MAX_VARIABLES * 4U)

typedef struct
{
    uint32_t address;
    uint8_t size;
} RAM_MonitorVariable_t;

static RAM_MonitorTx_t monitor_tx;
static RAM_MonitorVariable_t variables[RAM_MONITOR_MAX_VARIABLES];
static uint8_t variable_count;
static uint32_t last_stream_tick;

static uint8_t rx_state;
static uint8_t rx_length;
static uint8_t rx_index;
static uint8_t rx_checksum;
static uint8_t rx_payload[RAM_MONITOR_CONFIG_MAX_PAYLOAD];

static uint8_t checksum(const uint8_t *data, uint8_t length)
{
    uint8_t result = 0U;
    for (uint8_t i = 0U; i < length; i++) result = (uint8_t)(result + data[i]);
    return result;
}

static bool valid_range(uint32_t address, uint8_t size)
{
    if (size != 1U && size != 2U && size != 4U) return false;
    return address >= RAM_MONITOR_SRAM_START &&
           address <= RAM_MONITOR_SRAM_END - (uint32_t)size;
}

static void reset_rx(void)
{
    rx_state = 0U;
    rx_length = 0U;
    rx_index = 0U;
    rx_checksum = 0U;
}

static void apply_subscription(void)
{
    uint8_t count = rx_payload[0];
    if (count > RAM_MONITOR_MAX_VARIABLES ||
        rx_length != (uint8_t)(1U + count * 5U)) {
        return;
    }

    RAM_MonitorVariable_t next[RAM_MONITOR_MAX_VARIABLES];
    for (uint8_t i = 0U; i < count; i++) {
        uint8_t offset = (uint8_t)(1U + i * 5U);
        uint32_t address = (uint32_t)rx_payload[offset] |
                           ((uint32_t)rx_payload[offset + 1U] << 8U) |
                           ((uint32_t)rx_payload[offset + 2U] << 16U) |
                           ((uint32_t)rx_payload[offset + 3U] << 24U);
        uint8_t size = rx_payload[offset + 4U];
        if (!valid_range(address, size)) return;
        next[i].address = address;
        next[i].size = size;
    }
    memcpy(variables, next, sizeof(next));
    variable_count = count;
}

void RAM_Monitor_Init(RAM_MonitorTx_t tx)
{
    monitor_tx = tx;
    variable_count = 0U;
    last_stream_tick = HAL_GetTick();
    reset_rx();
}

bool RAM_Monitor_ProcessByte(uint8_t byte)
{
    switch (rx_state) {
    case 0U:
        if (byte == 0xAAU) {
            rx_state = 1U;
            return true;
        }
        return false;
    case 1U:
        if (byte == RAM_MONITOR_CONFIG_HEADER) {
            rx_state = 2U;
            return true;
        }
        rx_state = (byte == 0xAAU) ? 1U : 0U;
        return true;
    case 2U:
        rx_length = byte;
        if (rx_length == 0U || rx_length > RAM_MONITOR_CONFIG_MAX_PAYLOAD) {
            reset_rx();
            return true;
        }
        rx_index = 0U;
        rx_state = 3U;
        return true;
    case 3U:
        rx_payload[rx_index++] = byte;
        if (rx_index >= rx_length) rx_state = 4U;
        return true;
    case 4U:
        rx_checksum = byte;
        if (rx_checksum == checksum(rx_payload, rx_length)) apply_subscription();
        reset_rx();
        return true;
    default:
        reset_rx();
        return true;
    }
}

void RAM_Monitor_StreamDue(uint32_t now)
{
    if (monitor_tx == NULL || variable_count == 0U ||
        (now - last_stream_tick) < RAM_MONITOR_PERIOD_MS) {
        return;
    }
    last_stream_tick = now;

    uint8_t payload[RAM_MONITOR_PAYLOAD_MAX];
    uint8_t payload_length = 4U;
    payload[0] = (uint8_t)now;
    payload[1] = (uint8_t)(now >> 8U);
    payload[2] = (uint8_t)(now >> 16U);
    payload[3] = (uint8_t)(now >> 24U);

    for (uint8_t i = 0U; i < variable_count; i++) {
        memcpy(&payload[payload_length],
               (const void *)(uintptr_t)variables[i].address,
               variables[i].size);
        payload_length = (uint8_t)(payload_length + variables[i].size);
    }

    uint8_t frame[2U + 1U + RAM_MONITOR_PAYLOAD_MAX + 1U];
    frame[0] = 0xAAU;
    frame[1] = RAM_MONITOR_STREAM_HEADER;
    frame[2] = payload_length;
    memcpy(&frame[3], payload, payload_length);
    frame[3U + payload_length] = checksum(payload, payload_length);
    (void)monitor_tx(frame, (uint16_t)(4U + payload_length));
}
