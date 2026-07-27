/** @file diagnostics_can.h  Nonblocking CAN status publication. */
#ifndef DIAGNOSTICS_CAN_H
#define DIAGNOSTICS_CAN_H

#include "engine_control.h"
#include "main.h"

#include <stdint.h>

typedef struct
{
  int32_t mcu_temperature_c;
  uint16_t external_vbatt_raw;
  uint16_t vbus_raw;
  uint16_t vdda_mv;
} diagnostics_sensor_snapshot_t;

/** Configure diagnostics and reject unused incoming CAN traffic. */
HAL_StatusTypeDef diagnostics_can_init(FDCAN_HandleTypeDef *fdcan);

/** Publish 0x100..0x102 at a bounded rate; never waits for FIFO space. */
void diagnostics_can_service(uint32_t now_ms,
                             const engine_control_state_t *engine,
                             const diagnostics_sensor_snapshot_t *sensors);

uint32_t diagnostics_can_tx_drops(void);

#endif /* DIAGNOSTICS_CAN_H */
