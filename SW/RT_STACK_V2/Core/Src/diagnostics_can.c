/** @file diagnostics_can.c  Compact engine/sensor/fault CAN telemetry. */
#include "diagnostics_can.h"

#include <math.h>
#include <stddef.h>

#define CAN_ID_RT_STATUS          0x100U
#define CAN_ID_ENGINE_STATUS      0x101U
#define CAN_ID_FAULT_STATUS       0x102U
#define CAN_TX_PERIOD_MS             10U

static FDCAN_HandleTypeDef *can_handle;
static uint32_t last_tx_ms;
static uint32_t tx_drop_count;

static uint16_t clamp_u16(float value)
{
  if (!isfinite(value) || (value <= 0.0f))
  {
    return 0U;
  }
  if (value >= 65535.0f)
  {
    return UINT16_MAX;
  }
  return (uint16_t)(value + 0.5f);
}

static uint8_t saturate_u8(uint32_t value)
{
  return (value > UINT8_MAX) ? UINT8_MAX : (uint8_t)value;
}

static int8_t saturate_i8(int32_t value)
{
  if (value > INT8_MAX)
  {
    return INT8_MAX;
  }
  if (value < INT8_MIN)
  {
    return INT8_MIN;
  }
  return (int8_t)value;
}

static HAL_StatusTypeDef send_frame(uint32_t identifier, const uint8_t data[8])
{
  FDCAN_TxHeaderTypeDef header = {0};

  if ((can_handle == NULL) ||
      (HAL_FDCAN_GetTxFifoFreeLevel(can_handle) == 0U))
  {
    if (tx_drop_count != UINT32_MAX)
    {
      ++tx_drop_count;
    }
    return HAL_BUSY;
  }

  header.Identifier = identifier;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = FDCAN_DLC_BYTES_8;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_OFF;
  header.FDFormat = FDCAN_CLASSIC_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header.MessageMarker = 0U;

  if (HAL_FDCAN_AddMessageToTxFifoQ(can_handle, &header, data) != HAL_OK)
  {
    if (tx_drop_count != UINT32_MAX)
    {
      ++tx_drop_count;
    }
    return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef diagnostics_can_init(FDCAN_HandleTypeDef *fdcan)
{
  can_handle = fdcan;
  last_tx_ms = 0U;
  tx_drop_count = 0U;
  if (fdcan == NULL)
  {
    return HAL_ERROR;
  }

  return HAL_FDCAN_ConfigGlobalFilter(fdcan,
                                      FDCAN_REJECT,
                                      FDCAN_REJECT,
                                      FDCAN_REJECT_REMOTE,
                                      FDCAN_REJECT_REMOTE);
}

void diagnostics_can_service(uint32_t now_ms,
                             const engine_control_state_t *engine,
                             const diagnostics_sensor_snapshot_t *sensors)
{
  uint8_t data[8] = {0};
  uint16_t value;
  uint32_t capture_losses;

  if ((engine == NULL) || (sensors == NULL) ||
      ((now_ms - last_tx_ms) < CAN_TX_PERIOD_MS))
  {
    return;
  }
  last_tx_ms = now_ms;

  /* 0x100 remains backward-compatible with RT_STACK_V2.dbc. */
  value = clamp_u16(engine->angle_deg * 10.0f);
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8);
  data[2] = (uint8_t)saturate_i8(sensors->mcu_temperature_c);
  data[3] = 0U;
  data[4] = (uint8_t)sensors->external_vbatt_raw;
  data[5] = (uint8_t)(sensors->external_vbatt_raw >> 8);
  data[6] = (uint8_t)sensors->vbus_raw;
  data[7] = (uint8_t)(sensors->vbus_raw >> 8);
  (void)send_frame(CAN_ID_RT_STATUS, data);

  value = clamp_u16(engine->rpm);
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8);
  data[2] = (uint8_t)engine->mode;
  data[3] = (engine->crank_synced ? 0x01U : 0U) |
            (engine->phase_synced ? 0x02U : 0U) |
            (engine->outputs_requested ? 0x04U : 0U) |
            (engine->outputs_enabled ? 0x08U : 0U);
  data[4] = (uint8_t)engine->sync_epoch;
  data[5] = (uint8_t)(engine->sync_epoch >> 8);
  data[6] = (uint8_t)sensors->vdda_mv;
  data[7] = (uint8_t)(sensors->vdda_mv >> 8);
  (void)send_frame(CAN_ID_ENGINE_STATUS, data);

  /* Engine faults currently occupy eight bits; give trigger reasons sixteen
   * so TRIGGER_LOSS_FORCED (bit 8) is observable rather than truncated. */
  data[0] = (uint8_t)engine->latched_faults;
  data[1] = (uint8_t)engine->trigger_latched_faults;
  data[2] = (uint8_t)(engine->trigger_latched_faults >> 8);
  data[3] = (uint8_t)engine->ignition_faults;
  data[4] = (uint8_t)engine->injection_faults;
  data[5] = (uint8_t)(engine->injection_faults >> 8);
  capture_losses = engine->capture_dropped_events + engine->capture_overruns;
  if (capture_losses < engine->capture_dropped_events)
  {
    capture_losses = UINT32_MAX;
  }
  data[6] = saturate_u8(capture_losses);
  data[7] = saturate_u8(tx_drop_count);
  (void)send_frame(CAN_ID_FAULT_STATUS, data);
}

uint32_t diagnostics_can_tx_drops(void)
{
  return tx_drop_count;
}
