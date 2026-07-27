/**
 ******************************************************************************
 * @file    engine_watchdog_stm32.c
 * @brief   Direct-register STM32G4 independent-watchdog driver.
 ******************************************************************************
 */
#include "engine_watchdog_stm32.h"

#include "board_config.h"
#include "main.h"

#define IWDG_KEY_START          0xCCCCU
#define IWDG_KEY_WRITE_ACCESS   0x5555U
#define IWDG_KEY_RELOAD         0xAAAAU
#define IWDG_UPDATE_FLAGS       (IWDG_SR_PVU | IWDG_SR_RVU | IWDG_SR_WVU)

#if (BOARD_IWDG_RELOAD_VALUE > 0x0FFFU)
#error "BOARD_IWDG_RELOAD_VALUE must fit the 12-bit STM32 IWDG reload register"
#endif

bool engine_watchdog_stm32_start(void)
{
  /* Starting IWDG also starts LSI. The watchdog cannot be disabled again
   * without a reset, which is intentional for this safety boundary. */
  IWDG->KR = IWDG_KEY_START;
  IWDG->KR = IWDG_KEY_WRITE_ACCESS;
  IWDG->PR = BOARD_IWDG_PRESCALER_BITS;
  IWDG->RLR = BOARD_IWDG_RELOAD_VALUE;

  /* Register updates cross into the LSI clock domain. Bound this wait without
   * relying on SysTick; if it fails, the already-running watchdog resets the
   * part instead of leaving initialization trapped forever. */
  uint32_t spins = SystemCoreClock / 2U;
  while (((IWDG->SR & IWDG_UPDATE_FLAGS) != 0U) && (spins != 0U))
  {
    --spins;
    __NOP();
  }
  if ((IWDG->SR & IWDG_UPDATE_FLAGS) != 0U)
  {
    return false;
  }

  IWDG->KR = IWDG_KEY_RELOAD;
  return true;
}

void engine_watchdog_stm32_refresh(void)
{
  IWDG->KR = IWDG_KEY_RELOAD;
}
