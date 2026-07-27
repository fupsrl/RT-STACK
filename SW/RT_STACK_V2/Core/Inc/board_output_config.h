/**
 ******************************************************************************
 * @file    board_output_config.h
 * @brief   STM32 pin/peripheral mapping for ignition and injector outputs.
 *
 * Engine calibration belongs in engine_config.h.  Change this file only when
 * the PCB routing changes.  All public channel numbers are one based.
 ******************************************************************************
 */
#ifndef BOARD_OUTPUT_CONFIG_H
#define BOARD_OUTPUT_CONFIG_H

#ifdef ACTUATOR_HOST_TEST
#include "actuator_test_hal.h"
#else
#include "main.h"
#endif
#include <stdbool.h>
#include <stdint.h>

#define BOARD_IGNITION_OUTPUT_COUNT   12U
#define BOARD_INJECTOR_OUTPUT_COUNT   12U
#define BOARD_INJECTOR_PAIR_COUNT      6U

extern COMP_HandleTypeDef hcomp1, hcomp2, hcomp3, hcomp4, hcomp5, hcomp6;
extern DAC_HandleTypeDef  hdac1, hdac3, hdac4;

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
} board_ignition_output_t;

typedef struct
{
  COMP_HandleTypeDef *comparator;
  DAC_HandleTypeDef  *threshold_dac;
  uint32_t dac_channel;
  GPIO_TypeDef *select_port;
  uint16_t select_pin;
  GPIO_TypeDef *boost_port;
  uint16_t boost_pin;
} board_injector_pair_t;

typedef struct
{
  uint8_t pair_index;          /* index into board_injector_pairs[] */
  GPIO_PinState select_state;  /* routes the pair driver to this output */
} board_injector_output_t;

static const board_ignition_output_t
board_ignition_outputs[BOARD_IGNITION_OUTPUT_COUNT] =
{
  { IGN1_GPIO_Port,  IGN1_Pin  }, { IGN2_GPIO_Port,  IGN2_Pin  },
  { IGN3_GPIO_Port,  IGN3_Pin  }, { IGN4_GPIO_Port,  IGN4_Pin  },
  { IGN5_GPIO_Port,  IGN5_Pin  }, { IGN6_GPIO_Port,  IGN6_Pin  },
  { IGN7_GPIO_Port,  IGN7_Pin  }, { IGN8_GPIO_Port,  IGN8_Pin  },
  { IGN9_GPIO_Port,  IGN9_Pin  }, { IGN10_GPIO_Port, IGN10_Pin },
  { IGN11_GPIO_Port, IGN11_Pin }, { IGN12_GPIO_Port, IGN12_Pin },
};

static const board_injector_pair_t
board_injector_pairs[BOARD_INJECTOR_PAIR_COUNT] =
{
  { &hcomp1, &hdac1, DAC_CHANNEL_1, SEL12_GPIO_Port,   SEL12_Pin,   BOOST12_GPIO_Port,   BOOST12_Pin   },
  { &hcomp2, &hdac1, DAC_CHANNEL_2, SEL34_GPIO_Port,   SEL34_Pin,   BOOST34_GPIO_Port,   BOOST34_Pin   },
  { &hcomp3, &hdac3, DAC_CHANNEL_1, SEL56_GPIO_Port,   SEL56_Pin,   BOOST56_GPIO_Port,   BOOST56_Pin   },
  { &hcomp4, &hdac3, DAC_CHANNEL_2, SEL78_GPIO_Port,   SEL78_Pin,   BOOST78_GPIO_Port,   BOOST78_Pin   },
  { &hcomp5, &hdac4, DAC_CHANNEL_1, SEL910_GPIO_Port,  SEL910_Pin,  BOOST910_GPIO_Port,  BOOST910_Pin  },
  { &hcomp6, &hdac4, DAC_CHANNEL_2, SEL1112_GPIO_Port, SEL1112_Pin, BOOST1112_GPIO_Port, BOOST1112_Pin },
};

static const board_injector_output_t
board_injector_outputs[BOARD_INJECTOR_OUTPUT_COUNT] =
{
  { 0U, GPIO_PIN_SET   }, { 0U, GPIO_PIN_RESET },
  { 1U, GPIO_PIN_SET   }, { 1U, GPIO_PIN_RESET },
  { 2U, GPIO_PIN_SET   }, { 2U, GPIO_PIN_RESET },
  { 3U, GPIO_PIN_SET   }, { 3U, GPIO_PIN_RESET },
  { 4U, GPIO_PIN_SET   }, { 4U, GPIO_PIN_RESET },
  { 5U, GPIO_PIN_SET   }, { 5U, GPIO_PIN_RESET },
};

/* Direct GPIO writes keep the emergency path independent of HAL state. */
static inline void board_gpio_write(GPIO_TypeDef *port, uint16_t pin, bool high)
{
  if (high)
  {
    port->BSRR = (uint32_t)pin;
  }
  else
  {
    port->BRR = (uint32_t)pin;
  }
#ifdef ACTUATOR_HOST_TEST
  actuator_test_gpio_write_hook(port, pin, high);
#endif
}

static inline void board_ignition_write(uint8_t output_index, bool charge)
{
  const board_ignition_output_t *output = &board_ignition_outputs[output_index];
  board_gpio_write(output->port, output->pin, charge);
}

#endif /* BOARD_OUTPUT_CONFIG_H */
