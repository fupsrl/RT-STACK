#ifndef ACTUATOR_TEST_HAL_H
#define ACTUATOR_TEST_HAL_H

#include <stdint.h>

typedef struct { uint32_t BSRR, BRR; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
typedef struct { uint32_t CSR; } COMP_TypeDef;
typedef struct { uint32_t unused; } DAC_TypeDef;
typedef enum { HAL_COMP_STATE_RESET = 0, HAL_COMP_STATE_READY, HAL_COMP_STATE_BUSY }
  HAL_COMP_StateTypeDef;
typedef struct { COMP_TypeDef *Instance; HAL_COMP_StateTypeDef State; }
  COMP_HandleTypeDef;
typedef struct
{
  DAC_TypeDef *Instance;
  uint32_t value[2];
  uint8_t started[2];
} DAC_HandleTypeDef;
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;

#define COMP_CSR_EN       1UL
#define DAC_CHANNEL_1     0U
#define DAC_CHANNEL_2     1U
#define DAC_ALIGN_12B_R   0U
#define __HAL_COMP_DISABLE(h) ((h)->Instance->CSR &= ~COMP_CSR_EN)

extern uint32_t actuator_test_primask;
static inline uint32_t __get_PRIMASK(void) { return actuator_test_primask; }
static inline void __disable_irq(void) { actuator_test_primask = 1U; }
static inline void __enable_irq(void) { actuator_test_primask = 0U; }
#define __DMB() ((void)0)

HAL_StatusTypeDef HAL_COMP_Start(COMP_HandleTypeDef *handle);
HAL_StatusTypeDef HAL_COMP_Stop(COMP_HandleTypeDef *handle);
HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef *handle, uint32_t channel,
                                   uint32_t alignment, uint32_t value);
HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef *handle, uint32_t channel);
void actuator_test_gpio_write_hook(GPIO_TypeDef *port, uint16_t pin,
                                   _Bool high);

extern GPIO_TypeDef actuator_test_gpio[30];

#define IGN1_GPIO_Port   (&actuator_test_gpio[0])
#define IGN2_GPIO_Port   (&actuator_test_gpio[1])
#define IGN3_GPIO_Port   (&actuator_test_gpio[2])
#define IGN4_GPIO_Port   (&actuator_test_gpio[3])
#define IGN5_GPIO_Port   (&actuator_test_gpio[4])
#define IGN6_GPIO_Port   (&actuator_test_gpio[5])
#define IGN7_GPIO_Port   (&actuator_test_gpio[6])
#define IGN8_GPIO_Port   (&actuator_test_gpio[7])
#define IGN9_GPIO_Port   (&actuator_test_gpio[8])
#define IGN10_GPIO_Port  (&actuator_test_gpio[9])
#define IGN11_GPIO_Port  (&actuator_test_gpio[10])
#define IGN12_GPIO_Port  (&actuator_test_gpio[11])
#define IGN1_Pin 1U
#define IGN2_Pin 2U
#define IGN3_Pin 4U
#define IGN4_Pin 8U
#define IGN5_Pin 16U
#define IGN6_Pin 32U
#define IGN7_Pin 64U
#define IGN8_Pin 128U
#define IGN9_Pin 256U
#define IGN10_Pin 512U
#define IGN11_Pin 1024U
#define IGN12_Pin 2048U

#define SEL12_GPIO_Port    (&actuator_test_gpio[12])
#define SEL34_GPIO_Port    (&actuator_test_gpio[13])
#define SEL56_GPIO_Port    (&actuator_test_gpio[14])
#define SEL78_GPIO_Port    (&actuator_test_gpio[15])
#define SEL910_GPIO_Port   (&actuator_test_gpio[16])
#define SEL1112_GPIO_Port  (&actuator_test_gpio[17])
#define BOOST12_GPIO_Port   (&actuator_test_gpio[18])
#define BOOST34_GPIO_Port   (&actuator_test_gpio[19])
#define BOOST56_GPIO_Port   (&actuator_test_gpio[20])
#define BOOST78_GPIO_Port   (&actuator_test_gpio[21])
#define BOOST910_GPIO_Port  (&actuator_test_gpio[22])
#define BOOST1112_GPIO_Port (&actuator_test_gpio[23])
#define SEL12_Pin 1U
#define SEL34_Pin 2U
#define SEL56_Pin 4U
#define SEL78_Pin 8U
#define SEL910_Pin 16U
#define SEL1112_Pin 32U
#define BOOST12_Pin 1U
#define BOOST34_Pin 2U
#define BOOST56_Pin 4U
#define BOOST78_Pin 8U
#define BOOST910_Pin 16U
#define BOOST1112_Pin 32U

#endif
