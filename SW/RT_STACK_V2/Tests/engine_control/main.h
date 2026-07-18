/** Minimal CMSIS interrupt primitives for the engine-control host tests. */
#ifndef ENGINE_CONTROL_TEST_MAIN_H
#define ENGINE_CONTROL_TEST_MAIN_H

#include <stdint.h>

static inline uint32_t __get_PRIMASK(void)
{
  return 0U;
}

static inline void __disable_irq(void)
{
}

static inline void __enable_irq(void)
{
}

static inline void __DMB(void)
{
}

#endif /* ENGINE_CONTROL_TEST_MAIN_H */
