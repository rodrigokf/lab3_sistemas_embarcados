#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include <stdint.h>

extern uint32_t SystemCoreClock;

extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);

#endif // __SYSTEM_H__
