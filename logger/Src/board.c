#include "board.h"
#include "stm32f1xx_hal.h"
#include "xprintf.h"
#include "stdio.h"
#include "string.h"
#include <stdbool.h>
#include <stdint.h>

void showBootInfo(void)
{
  //mprintf("\r\n\nBooting project: %s\n", PROJECT_NAME);
  //mprintf("SVNRev: " REVISION_STR "\r\n");
  //mprintf("SVNRev: " "\r\n");
}

uint64_t getUptimePrecise(void)
{
  return (uint64_t)HAL_GetTick() * 1000 * 1000;
}

uint64_t getUptime(void)
{
  return (uint64_t)getUptimePrecise();
}

void boardRestart(void)
{
  NVIC_SystemReset();
}

volatile uint32_t systick_delay;
void delay_ms(uint32_t dval)
{
  systick_delay = dval;
  while (systick_delay > 0) {__ASM("wfi");};
}



void delay_us(int cnt)
{
  volatile uint32_t cnt_local = cnt * 5;
  while (cnt_local--);
}