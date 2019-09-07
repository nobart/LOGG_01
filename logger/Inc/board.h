#ifndef __board_H
#define __board_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "xprintf.h"

#define UNUSED_FUNCTION(x) ((void)(x))

#if !defined(FALSE)
#define FALSE 0 /**< define FALSE=0 */
#endif

#if !defined(TRUE)
#define TRUE (1) /**< define TRUE=1 */
#endif

#if !defined(NULL)
#define NULL (void *)0  /**< define NULL */
#endif

#define ARRAY_SIZE(arg_name) (sizeof(arg_name)/sizeof(arg_name[0]))
#define massert(arg_cond) if(!(arg_cond)) {mprintfVCP("assert: %s:%d %s\n", __FILE__, __LINE__, __FUNCTION__); volatile int stop = 1; while(stop);}


#define IS_AF(c)                      ((c >= 'A') && (c <= 'F'))
#define IS_af(c)                      ((c >= 'a') && (c <= 'f'))
#define IS_09(c)                      ((c >= '0') && (c <= '9'))
#define ISVALIDHEX(c)                  IS_AF(c) || IS_af(c) || IS_09(c)
#define ISVALIDDEC(c)                  IS_09(c)
#define CONVERTDEC(c)                  (c - '0')

#define CONVERTHEX_alpha(c)          (IS_AF(c) ? (c - 'A'+10) : (c - 'a'+10))
#define CONVERTHEX(c)               (IS_09(c) ? (c - '0') : CONVERTHEX_alpha(c))

void boardRestart(void);
void screenConfig(void);

uint64_t getUptimePrecise(void);
uint64_t getUptime(void);
void showBootInfo(void);

extern volatile uint32_t systick_delay;
void delay_ms(uint32_t dval);

#define US2NS(arg_time) ((uint64_t)(((uint64_t)(arg_time))*1000ULL))
#define MS2NS(arg_time) ((uint64_t)(US2NS(arg_time)*1000ULL))
#define  S2NS(arg_time) ((uint64_t)(MS2NS(arg_time)*1000ULL))
#define  M2NS(arg_time) ((uint64_t)(S2NS (arg_time)*60ULL))
#define  H2NS(arg_time) ((uint64_t)(M2NS (arg_time)*60ULL))
#define  D2NS(arg_time) ((uint64_t)(H2NS (arg_time)*24ULL))

#define NS2US(arg_time) ((uint64_t)((uint64_t)((arg_time))/1000ULL))
#define NS2MS(arg_time) ((uint64_t)(NS2US(arg_time)/1000ULL))
#define  NS2S(arg_time) ((uint64_t)(NS2MS(arg_time)/1000ULL))
#define  NS2M(arg_time) ((uint64_t)( NS2S(arg_time)/60ULL))
#define  NS2H(arg_time) ((uint64_t)( NS2M(arg_time)/60ULL))
#define  NS2D(arg_time) ((uint64_t)( NS2H(arg_time)/24ULL))

#define INIT_STATUS_NC          0
#define INIT_STATUS_OK          HAL_OK
#define INIT_STATUS_ERROR       HAL_ERROR


void delay_us(int cnt);


#ifdef __cplusplus
}
#endif
#endif /*__board_H */