#ifndef __xprintf_H
#define __xprintf_H
#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------*/
/* Universal string handler for user console interface  (C)ChaN, 2011     */
/*------------------------------------------------------------------------*/

#ifndef _STRFUNC
#define _STRFUNC

#define C_NORMAL  "\x1B[0m"  // normal
#define C_RED     "\x1B[31m" // red
#define C_GREEN   "\x1B[32m" // green
#define C_YELLOW  "\x1B[33m" // yellow
#define C_BLUE    "\x1B[34m" // blue
#define C_MAGENTA "\x1B[35m" // magenta
#define C_CYAN    "\x1B[36m" // cyan
#define C_WHITE   "\x1B[37m" // white

#define _USE_XFUNC_OUT  1 /* 1: Use output functions */
#define _CR_CRLF    1 /* 1: Convert \n ==> \r\n in the output char */

#define _USE_XFUNC_IN 0 /* 1: Use input function */
#define _LINE_ECHO    0 /* 1: Echo back input chars in xgets function */

#define _STATIC_BUFF_SIZE 1024

extern void (*xfunc_out)(unsigned char);
void mprintf(const char* fmt, ... );
void mprintfVCP(const char* fmt, ... );


//#define xdev_out(func) xfunc_out = (void(*)(unsigned char))(func)

void xsprintf (char* buff, const char* fmt, ...);

#define DW_CHAR   sizeof(char)
#define DW_SHORT  sizeof(short)
#define DW_LONG   sizeof(long)

#if _USE_XFUNC_IN
#define xdev_in(func) xfunc_in = (unsigned char(*)(void))(func)
extern unsigned char (*xfunc_in)(void);
int xgets (char* buff, int len);
int xfgets (unsigned char (*func)(void), char* buff, int len);
int xatoi (char** str, long* res);
#endif

#endif

#ifdef __cplusplus
}
#endif
#endif /*__xprintf_H */



