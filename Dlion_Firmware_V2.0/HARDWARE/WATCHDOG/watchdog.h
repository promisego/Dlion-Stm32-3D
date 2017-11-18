#ifndef WATCHDOG_H
#define WATCHDOG_H
#include "sys.h"


/*******************************************************本程序开源供3D打印学习使用************************************************
																												Dlion-3D打印主板
																												3D二进制创客---------技术论坛:www.3dbinmaker.com
																												文件说明：看门狗  版本：V1.0
																												Copyright(C)深圳洛众科技有限公司
																												All rights reserved
***********************************************************************************************************************************/



//#ifdef USE_WATCHDOG
  // intialise watch dog with a 1 sec interrupt time
void watchdog_init(void);
  // pad the dog/reset watchdog. MUST be called at least every second after the first watchdog_init or avr will go into emergency procedures..
void watchdog_reset(void);
//#else
  //If we do not have a watchdog, then we can have empty functions which are optimized away.
 // FORCE_INLINE void watchdog_init() {};
 // FORCE_INLINE void watchdog_reset() {};
//#endif

#endif
