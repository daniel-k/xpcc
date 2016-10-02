#ifndef _RTL8710_H_
#define _RTL8710_H_

#include <stdint.h>

#include "rtl8710_sys.h"
#include "rtl8710_int.h"
#include "rtl8710_peri_on.h"
#include "rtl8710_timer.h"
#include "rtl8710_gpio.h"
#include "rtl8710_log_uart.h"
#include "rtl8710_spi.h"

// this could be everything, who knows what realtek has built there
#define __CM3_REV 0x200

#define __MPU_PRESENT 0
#define __Vendor_SysTickConfig 0

// pessimistic guess!
#define __NVIC_PRIO_BITS 2

/* Cortex-M4 processor and core peripherals */
#include "core_cm3.h"

#endif

