/* The SPL's peripheral-header selection, which the vendor library expects the application to
 * supply (gd32f10x.h includes it under USE_STDPERIPH_DRIVER). The harness includes every peripheral
 * header of the family, so a vector body calls any driver without listing includes. */

#ifndef GD32F10X_LIBOPT_H
#define GD32F10X_LIBOPT_H

#include "gd32f10x_adc.h"
#include "gd32f10x_bkp.h"
#include "gd32f10x_can.h"
#include "gd32f10x_crc.h"
#include "gd32f10x_dac.h"
#include "gd32f10x_dbg.h"
#include "gd32f10x_dma.h"
#include "gd32f10x_enet.h"
#include "gd32f10x_exmc.h"
#include "gd32f10x_exti.h"
#include "gd32f10x_fmc.h"
#include "gd32f10x_fwdgt.h"
#include "gd32f10x_gpio.h"
#include "gd32f10x_i2c.h"
#include "gd32f10x_misc.h"
#include "gd32f10x_pmu.h"
#include "gd32f10x_rcu.h"
#include "gd32f10x_rtc.h"
#include "gd32f10x_sdio.h"
#include "gd32f10x_spi.h"
#include "gd32f10x_timer.h"
#include "gd32f10x_usart.h"
#include "gd32f10x_wwdgt.h"

#endif /* GD32F10X_LIBOPT_H */
