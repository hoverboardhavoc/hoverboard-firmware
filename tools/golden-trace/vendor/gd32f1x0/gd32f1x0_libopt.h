/* The SPL's peripheral-header selection, which the vendor library expects the application to
 * supply (gd32f1x0.h includes it under USE_STDPERIPH_DRIVER). The harness includes every peripheral
 * header of the family, so a vector body calls any driver without listing includes. */

#ifndef GD32F1X0_LIBOPT_H
#define GD32F1X0_LIBOPT_H

#include "gd32f1x0_adc.h"
#include "gd32f1x0_can.h"
#include "gd32f1x0_cec.h"
#include "gd32f1x0_cmp.h"
#include "gd32f1x0_crc.h"
#include "gd32f1x0_dac.h"
#include "gd32f1x0_dbg.h"
#include "gd32f1x0_dma.h"
#include "gd32f1x0_exti.h"
#include "gd32f1x0_fmc.h"
#include "gd32f1x0_fwdgt.h"
#include "gd32f1x0_gpio.h"
#include "gd32f1x0_i2c.h"
#include "gd32f1x0_ivref.h"
#include "gd32f1x0_misc.h"
#include "gd32f1x0_opa.h"
#include "gd32f1x0_pmu.h"
#include "gd32f1x0_rcu.h"
#include "gd32f1x0_rtc.h"
#include "gd32f1x0_slcd.h"
#include "gd32f1x0_spi.h"
#include "gd32f1x0_syscfg.h"
#include "gd32f1x0_timer.h"
#include "gd32f1x0_tsi.h"
#include "gd32f1x0_usart.h"
#include "gd32f1x0_wwdgt.h"

#endif /* GD32F1X0_LIBOPT_H */
