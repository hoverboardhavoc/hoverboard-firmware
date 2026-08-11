/* SPL peripheral-include selector for the stress firmware, GD32F1x0 (BOARD_OFFROAD).
 *
 * `gd32f1x0.h` includes `gd32f1x0_libopt.h` when `USE_STDPERIPH_DRIVER` is set; the GD32 SPL ships this
 * file in its per-project Template/, which this self-contained tree does not vendor. It lists exactly the
 * SPL peripheral headers this firmware uses (RCU clock tree, GPIO + per-pin AF mux, USART, FMC wait
 * states). Keep it in the firmware dir (on the `-I.` path) so the build stays hermetic to GD_SPL for the
 * SPL sources only. The F10x sibling is gd32f10x_libopt.h; the two are selected by the family header the
 * variant includes, not by the build.
 */
#ifndef GD32F1X0_LIBOPT_H
#define GD32F1X0_LIBOPT_H

#include "gd32f1x0_rcu.h"
#include "gd32f1x0_gpio.h"
#include "gd32f1x0_usart.h"
#include "gd32f1x0_fmc.h"
#include "gd32f1x0_misc.h" /* CMSIS system_gd32f1x0.c's SystemInit references nvic_vector_table_set */

#endif /* GD32F1X0_LIBOPT_H */
