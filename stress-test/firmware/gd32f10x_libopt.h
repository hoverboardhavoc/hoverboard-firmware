/* SPL peripheral-include selector for the stress firmware.
 *
 * `gd32f10x.h` includes `gd32f10x_libopt.h` when `USE_STDPERIPH_DRIVER` is set; the GD32 SPL ships this
 * file in its per-project Template/, which this self-contained tree does not vendor. It lists exactly the
 * SPL peripheral headers this firmware uses (RCU clock tree, GPIO AF, USART, FMC wait states). Keep it in
 * the firmware dir (on the `-I.` path) so the build stays hermetic to GD_SPL for the SPL sources only.
 */
#ifndef GD32F10X_LIBOPT_H
#define GD32F10X_LIBOPT_H

#include "gd32f10x_rcu.h"
#include "gd32f10x_gpio.h"
#include "gd32f10x_usart.h"
#include "gd32f10x_fmc.h"
#include "gd32f10x_misc.h" /* CMSIS system_gd32f10x.c's SystemInit references nvic_vector_table_set */

#endif /* GD32F10X_LIBOPT_H */
