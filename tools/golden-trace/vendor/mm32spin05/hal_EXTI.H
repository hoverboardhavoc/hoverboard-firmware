/* SPDX-License-Identifier: GPL-3.0-or-later
 * LibSamples V1.13's hal_syscfg.h includes "hal_EXTI.H"; the file it ships is hal_exti.h. A
 * case-insensitive file system finds it; a case-sensitive one (the CI runner) does not. This shim,
 * on the harness's include path ahead of the library's, continues the search for the real header.
 * include_next, so on a case-insensitive file system, where a lookup of "hal_exti.h" can land
 * here, it does not include itself. */
#include_next "hal_exti.h"
