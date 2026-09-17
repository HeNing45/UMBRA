/*
 * SPDX-FileCopyrightText: Copyright 2026 He Ning
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/* UMBRA Embench-IoT port configuration.
 *
 * Included by upstream support code via -DHAVE_CONFIG_H (support.h) and
 * unconditionally by support/chip.c. This port supplies a board layer only;
 * no chip layer is needed, so HAVE_CHIPSUPPORT_H stays undefined and chip.c
 * compiles to its empty translation unit.
 */
#ifndef UMBRA_EMBENCH_CONFIG_H
#define UMBRA_EMBENCH_CONFIG_H

#define HAVE_BOARDSUPPORT_H 1

#endif /* UMBRA_EMBENCH_CONFIG_H */
