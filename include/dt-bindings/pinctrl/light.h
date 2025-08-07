// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 BeagleBoard.org - https://beagleboard.org/
 * Copyright (C) 2023 Deepak Khatri <lorforlinux@beagleboard.org>
 */

#ifndef _LIGHT_H
#define _LIGHT_H

#define MUX_MODE0	0x0
#define MUX_MODE1	0x1
#define MUX_MODE2	0x2
#define MUX_MODE3	0x3
#define MUX_MODE4	0x4
#define MUX_MODE5	0x5

#define INPUT_ENABLE_SHIFT          (9)
#define SLEW_RATE_SHIFT             (8)
#define SCHMITT_TRIGGER_SHIFT       (7)
#define STRONG_PULLUP_SHIFT         (6)
#define PULL_SELECT_SHIFT           (5)
#define PULL_ENABLE_SHIFT           (4)
#define DRIVE_STRENGTH_BIT3_SHIFT   (3)
#define DRIVE_STRENGTH_BIT2_SHIFT   (2)
#define DRIVE_STRENGTH_BIT1_SHIFT   (1)
#define DRIVE_STRENGTH_BIT0_SHIFT   (0)

#define PULL_ENA		(1 << PULL_ENABLE_SHIFT)
#define PULL_UP			(1 << PULL_SELECT_SHIFT)
#define INPUT_EN		(1 << INPUT_ENABLE_SHIFT)
#define SLEWCONTROL		(1 << SLEW_RATE_SHIFT)
#define STRONG_PULLUP   (1 << STRONG_PULLUP_SHIFT)

#define DS3 (1 << DRIVE_STRENGTH_BIT3_SHIFT)
#define DS2 (1 << DRIVE_STRENGTH_BIT2_SHIFT)
#define DS1 (1 << DRIVE_STRENGTH_BIT1_SHIFT)
#define DS0 (1 << DRIVE_STRENGTH_BIT0_SHIFT)

#define STRENGTH_LOW    (DS0 | DS2)
#define STRENGTH_MID    (DS1 | DS3)
#define STRENGTH_HIGH   (DS0 | DS1 | DS2 | DS3)

#define PIN_OUTPUT                  0
#define PIN_OUTPUT_PULLDOWN         (PIN_OUTPUT | PULL_ENA)
#define PIN_OUTPUT_PULLUP           (PIN_OUTPUT | PULL_ENA | PULL_UP)
#define PIN_OUTPUT_STRONG_PULLUP    (PIN_OUTPUT | PULL_ENA | PULL_UP | STRONG_PULLUP)

#define PIN_INPUT                   INPUT_EN
#define PIN_INPUT_PULLDOWN          (PULL_ENA | INPUT_EN)
#define PIN_INPUT_PULLUP            (PULL_ENA | INPUT_EN | PULL_UP)
#define PIN_INPUT_STRONG_PULLUP     (PULL_ENA | INPUT_EN | PULL_UP | STRONG_PULLUP)

#endif