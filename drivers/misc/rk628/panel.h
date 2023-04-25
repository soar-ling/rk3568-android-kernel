// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Rockchip Electronics Co. Ltd.
 *
 * Author: Guochun Huang <hero.huang@rock-chips.com>
 */
#ifndef _PANEL_H
#define _PANEL_H

#include "rk628.h"

int panel_info_get(struct rk628 *rk628, struct device_node *np);
void panel_prepare(struct rk628 *rk628);
void panel_enable(struct rk628 *rk628);
void panel_unprepare(struct rk628 *rk628);
void panel_disable(struct rk628 *rk628);
#endif

