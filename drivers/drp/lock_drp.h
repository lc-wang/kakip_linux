/*
 * Driver for the Renesas RZ/V2H DRP-AI unit
 *
 * Copyright (C) 2023 Renesas Electronics Corporation
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef R_DRP_LOCK_H
#define R_DRP_LOCK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

 int lock_drp(unsigned long long *addr, unsigned int num);
 int unlock_drp(unsigned long long * addr, unsigned int num);
 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* R_DRP_LOCK_H */
