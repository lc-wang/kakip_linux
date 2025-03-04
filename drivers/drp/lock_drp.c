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
#include <stddef.h>
#include "lock_drp.h"

int lock_drp(unsigned long long *addr, unsigned int num)
{
	int result = 0;
	unsigned long long old_value;

	/* Check Arguments. */
	if ((NULL == addr) || (((unsigned long long)addr & 0x7uLL) != 0) || (num >= 64))
	{
		result = -2;
	}
	else
	{
		/* set the specified bit. */
		old_value = __sync_fetch_and_or(addr, (1uLL << num));
		/* Check previous value. */
		if ((old_value & (1uLL << num)) != 0)
		{
		  result = -1;
		}
	}

	return result;
}

int unlock_drp(unsigned long long *addr, unsigned int num)
{
	int result = 0;

	/* Check Arguments. */
	if ((NULL == addr) || (((unsigned long long)addr & 0x7uLL) != 0) || (num >= 64))
	{
		result = -2;
	}
	else
	{
		/* Clear the specified bit. */
		(void)__sync_fetch_and_and(addr, ~(1uLL << num));
	}
	return result;
}
