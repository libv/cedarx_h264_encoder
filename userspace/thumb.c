/*
 * Copyright (c) 2020 Luc Verhaegen <libv@skynet.be>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 */
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>
#include <fcntl.h>

#include "cedar_ioctl.h"

static int cedar_fd = -1;

int main(int argc, char *argv[])
{
	cedar_fd = open(CEDAR_DEVICE_PATH, O_RDWR);
	if (cedar_fd == -1) {
		fprintf(stderr, "Error: %s():open(%s): %s\n",
			__func__, CEDAR_DEVICE_PATH, strerror(errno));
		return errno;
	}

	return 0;
}
