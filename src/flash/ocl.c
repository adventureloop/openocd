/***************************************************************************
 *   Copyright (C) 2007 by Pavel Chromy                                    *
 *   chromy@asix.cz                                                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "ocl.h"
#include "flash.h"
#include "embeddedice.h"


struct ocl_priv
{
	struct arm_jtag *jtag_info;
	unsigned int buflen;
	unsigned int bufalign;
};

static int ocl_erase_check(struct flash_bank *bank)
{
	return ERROR_OK;
}

static int ocl_protect_check(struct flash_bank *bank)
{
	return ERROR_OK;
}

/* flash_bank ocl 0 0 0 0 <target#> */
FLASH_BANK_COMMAND_HANDLER(ocl_flash_bank_command)
{
	int retval;
	struct arm *armv4_5;
	struct arm7_9_common *arm7_9;
	struct ocl_priv *ocl;

	if (argc < 6)
	{
		LOG_WARNING("incomplete flash_bank ocl configuration");
		return ERROR_FLASH_BANK_INVALID;
	}

	if ((retval = arm7_9_get_arch_pointers(bank->target, &armv4_5, &arm7_9)) != ERROR_OK)
		return retval;

	ocl = bank->driver_priv = malloc(sizeof(struct ocl_priv));
	ocl->jtag_info = &arm7_9->jtag_info;
	ocl->buflen = 0;
	ocl->bufalign = 1;

	return ERROR_OK;
}

static int ocl_erase(struct flash_bank *bank, int first, int last)
{
	struct ocl_priv *ocl = bank->driver_priv;
	int retval;
	uint32_t dcc_buffer[3];

	/* check preconditions */
	if (bank->num_sectors == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (bank->target->state != TARGET_RUNNING)
	{
		LOG_ERROR("target has to be running to communicate with the loader");
		return ERROR_TARGET_NOT_RUNNING;
	}

	if ((first == 0) && (last == bank->num_sectors - 1))
	{
		dcc_buffer[0] = OCL_ERASE_ALL;
		if ((retval = embeddedice_send(ocl->jtag_info, dcc_buffer, 1) != ERROR_OK))
			return retval;
	}
	else
	{
		dcc_buffer[0] = OCL_ERASE_BLOCK;
		dcc_buffer[1] = first;
		dcc_buffer[2] = last;
		if ((retval = embeddedice_send(ocl->jtag_info, dcc_buffer, 3) != ERROR_OK))
			return retval;
	}

	/* wait for response, fixed timeout of 1 s */
	if ((retval = embeddedice_handshake(ocl->jtag_info, EICE_COMM_CTRL_WBIT, 1000) != ERROR_OK))
	{
		if (retval == ERROR_TARGET_TIMEOUT)
			LOG_ERROR("loader not responding");
		return retval;
	}

	/* receive response */
	if ((retval = embeddedice_receive(ocl->jtag_info, dcc_buffer + 1, 1) != ERROR_OK))
		return retval;

	if (dcc_buffer[1] != OCL_CMD_DONE)
	{
		if (dcc_buffer[0] == OCL_ERASE_ALL)
			LOG_ERROR("loader response to OCL_ERASE_ALL 0x%08" PRIx32 "", dcc_buffer[1]);
		else
			LOG_ERROR("loader response to OCL_ERASE_BLOCK 0x%08" PRIx32 "", dcc_buffer[1]);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static int ocl_protect(struct flash_bank *bank, int set, int first, int last)
{
	return ERROR_OK;
}

static int ocl_write(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct ocl_priv *ocl = bank->driver_priv;
	int retval;
	uint32_t *dcc_buffer;
	uint32_t *dcc_bufptr;
	int byteofs;
	int runlen;
	uint32_t chksum;

	int i;

	/* check preconditions */
	if (ocl->buflen == 0 || ocl->bufalign == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (bank->target->state != TARGET_RUNNING)
	{
		LOG_ERROR("target has to be running to communicate with the loader");
		return ERROR_TARGET_NOT_RUNNING;
	}

	/* allocate buffer for max. ocl buffer + overhead */
	dcc_buffer = malloc(sizeof(uint32_t)*(ocl->buflen/4 + 3));

	while (count)
	{
		if (count + (offset % ocl->bufalign) > ocl->buflen)
			runlen = ocl->buflen - (offset % ocl->bufalign);
		else
			runlen = count;

		dcc_buffer[0] = OCL_FLASH_BLOCK | runlen;
		dcc_buffer[1] = offset;
		dcc_bufptr = &dcc_buffer[2];

		*dcc_bufptr = 0xffffffff;
		byteofs = (offset % ocl->bufalign) % 4;
		chksum = OCL_CHKS_INIT;

		/* copy data to DCC buffer in proper byte order and properly aligned */
		for (i = 0; i < runlen; i++)
		{
			switch (byteofs++)
			{
				case 0:
					*dcc_bufptr &= *(buffer++) | 0xffffff00;
					break;
				case 1:
					*dcc_bufptr &= ((*(buffer++)) << 8) | 0xffff00ff;
					break;
				case 2:
					*dcc_bufptr &= ((*(buffer++)) << 16) | 0xff00ffff;
					break;
				case 3:
					*dcc_bufptr &= ((*(buffer++)) << 24) | 0x00ffffff;
					chksum ^= *(dcc_bufptr++);
					*dcc_bufptr = 0xffffffff;
					byteofs = 0;
					break;
			}
		}

		/* add the remaining word to checksum */
		if (byteofs)
			chksum ^= *(dcc_bufptr++);

		*(dcc_bufptr++) = chksum;

		/* send the data */
		if ((retval = embeddedice_send(ocl->jtag_info, dcc_buffer, dcc_bufptr-dcc_buffer)) != ERROR_OK)
		{
			free(dcc_buffer);
		  return retval;
		}

		/* wait for response, fixed timeout of 1 s */
		if ((retval = embeddedice_handshake(ocl->jtag_info, EICE_COMM_CTRL_WBIT, 1000) != ERROR_OK))
		{
			if (retval == ERROR_TARGET_TIMEOUT)
				LOG_ERROR("loader not responding");
			free(dcc_buffer);
			return retval;
		}

		/* receive response */
		if ((retval = embeddedice_receive(ocl->jtag_info, dcc_buffer, 1) != ERROR_OK))
		{
			free(dcc_buffer);
			return retval;
		}

		if (dcc_buffer[0] != OCL_CMD_DONE)
		{
			LOG_ERROR("loader response to OCL_FLASH_BLOCK 0x%08" PRIx32 "", dcc_buffer[0]);
			free(dcc_buffer);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		count -= runlen;
		offset += runlen;
	}

	free(dcc_buffer);
	return ERROR_OK;
}

static int ocl_probe(struct flash_bank *bank)
{
	struct ocl_priv *ocl = bank->driver_priv;
	int retval;
	uint32_t dcc_buffer[1];
	int sectsize;
	int i;

	/* purge pending data in DCC */
	embeddedice_receive(ocl->jtag_info, dcc_buffer, 1);

	dcc_buffer[0] = OCL_PROBE;
	if ((retval = embeddedice_send(ocl->jtag_info, dcc_buffer, 1) != ERROR_OK))
		return retval;

	/* wait for response, fixed timeout of 1 s */
	if ((retval = embeddedice_handshake(ocl->jtag_info, EICE_COMM_CTRL_WBIT, 1000) != ERROR_OK))
	{
		if (retval == ERROR_TARGET_TIMEOUT)
			LOG_ERROR("loader not responding");
		return retval;
	}

	/* receive response */
	if ((retval = embeddedice_receive(ocl->jtag_info, dcc_buffer, 1) != ERROR_OK))
		return retval;

	if (dcc_buffer[0] != OCL_CMD_DONE)
	{
		LOG_ERROR("loader response to OCL_PROBE 0x%08" PRIx32 "", dcc_buffer[0]);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* receive and fill in parameters, detection of loader is important, receive it one by one */
	if ((retval = embeddedice_handshake(ocl->jtag_info, EICE_COMM_CTRL_WBIT, 0) != ERROR_OK)
		|| (retval = embeddedice_receive(ocl->jtag_info, dcc_buffer, 1) != ERROR_OK))
		return retval;
	bank->base = dcc_buffer[0];

	if ((retval = embeddedice_handshake(ocl->jtag_info, EICE_COMM_CTRL_WBIT, 0) != ERROR_OK)
		|| (retval = embeddedice_receive(ocl->jtag_info, dcc_buffer, 1) != ERROR_OK))
		return retval;
	bank->size = dcc_buffer[0];

	if ((retval = embeddedice_handshake(ocl->jtag_info, EICE_COMM_CTRL_WBIT, 0) != ERROR_OK)
		|| (retval = embeddedice_receive(ocl->jtag_info, dcc_buffer, 1) != ERROR_OK))
		return retval;
	bank->num_sectors = dcc_buffer[0];

	if ((retval = embeddedice_handshake(ocl->jtag_info, EICE_COMM_CTRL_WBIT, 0) != ERROR_OK)
		|| (retval = embeddedice_receive(ocl->jtag_info, dcc_buffer, 1) != ERROR_OK))
		return retval;
	ocl->buflen = dcc_buffer[0] & 0xffff;
	ocl->bufalign = dcc_buffer[0] >> 16;

	bank->sectors = realloc(bank->sectors, sizeof(struct flash_sector)*bank->num_sectors);
	if (bank->num_sectors == 0)
	{
		LOG_ERROR("number of sectors shall be non zero value");
		return ERROR_FLASH_BANK_INVALID;
	}
	if (bank->size % bank->num_sectors) {
		LOG_ERROR("bank size not divisible by number of sectors");
		return ERROR_FLASH_BANK_INVALID;
	}
	sectsize = bank->size / bank->num_sectors;
	for (i = 0; i < bank->num_sectors; i++)
	{
		bank->sectors[i].offset = i * sectsize;
		bank->sectors[i].size = sectsize;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
	}

	if (ocl->bufalign == 0)
		ocl->bufalign = 1;

	if (ocl->buflen == 0)
	{
		LOG_ERROR("buflen shall be non zero value");
		return ERROR_FLASH_BANK_INVALID;
	}

	if ((ocl->bufalign > ocl->buflen) || (ocl->buflen % ocl->bufalign))
	{
		LOG_ERROR("buflen is not multiple of bufalign");
		return ERROR_FLASH_BANK_INVALID;
	}

	if (ocl->buflen % 4)
	{
		LOG_ERROR("buflen shall be divisible by 4");
		return ERROR_FLASH_BANK_INVALID;
	}

	return ERROR_OK;
}

static int ocl_info(struct flash_bank *bank, char *buf, int buf_size)
{
	return ERROR_OK;
}

static int ocl_auto_probe(struct flash_bank *bank)
{
	struct ocl_priv *ocl = bank->driver_priv;

	if (ocl->buflen == 0 || ocl->bufalign == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	return ERROR_OK;
}

struct flash_driver ocl_flash = {
		.name = "ocl",
		.flash_bank_command = &ocl_flash_bank_command,
		.erase = &ocl_erase,
		.protect = &ocl_protect,
		.write = &ocl_write,
		.probe = &ocl_probe,
		.erase_check = &ocl_erase_check,
		.protect_check = &ocl_protect_check,
		.info = &ocl_info,
		.auto_probe = &ocl_auto_probe,
	};
