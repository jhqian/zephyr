/*
 * Copyright (c) 2020 Softube AB
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_imx_flexspi
#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)

#include <zephyr.h>
#include <kernel.h>
#include <device.h>
#include <string.h>
#include <drivers/flash.h>
#include <errno.h>
#include <init.h>
#include <soc.h>
#include <devicetree.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(flash_qspi, LOG_LEVEL_DBG);

#include "fsl_flexspi.h"

struct flash_priv {
	struct k_sem write_lock;
	FLEXSPI_Type  *pflash_block_base;
};

/* Pre-defined LUT sequence indexes according to i.MX RT1020 Processor Reference Manual
 * 8.6.3.1 FlexSPI Configuration Block.
 *
 * Note : this must match the FCB block actually programming into the FLASH - see
 * evkmimxrt1020_flexspi_nor_config.c.
 */

#define NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD 0
#define NOR_CMD_LUT_SEQ_IDX_READSTATUS 1
#define NOR_CMD_LUT_SEQ_IDX_WRITEENABLE 2
#define NOR_CMD_LUT_SEQ_IDX_ERASESECTOR 3
#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD 4
#define NOR_CMD_LUT_SEQ_IDX_ERASECHIP 5
#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE 6
#define NOR_CMD_LUT_SEQ_IDX_READ_NORMAL 7
#define NOR_CMD_LUT_SEQ_IDX_READID 8
#define NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG 9
#define NOR_CMD_LUT_SEQ_IDX_ENTERQPI 10
#define NOR_CMD_LUT_SEQ_IDX_EXITQPI 11
#define NOR_CMD_LUT_SEQ_IDX_READSTATUSREG 12
#define NOR_CMD_LUT_SEQ_IDX_READ_FAST 13

#define CUSTOM_LUT_LENGTH 60
#define FLASH_QUAD_ENABLE 0x40
#define FLASH_BUSY_STATUS_POL 1
#define FLASH_BUSY_STATUS_OFFSET 0
#define FLASH_ERROR_STATUS_MASK 0x0e

flexspi_device_config_t deviceconfig = {
	.flexspiRootClk = 133000000,
	.flashSize = DT_REG_SIZE(SOC_NV_FLASH_NODE) / 1024,
	.CSIntervalUnit = kFLEXSPI_CsIntervalUnit1SckCycle,
	.CSInterval = 2,
	.CSHoldTime = 3,
	.CSSetupTime = 3,
	.dataValidTime = 0,
	.columnspace = 0,
	.enableWordAddress = 0,
	.AWRSeqIndex = 0,
	.AWRSeqNumber = 0,
	.ARDSeqIndex = NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD,
	.ARDSeqNumber = 1,
	.AHBWriteWaitUnit = kFLEXSPI_AhbWriteWaitUnit2AhbCycle,
	.AHBWriteWaitInterval = 0,
};

const uint32_t customLUT[CUSTOM_LUT_LENGTH] = {
	/* Normal read mode -SDR */
	[4 * NOR_CMD_LUT_SEQ_IDX_READ_NORMAL] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x03, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
	[4 * NOR_CMD_LUT_SEQ_IDX_READ_NORMAL + 1] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

	/* Fast read mode - SDR */
	[4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x0B, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
	[4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST + 1] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_DUMMY_SDR, kFLEXSPI_1PAD, 0x08, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

	/* Fast read quad mode - SDR */
	[4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xEB, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_4PAD, 0x18),
	[4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD + 1] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_DUMMY_SDR, kFLEXSPI_4PAD, 0x06, kFLEXSPI_Command_READ_SDR, kFLEXSPI_4PAD, 0x04),

	/* Read extend parameters */
	[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x81, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

	/* Write Enable */
	[4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x06, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

	/* Erase Sector  */
	[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xD7, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),

	/* Page Program - single mode */
	[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x02, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
	[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE + 1] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

	/* Page Program - quad mode */
	[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x32, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
	[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD + 1] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_4PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

	/* Read ID */
	[4 * NOR_CMD_LUT_SEQ_IDX_READID] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x9F, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

	/* Enable Quad mode */
	[4 * NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x01, kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0x04),

	/* Enter QPI mode */
	[4 * NOR_CMD_LUT_SEQ_IDX_ENTERQPI] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x35, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

	/* Exit QPI mode */
	[4 * NOR_CMD_LUT_SEQ_IDX_EXITQPI] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_4PAD, 0xF5, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

	/* Read status register */
	[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUSREG] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x05, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

	/* Erase whole chip */
	[4 * NOR_CMD_LUT_SEQ_IDX_ERASECHIP] =
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xC7, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),
};

static status_t flash_mcux_flexspi_qspi_wait_bus_busy(struct device *dev)
{
	bool is_busy = true;
	uint32_t read_value;
	status_t status;
	flexspi_transfer_t flash_transfer;

	struct flash_priv *priv = dev->driver_data;
	FLEXSPI_Type *base_address = priv->pflash_block_base;

	flash_transfer.deviceAddress = 0;
	flash_transfer.port = kFLEXSPI_PortA1;
	flash_transfer.cmdType = kFLEXSPI_Read;
	flash_transfer.SeqNumber = 1;
	flash_transfer.seqIndex = NOR_CMD_LUT_SEQ_IDX_READSTATUS;
	flash_transfer.data = &read_value;
	flash_transfer.dataSize = 1;

	do{
		status = FLEXSPI_TransferBlocking(base_address, &flash_transfer);
		if (status != kStatus_Success) {
			return status;
		}
		// Status bit information for IS25LP064.
		//
		// The Write In Progress (WIP) bit is read-only, and can be used to
		// detect the progress or completion of a program or erase operation. When the
		// WIP bit is “0”, the device is ready for write status register, program or
		// erase operation. When the WIP bit is “1”, the device is busy.
		is_busy = ((read_value & 0x01UL) == 0x01UL) ? true : false;
	} while (is_busy);

	return status;
}

status_t flash_mcux_flexspi_qspi_write_enable(struct device *dev, off_t offset)
{
	status_t status;
	flexspi_transfer_t flash_transfer;

	struct flash_priv *priv = dev->driver_data;
	FLEXSPI_Type *base_address = priv->pflash_block_base;

	flash_transfer.deviceAddress = offset;
	flash_transfer.port = kFLEXSPI_PortA1;
	flash_transfer.cmdType = kFLEXSPI_Command;
	flash_transfer.SeqNumber = 1;
	flash_transfer.seqIndex = NOR_CMD_LUT_SEQ_IDX_WRITEENABLE;

	status = FLEXSPI_TransferBlocking(base_address, &flash_transfer);

	return status;
}

static int flash_mcux_flexspi_qspi_read(struct device *dev, off_t offset,
					void *data, size_t len)
{
	status_t status;
	flexspi_transfer_t flash_transfer;
	unsigned int key;

	key = irq_lock();

	struct flash_priv *priv = dev->driver_data;
	FLEXSPI_Type *base_address = priv->pflash_block_base;

	flash_transfer.deviceAddress = offset;
	flash_transfer.port = kFLEXSPI_PortA1;
	flash_transfer.cmdType = kFLEXSPI_Read;
	flash_transfer.SeqNumber = 1;
	flash_transfer.seqIndex = NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD;
	flash_transfer.data = (uint32_t *)data;
	flash_transfer.dataSize = len;

	status = FLEXSPI_TransferBlocking(base_address, &flash_transfer);
	if (status == kStatus_Success) {
		status = flash_mcux_flexspi_qspi_wait_bus_busy(dev);
	}

	irq_unlock(key);

	return status;
}

static int flash_mcux_flexspi_qspi_write(struct device *dev, off_t offset,
					 const void *data, size_t len)
{
	status_t status = kStatus_Fail;
	flexspi_transfer_t flash_transfer;
	unsigned int key;
	uint32_t page_size = DT_PROP(SOC_NV_FLASH_NODE, write_block_size);

	struct flash_priv *priv = dev->driver_data;
	FLEXSPI_Type *base_address = priv->pflash_block_base;

	if (k_sem_take(&priv->write_lock, K_NO_WAIT)) {
		return -EACCES;
	}

	key = irq_lock();

	uint32_t bytes_to_write = 0;
	size_t left = len;
	uint32_t *p = (uint32_t *)data;
	off_t d = offset;
	while (left != 0) {
		if (left > page_size)
		{
			if ((uint32_t) d % page_size)
			{
				bytes_to_write = page_size - ((uint32_t) d % page_size);
			}
			else
			{
				bytes_to_write = page_size;
			}
		}
		else
		{
			if ((uint32_t) (d % page_size) + left <= page_size)
			{
				bytes_to_write = left;
			}
			else
			{
				bytes_to_write = page_size - (uint32_t) (d % page_size);
			}
		}

		status = flash_mcux_flexspi_qspi_write_enable(dev, d);
		if (status != kStatus_Success) {
			break;
		}

		flash_transfer.deviceAddress = d;
		flash_transfer.port = kFLEXSPI_PortA1;
		flash_transfer.cmdType = kFLEXSPI_Write;
		flash_transfer.SeqNumber = 1;
		flash_transfer.seqIndex = NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD;
		flash_transfer.data = p;
		flash_transfer.dataSize = bytes_to_write;

		status = FLEXSPI_TransferBlocking(base_address, &flash_transfer);
		if (status != kStatus_Success) {
			break;
		}
		status = flash_mcux_flexspi_qspi_wait_bus_busy(dev);
		if (status != kStatus_Success) {
			break;
		}
		FLEXSPI_SoftwareReset(base_address);
		p += bytes_to_write;
		d += bytes_to_write;
		left -= bytes_to_write;
	}

	irq_unlock(key);
	k_sem_give(&priv->write_lock);

	return status;
}

static int flash_mcux_flexspi_qspi_erase(struct device *dev, off_t offset, size_t len)
{
	status_t status = kStatus_Fail;
	flexspi_transfer_t flash_transfer;
	unsigned int key;

	struct flash_priv *priv = dev->driver_data;
	FLEXSPI_Type *base_address = priv->pflash_block_base;

	if (k_sem_take(&priv->write_lock, K_NO_WAIT)) {
		return -EACCES;
	}

	key = irq_lock();

	int sectors = len / DT_PROP(SOC_NV_FLASH_NODE, erase_block_size);

	if (len % DT_PROP(SOC_NV_FLASH_NODE, erase_block_size)) {
		sectors++;
	}

	for (int i = 0; i < sectors; i++) {
		off_t offset_to_sector = offset + i * DT_PROP(SOC_NV_FLASH_NODE, erase_block_size);
		status = flash_mcux_flexspi_qspi_write_enable(dev, offset_to_sector);
		if (status == kStatus_Success) {
			flash_transfer.deviceAddress = offset_to_sector;
			flash_transfer.port = kFLEXSPI_PortA1;
			flash_transfer.cmdType = kFLEXSPI_Command;
			flash_transfer.SeqNumber = 1;
			flash_transfer.seqIndex = NOR_CMD_LUT_SEQ_IDX_ERASESECTOR;

			status = FLEXSPI_TransferBlocking(base_address, &flash_transfer);
			if (status == kStatus_Success) {
				status = flash_mcux_flexspi_qspi_wait_bus_busy(dev);
				FLEXSPI_SoftwareReset(base_address);
			}
		}
	}

	irq_unlock(key);
	k_sem_give(&priv->write_lock);

	return status;
}

/* Write/erase operations in this driver is protected by a semaphore. This
 * prevents access from multiple threads, but using this function the semaphore
 * can be locked, preventing all write/erase operations. */
static int flash_mcux_flexspi_qspi_write_protection(struct device *dev, bool enable)
{
	struct flash_priv *priv = dev->driver_data;
	int rc = 0;

	if (enable) {
		rc = k_sem_take(&priv->write_lock, K_FOREVER);
	} else {
		k_sem_give(&priv->write_lock);
	}

	return rc;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static const struct flash_pages_layout dev_layout = {
	.pages_count = KB(CONFIG_FLASH_SIZE) /
				DT_PROP(SOC_NV_FLASH_NODE, erase_block_size),
	.pages_size = DT_PROP(SOC_NV_FLASH_NODE, erase_block_size),
};

static void flash_mcux_flexspi_qspi_pages_layout(struct device *dev,
						 const struct flash_pages_layout **layout,
						 size_t *layout_size)
{
	*layout = &dev_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static struct flash_priv flash_data;

static const struct flash_driver_api flash_mcux_flexspi_qspi_api = {
	.write_protection = flash_mcux_flexspi_qspi_write_protection,
	.erase = flash_mcux_flexspi_qspi_erase,
	.write = flash_mcux_flexspi_qspi_write,
	.read = flash_mcux_flexspi_qspi_read,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_mcux_flexspi_qspi_pages_layout,
#endif
	.write_block_size = DT_PROP(SOC_NV_FLASH_NODE, write_block_size),
};

static int flash_mcux_flexspi_qspi_init(struct device *dev)
{
	// Get the parent node of the chosen flash chip to get the reg address of the FlexSPI device
	FLEXSPI_Type *base = (FLEXSPI_Type *)DT_REG_ADDR_BY_IDX(DT_INST(0, nxp_imx_flexspi), 0);
	struct flash_priv *priv = dev->driver_data;

	priv->pflash_block_base = base;
	unsigned int key;

	key = irq_lock();

	flexspi_config_t config;

	/*Get FLEXSPI default settings and configure the flexspi. */
	FLEXSPI_GetDefaultConfig(&config);

	/*Set AHB buffer size for reading data through AHB bus. */
	config.ahbConfig.enableAHBPrefetch = true;
	config.ahbConfig.enableAHBBufferable = true;
	config.ahbConfig.enableReadAddressOpt = true;
	config.ahbConfig.enableAHBCachable = true;
	config.rxSampleClock = kFLEXSPI_ReadSampleClkLoopbackFromDqsPad;
	FLEXSPI_Init(base, &config);

	/* Configure flash settings according to serial flash feature. */
	FLEXSPI_SetFlashConfig(base, &deviceconfig, kFLEXSPI_PortA1);

	/* Update LUT table. */
	FLEXSPI_UpdateLUT(base, 0, customLUT, CUSTOM_LUT_LENGTH);

	/* Do software reset. */
	FLEXSPI_SoftwareReset(base);

	irq_unlock(key);

	k_sem_init(&priv->write_lock, 0, 1);

	return 0;
}

DEVICE_AND_API_INIT(flash_mcux, "FLASH_0",
			flash_mcux_flexspi_qspi_init, &flash_data, NULL, POST_KERNEL,
			CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &flash_mcux_flexspi_qspi_api);

