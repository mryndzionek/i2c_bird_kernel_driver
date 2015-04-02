/*
 * Driver for the I2C Bird USB-I2C adapter
 *
 * Copyright (C) 2014-2015 MpicoSys Embedded Pico Systems Sp. z o.o.
 * Author: Mariusz Ryndzionek <mariusz.ryndzionek@mpicosys.com>
 *
 * Derived from:
 *  i2c-diolan-u2c.c
 *  Copyright (c) 2010-2011 Ericsson AB
 *
 *  iicbird CLI tools
 *  Copyright (C) 2007 Frank Hartmann (soundart@gmx.net)
 *                     Hartmut Hackmann <hartmut.hackmann@t-online.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 */

//#define DEBUG 1

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/i2c.h>

#define DRIVER_NAME		"i2c-bird"

#define USB_VENDOR_ID_IICBIRD		0x0471
#define USB_DEVICE_ID_IICBIRD   	0x0834

#ifdef __BIG_ENDIAN
#define ENDIAN 1
#else
#define ENDIAN 0
#endif

#define NO_ERROR               0x00	/* transmission successfully completed */
#define I2C_BUS_JAM            0x01	/* SCL and SDA low */
#define I2C_SEQENCE_ERROR      0x02	/* sequence not possible e.g. new message without start */
#define I2C_TIME_OUT           0x03	/* transmission not finished in time */
#define I2C_HOLD               0x04	/* transmission successfully completed, bus hold */
#define I2C_SCL_LOW            0x05	/* SCL is permanent low */
#define I2C_SDA_LOW            0x06	/* SDA is permanent low */

#define I2C_NO_ADDRACK         0x20	/* no slave has acknowledge the slave address */
#define I2C_NO_DATAACK         0x30	/* the slave has not acknowledged the data */
#define I2C_ARBIT_LOST         0x38	/* arbitration lost */
#define I2C_ERROR              0xf8	/* unknown error */

#define CMD_SET_TERMINATION     0x00
#define CMD_SET_POWER_FEED      0x01
#define CMD_IDENTIFY            0x07
#define CMD_GET_CAPABILITIES    0x05
#define CMD_GET_FW_VERSION	    0x02
#define CMD_GET_INTERFACE_TYPE  0x09
#define CMD_SET_INTERFACE_TYPE  0x14

#define  CAPS_USB      1
#define  CAPS_IDENTIFY 2
#define  CAPS_GPIO     4
#define  CAPS_SCRIPT   8
#define  CAPS_I2C      16
#define  CAPS_SPI      32

#define IICBIRD_SPEED_FAST	400	/* 400 kHz */
#define IICBIRD_SPEED_STD	100	/* 100 kHz */
#define IICBIRD_SPEED_1KHZ	1	/* 1 kHz, minimum speed */
#define IICBIRD_SPEED(f)	((DIV_ROUND_UP((f), 1000)))

#define IICBIRD_FREQ_FAST	400000
#define IICBIRD_FREQ_STD	100000
#define IICBIRD_FREQ(s)		(1000 * s)

#define IICBIRD_USB_TIMEOUT	    1000	/* in ms */
#define IICBIRD_SYNC_TIMEOUT	20	/* in ms */

/*#define FW_SET_MODE                  3 */
#define FW_READ_MODE                 4
#define FW_WRITE_MODE                5
#define FW_WRITE_READ_MODE           6

#define FW_READ_BUS                  0
#define FW_WRITE_BUS                 1
#define FW_WRITE_READ_BUS            2

/* CommandExtension, for the i2c commands */
/* The bits can be logically ored         */

#define NORMAL_I2C_OPERATION         0	/* generate start and stop condition related to the */
/* current transfer */
#define NO_START_CONDITION           1	/* do not generate a start condition */
#define NO_STOP_CONDITION            2	/* do not generate a stop condition */
#define FOLLOW_START_CONDITION       4	/* a start condition will occur in the next transfer */

#define BIRD_USB_ERROR          0xfd	/* USB error like "open failed" */

struct iicbird_cmd {
	u16 EndianType;		/* 0 for little endian, 1 for big endian   */
	u16 Command;
	u16 CommandExtension;
	u16 WriteDataLength;	/* bytes to write in the data buffer part  */
	u16 ReadDataLength;	/* bytes to read                           */
	u16 TimeOut;		/* timeout in ms                           */
	u16 Address;		/* 7 / 10 bit slave address or prog offset */
	u16 Bitrate;		/* transfer rate in kbit/second            */
	u8 data[I2C_SMBUS_BLOCK_MAX + 1];
};

/* Structure to hold all of our device specific stuff */
struct i2c_bird {
	u16 speed;
	int ep_in, ep_out;	/* Endpoints    */
	struct usb_device *usb_dev;	/* the usb device for this device */
	struct usb_interface *interface;	/* the interface for this device */
	struct i2c_adapter adapter;	/* i2c related things */
};

static uint frequency = IICBIRD_FREQ_STD;	/* I2C clock frequency in Hz */
static bool powerfeed = false;
static uint pullups = 0;

module_param(frequency, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(frequency,
		 "I2C clock frequency in hertz (default: 100000)");

module_param(powerfeed, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(powerfeed, "5V USB power feed (default: false)");

module_param(pullups, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(pullups, "Pullups config [0,1,2] (default: 0)");

/* usb layer */

static int iicbird_transfer(struct i2c_bird *dev,
			    unsigned char address, unsigned short command,
			    unsigned short command_ext,
			    unsigned char *wr_msg,
			    unsigned int wr_msg_length,
			    unsigned char *rd_msg,
			    unsigned int rd_msg_length, u16 speed)
{
	struct iicbird_cmd cmd;
	int usbres, i, iicres, transferred;

	cmd.EndianType = ENDIAN;
	cmd.Command = command;
	cmd.CommandExtension = command_ext;
	cmd.WriteDataLength = wr_msg_length;
	cmd.ReadDataLength = rd_msg_length + 1;
	cmd.TimeOut = 20;
	cmd.Address = (address >> 1);
	cmd.Bitrate = speed;

	for (i = 0; i < wr_msg_length; i++) {
		cmd.data[i] = wr_msg[i];
	}
	usbres = usb_bulk_msg(dev->usb_dev,
			      usb_sndbulkpipe(dev->usb_dev, dev->ep_out),
			      (void *) &cmd,
			      sizeof(cmd) - I2C_SMBUS_BLOCK_MAX +
			      wr_msg_length, &transferred,
			      IICBIRD_USB_TIMEOUT);

	if (usbres != 0) {
		return BIRD_USB_ERROR;
	}
	usbres = usb_bulk_msg(dev->usb_dev,
			      usb_rcvbulkpipe(dev->usb_dev, dev->ep_in),
			      (void *) &cmd.data, rd_msg_length + 1,
			      &transferred, IICBIRD_USB_TIMEOUT);

	if (usbres != 0) {
		return BIRD_USB_ERROR;
	}
	iicres = cmd.data[0];

	if (transferred < rd_msg_length) {
		rd_msg_length = transferred;
		/* Hac: Do we need a special return code for less data? */
	}
	for (i = 0; i < rd_msg_length; i++) {
		rd_msg[i] = cmd.data[i + 1];
	}
	return iicres;
}

static int iicbird_power_sw(struct i2c_bird *dev, int mode)
{
	unsigned char wr_msg[2];
	unsigned char rd_msg;
	wr_msg[0] = CMD_SET_POWER_FEED;
	wr_msg[1] = mode;

	/* Hac: fixme check power level before switching on */
	return iicbird_transfer(dev, 0, FW_WRITE_MODE, 0, wr_msg, 2,
				&rd_msg, 0, 100);
}

static int iicbird_pullup(struct i2c_bird *dev, int mode)
{
	unsigned char wr_msg[2];
	unsigned char rd_msg;
	wr_msg[0] = CMD_SET_TERMINATION;
	wr_msg[1] = mode;
	return iicbird_transfer(dev, 0, FW_WRITE_MODE, 0, wr_msg, 2,
				&rd_msg, 0, 100);
}

static int iicbird_identify(struct i2c_bird *dev, unsigned int seconds)
{
	unsigned char wr_msg[2];
	unsigned char rd_msg;
	wr_msg[0] = CMD_IDENTIFY;
	wr_msg[1] = seconds;
	return iicbird_transfer(dev, 0, FW_WRITE_MODE, 0, wr_msg, 2,
				&rd_msg, 0, 100);
}

static int iicbird_get_capabilities(struct i2c_bird *dev)
{
	unsigned char wr_msg;
	unsigned char rd_msg[4];
	char res;
	wr_msg = CMD_GET_CAPABILITIES;
	res =
	    iicbird_transfer(dev, 0, FW_WRITE_READ_MODE, 0, &wr_msg, 1,
			     rd_msg, 4, 100);

	if (res == 0) {
		dev_info(&dev->interface->dev,
			 "I2C Bird capabilities %x\n",
			 (unsigned int) rd_msg[0]);
		return rd_msg[0];
	} else
		return -1;
}

static int iicbird_fw_version(struct i2c_bird *dev)
{
	unsigned char wr_msg;
	unsigned char rd_msg[2];
	char res;
	wr_msg = CMD_GET_FW_VERSION;
	res =
	    iicbird_transfer(dev, 0, FW_WRITE_READ_MODE, 0, &wr_msg, 1,
			     rd_msg, 2, 100);

	if (res == 0) {
		dev_info(&dev->interface->dev,
			 "I2C Bird firmware version %u.%u\n",
			 (unsigned int) rd_msg[1],
			 (unsigned int) rd_msg[0]);
		return 0;
	} else
		return -1;
}

static int iicbird_init(struct i2c_bird *dev)
{
	int ret;

	if (frequency >= 400000) {
		dev->speed = IICBIRD_SPEED_FAST;
		frequency = IICBIRD_FREQ_FAST;
	} else if (frequency < 1000) {
		dev->speed = IICBIRD_SPEED_1KHZ;
		frequency = IICBIRD_FREQ(dev->speed);
	} else {
		dev->speed = IICBIRD_SPEED(frequency);
		frequency = IICBIRD_FREQ(dev->speed);
	}

	dev_info(&dev->interface->dev,
		 "I2C Bird at USB bus %03d address %03d speed %d Hz\n",
		 dev->usb_dev->bus->busnum, dev->usb_dev->devnum,
		 frequency);

	ret = iicbird_fw_version(dev);
	if (ret < 0) {
		dev_err(&dev->interface->dev,
			"Failed to get firmware version\n");
		return ret;
	}

	ret = iicbird_get_capabilities(dev);
	if (ret < 0) {
		dev_err(&dev->interface->dev,
			"Failed to get capabilities\n");
		return -1;
	} else {
		if (!(ret & CAPS_I2C)) {
			dev_err(&dev->interface->dev,
				"I2C capability not supported\n");
			return -1;
		}
	}

	if ((pullups > 1) && (pullups < 4)) {
		dev_info(&dev->interface->dev, "Setting pullups to %d\n",
			 pullups);
		ret = iicbird_pullup(dev, pullups);
		if (ret < 0) {
			dev_err(&dev->interface->dev,
				"Failed to set pullups\n");
			return -1;
		}
	}

	if (powerfeed) {
		dev_info(&dev->interface->dev,
			 "Activating USB power feed\n");
		ret = iicbird_power_sw(dev, 1);
		if (ret < 0) {
			dev_err(&dev->interface->dev,
				"Failed to set USB power feed\n");
			return -1;
		}
	}

	ret = iicbird_identify(dev, 2);

	return ret;
}

/* i2c layer */

static int iicbird_write_xfer(struct i2c_bird *dev, u8 begin_cmd_ext,
			      u8 end_cmd_ext, struct i2c_msg *pmsg)
{
	int res;
	u16 command_ext;

	res = NO_ERROR;

	dev_dbg(&dev->interface->dev,
		"  %s (flags %d) %d bytes to 0x%02x\n",
		pmsg->flags & I2C_M_RD ? "read" : "write", pmsg->flags,
		pmsg->len, pmsg->addr);

	if (pmsg->len <= I2C_SMBUS_BLOCK_MAX) {
		command_ext =
		    NORMAL_I2C_OPERATION | NO_START_CONDITION |
		    NO_STOP_CONDITION;
		if (begin_cmd_ext) {
			command_ext &= ~NO_START_CONDITION;
		}

		if (end_cmd_ext) {
			command_ext &= ~NO_STOP_CONDITION;
		}

		res = iicbird_transfer(dev, pmsg->addr << 1, FW_WRITE_BUS,
				       command_ext, &pmsg->buf[0],
				       pmsg->len, NULL, 0, dev->speed);
		dev_dbg(&dev->interface->dev,
			"I2C wr transfer returned 0x%02x (0x%02x)\n", res,
			command_ext);
		if ((res != NO_ERROR) && (res != I2C_HOLD)) {
			switch (res) {
			case I2C_TIME_OUT:
				return -ETIMEDOUT;

			case I2C_NO_ADDRACK:
				return -ENXIO;

			default:
				return -EIO;
			}
		}
	} else
		return -EPROTO;

	return res;
}

static int iicbird_read_xfer(struct i2c_bird *dev, u8 begin_cmd_ext,
			     u8 end_cmd_ext, struct i2c_msg *pmsg)
{
	int res, offs;
	u16 command_ext;

	res = NO_ERROR;
	offs = 0;

	dev_dbg(&dev->interface->dev,
		"  %s (flags %d) %d bytes from 0x%02x\n",
		pmsg->flags & I2C_M_RD ? "read" : "write", pmsg->flags,
		pmsg->len, pmsg->addr);

	if (pmsg->len <= I2C_SMBUS_BLOCK_MAX) {
		command_ext =
		    NORMAL_I2C_OPERATION | NO_START_CONDITION |
		    NO_STOP_CONDITION;
		if (begin_cmd_ext) {
			command_ext &= ~NO_START_CONDITION;
		}

		if (pmsg->flags & I2C_M_RECV_LEN) {
			res =
			    iicbird_transfer(dev, pmsg->addr << 1,
					     FW_READ_BUS, command_ext,
					     NULL, 0, &pmsg->buf[0], 1,
					     dev->speed);

			dev_dbg(&dev->interface->dev,
				"I2C rd transfer returned 0x%02x\n", res);

			if ((res != NO_ERROR) && (res != I2C_HOLD)) {
				switch (res) {
				case I2C_TIME_OUT:
					return -ETIMEDOUT;

				case I2C_NO_ADDRACK:
					return -ENXIO;

				default:
					return -EIO;
				}
			}

			dev_dbg(&dev->interface->dev, "--> %d\n",
				pmsg->len);

			pmsg->len += pmsg->buf[0];
			if (pmsg->len > I2C_SMBUS_BLOCK_MAX + 1)
				return -EPROTO;

			offs++;

			dev_dbg(&dev->interface->dev, "---> %d\n",
				pmsg->len);

			command_ext =
			    NORMAL_I2C_OPERATION | NO_START_CONDITION |
			    NO_STOP_CONDITION;
		}

		if (end_cmd_ext) {
			command_ext &= ~NO_STOP_CONDITION;
		}

		res = iicbird_transfer(dev, pmsg->addr << 1, FW_READ_BUS,
				       command_ext, NULL, 0,
				       &pmsg->buf[offs], pmsg->len - offs,
				       dev->speed);

		dev_dbg(&dev->interface->dev,
			"I2C rd transfer returned 0x%02x\n", res);
		if ((res != NO_ERROR) && (res != I2C_HOLD)) {
			switch (res) {
			case I2C_TIME_OUT:
				return -ETIMEDOUT;

			case I2C_NO_ADDRACK:
				return -ENXIO;

			default:
				return -EIO;
			}
		}
	} else
		return -EPROTO;

	return res;
}

static int iicbird_usb_xfer(struct i2c_adapter *adapter,
			    struct i2c_msg *msgs, int num)
{
	int res;
	int j;
	struct i2c_msg *pmsg;
	struct i2c_bird *dev = i2c_get_adapdata(adapter);

	res = NO_ERROR;
	j = 0;

	dev_dbg(&dev->interface->dev,
		"starting I2C transfer num=%d speed=%d\n", num,
		dev->speed);

	while (j < num) {
		pmsg = &msgs[j];

		if (pmsg->flags & I2C_M_RD) {

			res = iicbird_read_xfer(dev, 1, 1, pmsg);
			if ((res != NO_ERROR) && (res != I2C_HOLD)) {
				return res;
			}

		} else {
			if (j < num - 1) {
				if (msgs[j + 1].flags & I2C_M_RD) {
					res =
					    iicbird_write_xfer(dev, 1, 0,
							       pmsg);
					if ((res != NO_ERROR)
					    && (res != I2C_HOLD)) {
						return res;
					}
					pmsg = &msgs[++j];
					res =
					    iicbird_read_xfer(dev, 1, 1,
							      pmsg);
					if ((res != NO_ERROR)
					    && (res != I2C_HOLD)) {
						return res;
					}

				} else {
					res =
					    iicbird_write_xfer(dev, 1, 1,
							       pmsg);
					if ((res != NO_ERROR)
					    && (res != I2C_HOLD)) {
						return res;
					}
				}
			} else {
				res = iicbird_write_xfer(dev, 1, 1, pmsg);
				if ((res != NO_ERROR) && (res != I2C_HOLD)) {
					return res;
				}
			}
		}
		j++;
	}

	return num;
}

/*
 * Return list of supported functionality.
 */
static u32 iicbird_usb_func(struct i2c_adapter *a)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
	//I2C_FUNC_SMBUS_READ_BLOCK_DATA | I2C_FUNC_SMBUS_BLOCK_PROC_CALL;
}

static const struct i2c_algorithm iicbird_usb_algorithm = {.master_xfer =
	    iicbird_usb_xfer,.functionality = iicbird_usb_func,
};

/* device layer */

static const struct usb_device_id i2c_bird_table[] =
    { {USB_DEVICE(USB_VENDOR_ID_IICBIRD, USB_DEVICE_ID_IICBIRD)}, {}
};

MODULE_DEVICE_TABLE(usb, i2c_bird_table);

static void i2c_bird_free(struct i2c_bird *dev)
{
	usb_put_dev(dev->usb_dev);
	kfree(dev);
}

static int i2c_bird_probe(struct usb_interface *interface,
			  const struct usb_device_id *id)
{
	struct usb_host_interface *hostif = interface->cur_altsetting;
	struct i2c_bird *dev;
	int ret;

	if (hostif->desc.bInterfaceNumber != 0
	    || hostif->desc.bNumEndpoints < 2)
		return -ENODEV;

	/* allocate memory for our device state and initialize it */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (dev == NULL) {
		ret = -ENOMEM;
		goto error;
	}
	dev->ep_out = hostif->endpoint[1].desc.bEndpointAddress;
	dev->ep_in = hostif->endpoint[0].desc.bEndpointAddress;

	dev->usb_dev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);

	/* setup i2c adapter description */
	dev->adapter.owner = THIS_MODULE;
	dev->adapter.class = I2C_CLASS_HWMON;
	dev->adapter.algo = &iicbird_usb_algorithm;
	i2c_set_adapdata(&dev->adapter, dev);
	snprintf(dev->adapter.name, sizeof(dev->adapter.name),
		 DRIVER_NAME " at bus %03d device %03d",
		 dev->usb_dev->bus->busnum, dev->usb_dev->devnum);

	dev->adapter.dev.parent = &dev->interface->dev;

	/* initialize diolan i2c interface */
	ret = iicbird_init(dev);
	if (ret < 0) {
		dev_err(&interface->dev, "failed to initialize adapter\n");
		goto error_free;
	}

	/* and finally attach to i2c layer */
	ret = i2c_add_adapter(&dev->adapter);
	if (ret < 0) {
		dev_err(&interface->dev, "failed to add I2C adapter\n");
		goto error_free;
	}

	dev_info(&interface->dev, "connected " DRIVER_NAME "\n");

	return 0;

      error_free:usb_set_intfdata(interface, NULL);
	i2c_bird_free(dev);
      error:return ret;
}

static void i2c_bird_disconnect(struct usb_interface *interface)
{
	struct i2c_bird *dev = usb_get_intfdata(interface);

	i2c_del_adapter(&dev->adapter);
	usb_set_intfdata(interface, NULL);
	i2c_bird_free(dev);

	dev_dbg(&interface->dev, "disconnected\n");
}

static struct usb_driver i2c_bird_driver = {.name = DRIVER_NAME,.probe =
	    i2c_bird_probe,.disconnect = i2c_bird_disconnect,.id_table =
	    i2c_bird_table,
};

module_usb_driver(i2c_bird_driver);

MODULE_AUTHOR("Mariusz Ryndzionek <mryndzionek@gmail.com>");
MODULE_DESCRIPTION(DRIVER_NAME " driver");
MODULE_LICENSE("GPL");
