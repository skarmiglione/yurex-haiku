/*
 * $Id$
 *
 * Copyright (c) 2010 Takashi TOYOSHIMA <toyoshim@gmail.com>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/* Driver for Maywa-Denki & KAYAC YUREX BBU sensor */

#include <OS.h>
#include <KernelExport.h>
#include <Drivers.h>
#include <USB3.h>
#include <usb/USB_hid.h>
#include <string.h>

//#define DEBUG_YUREX

#if !defined(DEBUG_YUREX)
# define TRACE(x...)
#else // !defined(DEBUG_YUREX)
# define TRACE(x...) dprintf("yurex: "x)
#endif // !defined(DEBUG_YUREX)
#define TRACE_ALWAYS(x...) dprintf("yurex: "x)

// driver name
#define DRIVER_NAME "yurex"
static const char *kDriverName = DRIVER_NAME;

// device name
static const char *kDeviceName = "misc/" DRIVER_NAME "/%08ld/%s";

// supported usb type
#define USB_VENDOR_MICRODIA		0x0c45
#define USB_PRODUCT_MICRODIA_YUREX	0x1010
static usb_support_descriptor sSupportedDevices[1] = {
	{
		USB_HID_DEVICE_CLASS,			// Class
		USB_HID_INTERFACE_BOOT_SUBCLASS,	// Subclass
		0,					// Protocol
		USB_VENDOR_MICRODIA,			// Vendor ID
		USB_PRODUCT_MICRODIA_YUREX		// Product ID
	},
};

// driver api version
int32 api_version = B_CUR_DRIVER_API_VERSION;

// usb definition
#define USB_DESCRIPTOR_HID		0x21
#define USB_DESCRIPTOR_HID_REPORT	0x22

// usb module information
static usb_module_info *gUsb;

// device instance variables
typedef struct _device {
	struct _device *next;			// device list link
	usb_device      udev;			// usb device ID
	char            name_bbu[256];		// bbu device pathname
	char            name_anime[256];	// anime device pathname
	size_t          ifno;			// interface ID
	int             ep_detect;		// endpoint informations are valid?
	uint8           ep_address;		//   endpoint address
	usb_pipe        ep;			//   endpoint pipe handle
	sem_id          sem;			// semaphoe to access work area
	uint64          bbu;			//   BBU count value (in 40-bit)
	int		anime;			//   animation 0:off / 1:on
	char		buf[8];			// dummy buffer
} device;

// transaction variables
#define YUREX_DEVICE_TYPE_BBU	0
#define YUREX_DEVICE_TYPE_ANIME	1
typedef struct _dev_open {
	device *dev;		// device instance variables
	int     type;		// device type 0:bbu / 1:anime
	uint8   buf[16];	// read buffer
	size_t  buf_len;	// read buffer length
} dev_open;

// global variables
static sem_id  gLock        = 0;	// semaphoe to access global variables
static uint32  gDeviceCount = 0;	// number of devices
static char  **gDeviceNames = NULL;	// published device pathnames
static device *gDeviceList  = NULL;	// device list

// callback definition
static status_t device_added(const usb_device dev, void **cookie);
static status_t device_removed(void *cookie);
static status_t device_open(const char *name, uint32 flags, void **cookie);
static status_t device_close(void *cookie);
static status_t device_free(void *cookie);
static status_t device_read(void *cookie, off_t position, void *buffer, size_t *length);
static status_t device_write(void *cookie, off_t position, const void *buffer, size_t *length);

// usb framework hooks
static usb_notify_hooks sNotifyHooks = {
	device_added,
	device_removed
};

// yurex command definition (based on OpenBSD uyurex.c)
#define CMD_NONE	0xf0
#define CMD_EOF		0x0d
#define CMD_ACK		0x21
#define CMD_MODE	0x41
#define CMD_VALUE	0x43
#define CMD_READ	0x52
#define CMD_WRITE	0x53
#define CMD_PADDING	0xff

// yurex functions definition
static void yurex_callback(void *cookie, status_t status, void *data, size_t actualLength);
static void yurex_set_mode(device *dev, uint8 val);
static void yurex_read_bbu(device *dev);
static void yurex_write_bbu(device *dev, uint64 bbu);
static void yurex_interrupt(device *dev);

//
// yurex functions
//

void
yurex_callback
(void *cookie, status_t status, void *data, size_t actualLength)
{
	device *dev = (device *)cookie;
	uint8 *req = (uint8 *)data;
	TRACE("callback: cookie=%p, len=%d\n", cookie, actualLength);
	TRACE(" cmd: %02x\n", req[0]);

	if ((CMD_VALUE == req[0]) || // BBU update notification
		(CMD_READ  == req[0])) { // BBU read result
		int i;
		acquire_sem(dev->sem);
		dev->bbu = 0;
		for (i = 1; i <= 5; i++) {
			dev->bbu <<= 8;
			dev->bbu += req[i];
		}
		release_sem(dev->sem);
		if (CMD_EOF != req[6])
			TRACE_ALWAYS("invalid bbu delta EOF\n");
		TRACE("bbu=%ld\n", dev->bbu);
	} else if ((CMD_ACK == req[0]) && (CMD_WRITE == req[1]))
		yurex_read_bbu(dev);

	// requeue interrupt
	if (0 != dev->ep_detect)
		yurex_interrupt(dev);
}

void
yurex_set_mode
(device *dev, uint8_t val)
{
	uint8 req[8];
	size_t actualLength;
	status_t result;

	memset(req, CMD_PADDING, sizeof(req));
	req[0] = CMD_MODE;
	req[1] = val;
	req[2] = CMD_EOF;
	result = gUsb->send_request(dev->udev,
		USB_REQTYPE_INTERFACE_OUT |
		USB_REQTYPE_CLASS,
		USB_REQUEST_HID_SET_REPORT,
		2 << 8, // Output Report
		dev->ifno,
		8,
		req,
		&actualLength);
	TRACE("output report: result=%d, len=%d\n", result, actualLength);
}

void
yurex_read_bbu
(device *dev)
{
	uint8 req[8];
	status_t result;
	size_t actualLength;

	memset(req, CMD_PADDING, sizeof(req));
	req[0] = CMD_READ;
	req[1] = CMD_EOF;
	result = gUsb->send_request(dev->udev,
		USB_REQTYPE_INTERFACE_OUT |
		USB_REQTYPE_CLASS,
		USB_REQUEST_HID_SET_REPORT,
		2 << 8, // Output Report
		dev->ifno,
		8,
		req,
		&actualLength);
	TRACE("read_req: result=%d, len=%d\n", result, actualLength);
}

void
yurex_write_bbu
(device *dev, uint64 bbu)
{
	uint8 req[8];
	status_t result;
	size_t actualLength;

	memset(req, CMD_PADDING, sizeof(req));
	req[0] = CMD_WRITE;
	req[1] = (bbu >> 32) & 0xff;
	req[2] = (bbu >> 24) & 0xff;
	req[3] = (bbu >> 16) & 0xff;
	req[4] = (bbu >>  8) & 0xff;
	req[5] = (bbu >>  0) & 0xff;
	req[6] = CMD_EOF;
	result = gUsb->send_request(dev->udev,
		USB_REQTYPE_INTERFACE_OUT |
		USB_REQTYPE_CLASS,
		USB_REQUEST_HID_SET_REPORT,
		2 << 8, // Output Report
		dev->ifno,
		8,
		req,
		&actualLength);
	TRACE("write_req: result=%d, len=%d\n", result, actualLength);
}

void
yurex_interrupt
(device *dev)
{
	status_t result;

	TRACE("yurex_interrupt(%p)\n", dev);
	result = gUsb->queue_interrupt(dev->ep,
		dev->buf,
		8,
		&yurex_callback,
		dev);
	TRACE("queue_interrupt: cookie=%p, result=%d\n", dev, result);
}

//
// driver api functions
//

status_t
init_hardware
(void)
{
	return B_OK;
}

status_t
init_driver
(void)
{
	TRACE("init_driver()\n");
	gDeviceList = NULL;
	gDeviceCount = 0;

	TRACE(" create_sem\n");
	gLock = create_sem(1, DRIVER_NAME "_driver_sem");
	if (gLock < B_OK)
		return B_ERROR;

	TRACE(" get usb module\n");
	if (B_OK != get_module(B_USB_MODULE_NAME, (module_info **)&gUsb))
		return B_ERROR;

	TRACE(" register/install\n");
	gUsb->register_driver(kDriverName, sSupportedDevices, 1, NULL);
	gUsb->install_notify(kDriverName, &sNotifyHooks);

	return B_OK;
}

void
uninit_driver
(void)
{
	TRACE("uninit_driver()\n");

	TRACE(" uninstall\n");
	gUsb->uninstall_notify(kDriverName);

	TRACE(" put usb module\n");
	put_module(B_USB_MODULE_NAME);

	TRACE(" free resource\n");
	acquire_sem(gLock);
	if (NULL != gDeviceNames) {
		int i;
		for (i = 0; NULL != gDeviceNames[i]; i++)
			free(gDeviceNames[i]);
		free(gDeviceNames);
		gDeviceNames = NULL;
	}
	release_sem(gLock);
	delete_sem(gLock);
}

const char **
publish_devices
(void)
{
	status_t result = B_OK;
	TRACE("publish_devices()\n");

	acquire_sem(gLock);

	TRACE(" free resources\n");
	if (NULL != gDeviceNames) {
		int i;
		for (i = 0; NULL != gDeviceNames[i]; i++)
			free(gDeviceNames[i]);
		free(gDeviceNames);
		gDeviceNames = NULL;
	}

	TRACE(" allocate device names\n");

	gDeviceNames = (char **)malloc(sizeof(char *) * (gDeviceCount * 2 + 1));
	if (NULL != gDeviceNames) {
		int i;
		int devices = gDeviceCount * 2;
		device *device = gDeviceList;
		for (i = 0; i < devices; i += 2) {
			gDeviceNames[i + 0] = strdup(device->name_bbu);
			gDeviceNames[i + 1] = strdup(device->name_anime);
			device = device->next;
		}
		gDeviceNames[devices] = NULL;
	} else
		result = B_ERROR;

	release_sem(gLock);

	if (B_OK != result)
		return NULL;

	return (const char **)gDeviceNames;
}

device_hooks *
find_device
(const char *name)
{
	static device_hooks hooks = {
		&device_open,
		&device_close,
		&device_free,
		NULL,
		&device_read,
		&device_write,
		NULL,
		NULL,
		NULL,
		NULL
	};
	TRACE_ALWAYS("find_device(%s)\n", name);
	return &hooks;
}

//
// driver hook functions
//

status_t
device_added
(const usb_device udev, void **cookie)
{
	device *dev;
	const usb_configuration_info *conf;
	size_t i, j;
	status_t result = B_OK;

	TRACE("device_added(0x%08lx)\n", (int32)udev);

	// initialize instance
	dev = (device *)malloc(sizeof(device));
	*cookie = (void *)dev;
	if (NULL == *cookie)
		return B_ERROR;

	memset(dev, 0, sizeof(device));
	dev->sem   = create_sem(1, DRIVER_NAME "_instance_sem");
	dev->udev  = udev;
	dev->anime = 1;
	snprintf(dev->name_bbu  , 256, kDeviceName, udev, "bbu");
	snprintf(dev->name_anime, 256, kDeviceName, udev, "animation");

	// add instance
	acquire_sem(gLock);
	TRACE(" add instance(%d)\n", gDeviceCount + 1);

	dev->next = gDeviceList;
	gDeviceList = dev;
	gDeviceCount++;

	release_sem(gLock);

	// get usb device default configuration
	conf = gUsb->get_nth_configuration(udev, 0);
	if (NULL == conf)
		return B_ERROR;

	// interface check
	for (i = 0; i < conf->interface_count; i++) {
		usb_interface_info *intf = conf->interface[i].active;
		if (NULL == intf) continue;
		
		// endpoint check on active interface
		for (j = 0; j < intf->endpoint_count; j++) {
			usb_endpoint_info *ep = &intf->endpoint[j];
			usb_endpoint_descriptor *epd;
			if (NULL == ep)
				continue;
			epd = ep->descr;
			if ((USB_ENDPOINT_ATTR_INTERRUPT != epd->attributes) ||
				(USB_ENDPOINT_ADDR_DIR_IN !=
				 (epd->endpoint_address & USB_ENDPOINT_ADDR_DIR_IN)) ||
				(8 != epd->max_packet_size))
				continue;
			dev->ep         = ep->handle;
			dev->ep_address = epd->endpoint_address;
			dev->ep_detect  = 1;
			dev->ifno       = i;
			break;
		}
		if (0 != dev->ep_detect)
			break;
	}
	if (0 == dev->ep_detect)
		TRACE_ALWAYS("can not find suitable endpoint\n");

	// set usb device configuration	
	if (B_OK != gUsb->set_configuration(dev->udev, conf))
		TRACE_ALWAYS("can not set default configuration\n");

	// initialize yurex
	yurex_set_mode(dev, 0);
	yurex_read_bbu(dev);
	yurex_interrupt(dev);

	return B_OK;
}

status_t
device_removed
(void *cookie)
{
	device *dev = (device *)cookie;
	TRACE("device_removed(0x%08lx)\n", dev->udev);

	// remove instance
	acquire_sem(gLock);
	TRACE("remove instance(%d)\n", gDeviceCount - 1);

	if (gDeviceList == dev) gDeviceList = dev->next;
	else {
		device *list;
		for (list = gDeviceList; NULL != list; list = list->next) {
			if (list->next == dev) {
				list->next = dev->next;
				break;
			}
		}
	}
	gDeviceCount--;

	release_sem(gLock);

	// flush usb transactions
	dev->ep_detect = 0; // forbit interrupt requeue
	gUsb->cancel_queued_transfers(dev->ep);

	// free resource
	free(cookie);

	return B_OK;
}

status_t
device_open
(const char *name, uint32 flags, void **cookie)
{
	device *list;
	dev_open *dev = (dev_open *)malloc(sizeof(dev_open));
	TRACE("open(%s)\n", name);

	if (NULL == dev)
		return B_ERROR;
	memset(dev, 0, sizeof(dev_open));	
	*cookie = (void *)dev;

	// search cookie
	acquire_sem(gLock);
	for (list = gDeviceList; NULL != list; list = list->next) {
		int match = 1;
		if (0 == strcmp(name, list->name_bbu))
			dev->type = YUREX_DEVICE_TYPE_BBU;
		else if (0 == strcmp(name, list->name_anime))
			dev->type = YUREX_DEVICE_TYPE_ANIME;
		else
			match = 0;
		TRACE(" %s ... %s\n",
			name,
			(0 == match)? "unmatch": "match");
		if (1 == match) {
			dev->dev = list;
			break;
		}
	}
	release_sem(gLock);

	if (NULL == dev->dev) {
		TRACE_ALWAYS("cookie not found\n");
		return B_ERROR;
	}

	return B_OK;
}

status_t
device_close
(void *cookie)
{
	TRACE("close()\n");
	return B_ERROR;
}

status_t
device_free
(void *cookie)
{
	TRACE("free()\n");
	
	if (NULL != cookie)
		free(cookie);

	return B_ERROR;
}

status_t
device_read
(void *cookie, off_t position, void *buffer, size_t *length)
{
	size_t len;
	dev_open *dev = (dev_open *)cookie;
	TRACE("read(%d, %d)\n", position, *length);
	if (0 == position) {
		acquire_sem(dev->dev->sem);
		if (YUREX_DEVICE_TYPE_BBU == dev->type)
			dev->buf_len = snprintf(dev->buf, 16, "%ld\n", dev->dev->bbu);
		else
			dev->buf_len = snprintf(dev->buf, 16, "%d\n", dev->dev->anime);
		release_sem(dev->dev->sem);
	}
	
	len = dev->buf_len - position;
	if (len > *length)
		len = *length;
	memcpy(buffer, &dev->buf[position], len);
	*length = len;

	return B_OK;
}

status_t
device_write
(void *cookie, off_t position, const void *buffer, size_t *length)
{
	dev_open *dev = (dev_open *)cookie;
	TRACE("write(%d)\n", *length);
	if (0 == *length)
		return B_OK;
	
	if (YUREX_DEVICE_TYPE_ANIME == dev->type) {
		if ('0' == *(char *)buffer) {
			TRACE(" animation off\n");
			acquire_sem(dev->dev->sem);
			dev->dev->anime = 0;
			release_sem(dev->dev->sem);
			yurex_set_mode(dev->dev, 0xff);
		} else {
			TRACE(" animation on\n");
			acquire_sem(dev->dev->sem);
			dev->dev->anime = 1;
			release_sem(dev->dev->sem);
			yurex_set_mode(dev->dev, 0x00);
		}
	} else {
		uint64 bbu = 0;
		const char *bbu_str = (const char *)buffer;
		while (('0' <= *bbu_str) && (*bbu_str <= '9')) {
			bbu *= 10;
			bbu += *bbu_str++ - '0';
		}
		TRACE(" write bbu: %ld\n", bbu);
		yurex_write_bbu(dev->dev, bbu);
	}
	return B_OK;
}
