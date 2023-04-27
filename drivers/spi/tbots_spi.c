#include <linux/spi/tbots_spi.h>

#define TBOTS_SPI_MAJOR 60

// register_chrdev adds a new driver by registering a major number to it
// MAJOR NUMBER identifies the driver associated with it (so all major numbers with this major number will associate with this driver)
// we register the file operations structure, whenever we perform an action on the character device file of an associated major number, then we invoke the relevant file_operations structure
// ARUN: make sure this file_operations struct is global static!
//
// you need to make a device node on the device tree /dev/front_right /dev/front_left /dev/back_left /dev/back_right
//
// AKHIL: how do i know what major number is associated with the new spi devices?
// AKHIL: dynamic magic number?? do I need a rc.local script?

static const struct file_operations tbots_spi_fops = {
	.owner =	THIS_MODULE,

    .write = // TODO: impelment
    .read  = // TODO: implement 
    .unlocked_ioctl = // TODO:implement
    .compat_ioctl = // TODO: implement
    .open = // TODO: spidev impelment
    .release = // TODO: implement 
    .llseek = // TODO: implement (doesn't actually need to do anything) 
}
