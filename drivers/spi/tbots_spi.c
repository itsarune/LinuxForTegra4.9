#include <linux/fs.h> // allows us to add a new driver

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

//interface:
// goal: write to all motors at once
// goal: read all motor velocities at once
// goal: read all faults to all motors at once
// how: --struct ioc messages contains (chip select id? motor name?)-- won't work i don't think
// how: one file descriptor connects to all four motors, then we can send a ioc message with the commanded motor velcoities for four motors

struct spi_device tbots_devices[5] = 
{

};

static struct spi_driver spidev_spi_driver = {
	.driver = {
		.name =		"tbots_spi",
        .owner = THIS_MODULE,
        .pm = &tbots_spi_pm_ops, // TODO: what's this for, needed?
        // TODO: unclear if the following two lines are needed
		//.of_match_table = of_match_ptr(spidev_dt_ids),
		//.acpi_match_table = ACPI_PTR(spidev_acpi_ids),
	},
	.probe =	tbots_spi_probe, // TODO: implement
	.remove =	tbots_spi_remove, // TODO: implement

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */


static const struct file_operations tbots_spi_fops =
{
	.owner =	THIS_MODULE,

    .write = // TODO: impelment
    .read  = // TODO: implement 
    .unlocked_ioctl = // TODO:implement
    .compat_ioctl = // TODO: implement
    .open = tbots_spi_open,
    .release = tbots_spi_release,
    .llseek = // TODO: implement (doesn't actually need to do anything) 
}

struct tbots_spi_data {
	dev_t			devt;
	spinlock_t		spi_lock;
    // 5 motors here spi_device info globals 
	//struct spi_device	*spi;
	struct list_head	device_entry;

	/* TX/RX buffers are NULL unless this device is open (users > 0) */
	struct mutex		buf_lock;
	unsigned		users;
	u8			*tx_buffer;
	u8			*rx_buffer;
	u32			speed_hz;
};

static int tbots_spi_open(struct inode *inode, struct file *filp)
{
    struct tbots_spi_data *dev;

    if (!dev->tx_buffer) {
	    dev->tx_buffer = kmalloc(bufsiz, GFP_KERNEL);
        if (!dev->tx_buffer) {
            // out of memory
			dev_dbg(&dev->spi->dev, "open/ENOMEM\n");
			status = -ENOMEM;
            return status;
		}
	}

	if (!dev->rx_buffer) {
		dev->rx_buffer = kmalloc(bufsiz, GFP_KERNEL);
		if (!dev->rx_buffer) {
            // out of memory
			dev_dbg(&dev->spi->dev, "open/ENOMEM\n");
			status = -ENOMEM;
			goto err_alloc_rx_buf;
		}
	}

    dev->users++;
    filp->private_data = dev;
    nonseekable_open(inode, filp);

    return 0;

err_alloc_rx_buf:
	kfree(dev->tx_buffer);
	dev->tx_buffer = NULL;
}

static int tbots_spi_release(struct inode *inode, struct file *flip)
{
    struct spidev_data *dev;
    int dofree;

    // deallocate everything in filp->private_data
    dev = filp->private_data;
    filp->private_data = NULL;
    
    spin_lock_irq(*dev->spi_lock);
    // wait until any in-progress transfers are done
    dofree = (dev->spi == NULL);
    spin_unlock_irq(*dev->spi_lock);

    // decrement user count
	dev->users--;

    // last close? shut down device
    if (!dev->users)
    {
		kfree(dev->tx_buffer);
		dev->tx_buffer = NULL;

		kfree(dev->rx_buffer);
		dev->rx_buffer = NULL;

		if (dofree)
			kfree(dev);
		else
			dev->speed_hz = dev->spi->max_speed_hz;
	}

    return 0;
}

static ssize_t spidev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    ssize_t status = 0;

    if (count > bufsiz)
    {
		return -EMSGSIZE;
    }
    // TODO: finish
}
