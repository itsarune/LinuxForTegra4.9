#include <linux/fs.h> // allows us to add a new driver

#include <linux/spi/spi.h>
#include <linux/spi/tbots_spi.h>

#define TBOTS_SPI_MAJOR 60
#define TBOTS_MOTOR_BUS_NUM 0
#define TBOTS_MULTIPLE_TRANSFER_OFFSET 5

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

struct spi_board_info tbots_device_info[5] = 
{
    {
        .modalias       = "front_left",
        .max_speed_hz   = 1000000,
        .bus_num        = TBOTS_MOTOR_BUS_NUM,
        .chip_select    = 0,
        .mode           = SPI_MODE_3
    },
    {
        .modalias       = "back_left",
        .max_speed_hz   = 1000000,
        .bus_num        = TBOTS_MOTOR_BUS_NUM,
        .chip_select    = 1,
        .mode           = SPI_MODE_3
    },
    {
        .modalias       = "back_right",
        .max_speed_hz   = 1000000,
        .bus_num        = TBOTS_MOTOR_BUS_NUM,
        .chip_select    = 2,
        .mode           = SPI_MODE_3
    },
    {
        .modalias       = "front_right",
        .max_speed_hz   = 1000000,
        .bus_num        = TBOTS_MOTOR_BUS_NUM,
        .chip_select    = 3,
        .mode           = SPI_MODE_3
    },
    {
        .modalias       = "dribbler",
        .max_speed_hz   = 1000000,
        .bus_num        = TBOTS_MOTOR_BUS_NUM,
        .chip_select    = 4,
        .mode           = SPI_MODE_3
    },
};

struct spi_device tbots_devices[5];

// TODO: do this in init()
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
    .open = tbots_spi_open,
    .release = tbots_spi_release,
    .llseek = // TODO: implement (doesn't actually need to do anything) 
}

struct tbots_spi_data {
	dev_t			devt;
	spinlock_t		spi_lock;
    // 5 motors here spi_device info globals 
	//struct spi_device	*spi;
    char            *requested_gpios;
    char            num_transfers;
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

static ssize_t
tbots_spi_sync(struct tbots_spi_data *spidata, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;
	struct spi_device *spi;

	spin_lock_irq(&spidata->spi_lock);
    // TODO: get spi_device, set up spi device in init
	spi = spidata->spi;
	spin_unlock_irq(&spidata->spi_lock);

	if (spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_sync(spi, message);

	if (status == 0)
		status = message->actual_length;

	return status;
}


static inline ssize_t
tbots_sync_read(struct tbots_spi_data *spi_data, size_t len)
{
    ssize_t total_read = 0;

    for (int i = 0; i < spi_data->num_transfers; ++i)
    {
        struct spi_transfer	t = {
                .rx_buf		= spi_data->rx_buffer+(TBOTS_MULTIPLE_TRANSFER_OFFSET*i),
                .len		= len,
                .speed_hz	= spi_data->speed_hz,
            };
        struct spi_message	m;

        spi_message_init(&m);
        spi_message_add_tail(&t, &m);
        total_read += tbots_spi_sync(spi_data, &m);
    }
}


static ssize_t tbots_spi_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    struct tbots_spi_data *tbots_spi;
    ssize_t status = 0;

    if (count > bufsiz)
    {
		return -EMSGSIZE;
    }

    // TODO: finish
    tbots_spi = filp->private_data;

    mutex_lock(&spidev->buf_lock);
    status = tbots_spi_sync_read(tbots_spi, count);
    if (status > 0) {
		unsigned long	missing;

		missing = copy_to_user(buf, spidev->rx_buffer, status);
		if (missing == status)
			status = -EFAULT;
		else
			status = status - missing;
	}
	mutex_unlock(&spidev->buf_lock);

	return status;
}
