#include <asm-generic/errno-base.h>
#include <linux/fs.h> // allows us to add a new driver
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#define TBOTS_SPI_MAJOR 60
#define TBOTS_MOTOR_BUS_NUM 0
#define TBOTS_MULTIPLE_TRANSFER_OFFSET 5
#define TBOTS_N_DEVICES 5
#define TBOTS_MAX_SPEED_HZ 1000000
#define TBOTS_BUF_SIZE 4096

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

struct spi_board_info tbots_device_info[TBOTS_N_DEVICES] = 
{
    {
        .modalias       = "front_left",
        .max_speed_hz   = TBOTS_MAX_SPEED_HZ,
        .bus_num        = TBOTS_MOTOR_BUS_NUM,
        .chip_select    = 0,
        .mode           = SPI_MODE_3
    },
    {
        .modalias       = "back_left",
        .max_speed_hz   = TBOTS_MAX_SPEED_HZ,
        .bus_num        = TBOTS_MOTOR_BUS_NUM,
        .chip_select    = 1,
        .mode           = SPI_MODE_3
    },
    {
        .modalias       = "back_right",
        .max_speed_hz   = TBOTS_MAX_SPEED_HZ,
        .bus_num        = TBOTS_MOTOR_BUS_NUM,
        .chip_select    = 2,
        .mode           = SPI_MODE_3
    },
    {
        .modalias       = "front_right",
        .max_speed_hz   = TBOTS_MAX_SPEED_HZ,
        .bus_num        = TBOTS_MOTOR_BUS_NUM,
        .chip_select    = 3,
        .mode           = SPI_MODE_3
    },
    {
        .modalias       = "dribbler",
        .max_speed_hz   = TBOTS_MAX_SPEED_HZ,
        .bus_num        = TBOTS_MOTOR_BUS_NUM,
        .chip_select    = 4,
        .mode           = SPI_MODE_3
    },
};

struct spi_device* tbots_devices[TBOTS_N_DEVICES];

struct tbots_spi_data
{
	spinlock_t		spi_lock;

	/* TX/RX buffers are NULL unless this device is open (users > 0) */
	struct mutex		buf_lock;
	unsigned		users;
	u8			*tx_buffer;
	u8			*rx_buffer;
	u32			speed_hz;
    bool        sync;
};

static struct class *tbots_spi_class;

static dev_t dev_tbots_spi;
DEFINE_MUTEX(buf_lock);

static int tbots_spi_open(struct inode *inode, struct file *filp)
{
    struct tbots_spi_data *dev;
    int status;

    printk(KERN_ALERT "TBOTS SPI OPEN CALLED\n");

    /* Allocate driver data */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
    {
		return -ENOMEM;
    }

    if (!dev->tx_buffer)
    {
	    dev->tx_buffer = kmalloc(TBOTS_BUF_SIZE, GFP_KERNEL);
        if (!dev->tx_buffer)
        {
            // out of memory
            printk(KERN_DEBUG "[TBOTS SPI] can't allocate tx buffer");
			status = -ENOMEM;
            return status;
		}
	}

	if (!dev->rx_buffer)
    {
		dev->rx_buffer = kmalloc(TBOTS_BUF_SIZE, GFP_KERNEL);
		if (!dev->rx_buffer)
        {
            // out of memory
            printk(KERN_DEBUG "[TBOTS SPI] can't allocate rx buffer");
			status = -ENOMEM;
			goto err_alloc_rx_buf;
		}
	}

    spin_lock_init(&dev->spi_lock);
    mutex_init(&dev->buf_lock);

    dev->users++;
    filp->private_data = dev;
    nonseekable_open(inode, filp);

    printk(KERN_ALERT "TBOTS SPI OPEN FINISHED\n");

    return 0;

err_alloc_rx_buf:
	kfree(dev->tx_buffer);
	dev->tx_buffer = NULL;

    return status;
}

static int tbots_spi_release(struct inode *inode, struct file *filp)
{
    struct tbots_spi_data *tbots_data;
    int dofree;

    // deallocate everything in filp->private_data
    tbots_data = filp->private_data;
    filp->private_data = NULL;

    // decrement user count
	tbots_data->users--;

    // last close? shut down device
    if (!tbots_data->users)
    {
		kfree(tbots_data->tx_buffer);
		tbots_data->tx_buffer = NULL;

		kfree(tbots_data->rx_buffer);
		tbots_data->rx_buffer = NULL;

		if (dofree)
        {
			kfree(tbots_data);
        }
		else
        {
			tbots_data->speed_hz = TBOTS_MAX_SPEED_HZ;
        }
	}

    return 0;
}

static ssize_t
tbots_spi_sync(struct tbots_spi_data *spidata, struct spi_message *message, int cs)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;
	struct spi_device *spi;

	spin_lock_irq(&spidata->spi_lock);
	spi = tbots_devices[cs];
	spin_unlock_irq(&spidata->spi_lock);

	if (spi == NULL)
    {
		status = -ESHUTDOWN;
    }
	else
    {
		status = spi_sync(spi, message);
    }

    if (status == 0)
    {
		status = message->actual_length;
    }

	return status;
}

static inline ssize_t
tbots_sync_read(struct tbots_spi_data *spi_data, size_t len)
{
    ssize_t total_read = 0;

    int i;
    // don't read anything from the dribbler
    for (i = 0; i < TBOTS_N_DEVICES; ++i)
    {
        struct spi_message m;

        struct spi_transfer write_tx =
        {
            .tx_buf = spi_data->tx_buffer,
            .len = 1,
            .speed_hz = spi_data->speed_hz,
        };

        struct spi_transfer read_tx =
        {
            .rx_buf		= spi_data->rx_buffer+(4*i),
            .len        = 4,
            .speed_hz   = spi_data->speed_hz,
        };


        spi_message_init(&m);
        spi_message_add_tail(&write_tx, &m);
        tbots_spi_sync(spi_data, &m, i);

        spi_message_init(&m);
        spi_message_add_tail(&read_tx, &m);
        total_read += tbots_spi_sync(spi_data, &m, i);
    }

    return total_read;
}

static inline ssize_t
tbots_sync_write(struct tbots_spi_data *tbots_data, size_t len)
{
    ssize_t total_writ = 0;
    int i;
    for (i = 0; i < TBOTS_N_DEVICES; ++i)
    {
        struct spi_transfer	t = {
                .tx_buf		= tbots_data->tx_buffer+i*5,
                .len		= 5,
                .speed_hz	= tbots_data->speed_hz,
            };
        struct spi_message	m;

        spi_message_init(&m);
        spi_message_add_tail(&t, &m);
	    total_writ += tbots_spi_sync(tbots_data, &m, i);
    }

    return total_writ;
}

/* Write-only message with current device setup */
static ssize_t
tbots_spi_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct tbots_spi_data	*tbots_data;
	ssize_t			status = 0;
	unsigned long		missing;

    printk(KERN_ALERT "TBOTS SPI WRITE CALLED\n");

    // return error if it's bigger than the buffer we allocated
	if (count > TBOTS_BUF_SIZE)
    {
		return -EMSGSIZE;
    }

	tbots_data = (struct tbots_spi_data *) filp->private_data;

    printk(KERN_ALERT "mutex lock status: %i\n", mutex_is_locked(&buf_lock));
	mutex_lock(&buf_lock);
	missing = copy_from_user(tbots_data->tx_buffer, buf, count);
    printk(KERN_ALERT "data copied from user buffer: %li\n", count);
	if (missing == 0)
    {
        status = tbots_sync_write(tbots_data, count);
    }
	else
    {
		status = -EFAULT;
    }
	mutex_unlock(&buf_lock);

	return status;
}

static ssize_t tbots_spi_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    struct tbots_spi_data *tbots_spi;
    ssize_t status = 0;
    int i;

    printk(KERN_ALERT "TBOTS SPI READ CALLED\n");

    if (count > TBOTS_BUF_SIZE)
    {
		return -EMSGSIZE;
    }

    tbots_spi = filp->private_data;

    mutex_lock(&tbots_spi->buf_lock);
    tbots_spi->tx_buffer[0] = 60;
    if (status != 0)
    {
       pr_debug("couldn't copy from transaction buffer to write address before a read buffer"); 
       status = -EFAULT; 
       mutex_unlock(&tbots_spi->buf_lock);
       return status;
    }
    status = tbots_sync_read(tbots_spi, count);
    printk(KERN_ALERT "read: ");
    for (i = 0; i < 25; ++i)
    {
        printk(KERN_ALERT "%x", tbots_spi->rx_buffer[i]);
    }
    printk(KERN_ALERT "\n");
    if (status > 0)
    {
		unsigned long	missing;

		missing = copy_to_user(buf, tbots_spi->rx_buffer, status);
		if (missing == status)
        {
			status = -EFAULT;
        }
		else
        {
			status = status - missing;
        }
	}
	mutex_unlock(&tbots_spi->buf_lock);

	return status;
}

static void __exit tbots_spi_exit(void)
{
    int i;
    for (i = 0; i < TBOTS_N_DEVICES; ++i)
    {
        if (tbots_devices[i])
        {
            spi_unregister_device(tbots_devices[i]);
        }
    }
    class_destroy(tbots_spi_class);
    unregister_chrdev(TBOTS_SPI_MAJOR, "motors");
}

static const struct file_operations tbots_spi_fops =
{
	.owner =	THIS_MODULE,

    .write = tbots_spi_write,
    .read  = tbots_spi_read,
    .open = tbots_spi_open,
    .release = tbots_spi_release,
};

static int __init tbots_spi_init(void)
{
    int ret;
    int counter;
    struct spi_master *master;

    printk(KERN_ALERT "TBOTS SPI DRIVER MODULE LOADED\n");

    ret = register_chrdev(TBOTS_SPI_MAJOR, "motors", &tbots_spi_fops);
    if (ret < 0)
    {
        return ret;
    }

    tbots_spi_class = class_create(THIS_MODULE, "motors");
    if (IS_ERR(tbots_spi_class))
    {
        unregister_chrdev(TBOTS_SPI_MAJOR, "motors");
        return PTR_ERR(tbots_spi_class);
    }

    master = spi_busnum_to_master(TBOTS_MOTOR_BUS_NUM);
    if (master == NULL)
    {
        pr_err("SPI Master not found");
        return -ENODEV;
    }

    for (counter = 0; counter < TBOTS_N_DEVICES; ++counter)
    {
        tbots_devices[counter] = spi_new_device(master, &tbots_device_info[counter]);
        if (tbots_devices[counter] == NULL)
        {
            pr_err("Failed to create a SPI device\n");
            return -ENODEV;
        }
    }

    for (counter = 0; counter < TBOTS_N_DEVICES; ++counter)
    {
        ret = spi_setup(tbots_devices[counter]);
        tbots_devices[counter]->bits_per_word = 8;
        if (ret)
        {
            goto error_handle;
            pr_err("Failed to setup SPI device\n");
            spi_unregister_device(tbots_devices[counter]);
            return -ENODEV;
        }
    }

    dev_tbots_spi = MKDEV(TBOTS_SPI_MAJOR, 0);

    printk(KERN_ALERT "TBOTS SPI DRIVER MODULE LOADED SUCCESSFULLY\n");
    return 0;

error_handle:
    for (counter = 0; counter < TBOTS_N_DEVICES; ++counter)
    {
        if (tbots_devices[counter])
        {
            spi_unregister_device(tbots_devices[counter]);
        }
    }

    return -ENODEV;
}

MODULE_LICENSE("GPL");
module_init(tbots_spi_init);
module_exit(tbots_spi_exit);
