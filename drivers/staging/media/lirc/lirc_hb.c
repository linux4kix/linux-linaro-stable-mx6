/*
 * lirc_hb.c
 *
 * lirc_hb - Device driver that records pulse- and pause-lengths
 *	      (space-lengths) (just like the lirc_serial driver does)
 *	      between GPIO interrupt events on the Hummingboard.
 *	      Lots of code has been taken from the lirc_serial module,
 *	      so I would like say thanks to the authors.
 *
 * Copyright (C) 2012 - 2014 CurlyMo <development@xbian.org>,
 *                    Aron Robert Szabo <aron@reon.hu>,
 *		       Michael Bishop <cleverca22@gmail.com>
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/spinlock.h>
#include <media/lirc.h>
#include <media/lirc_dev.h>
#include <linux/gpio.h>

#define LIRC_DRIVER_NAME "lirc_hb"
#define RBUF_LEN 256
#define LIRC_TRANSMITTER_LATENCY 256

#ifndef MAX_UDELAY_MS
#define MAX_UDELAY_US 5000
#else
#define MAX_UDELAY_US (MAX_UDELAY_MS*1000)
#endif

#define dprintk(fmt, args...)					\
	do {							\
		if (debug)					\
			printk(KERN_DEBUG LIRC_DRIVER_NAME ": "	\
			       fmt, ## args);			\
	} while (0)

/* module parameters */

/* set the default GPIO input pin */
static int gpio_in_pin = 72;
/* set the default GPIO output pin */
static int gpio_out_pin = 73;
/* enable debugging messages */
static int debug;
/* -1 = auto, 0 = active high, 1 = active low */
static int sense = -1;
/* use softcarrier by default */
static int softcarrier = 1;

struct irq_chip *irqchip;
struct irq_data *irqdata;

/* forward declarations */
static long send_pulse(unsigned long length);
static void send_space(long length);
static void lirc_hb_exit(void);

int valid_gpio_pins[] = { 1, 73, 72, 71, 70, 194, 195, 67 };

static struct platform_device *lirc_hb_dev;
static struct timeval lasttv = { 0, 0 };
static struct lirc_buffer rbuf;
static spinlock_t lock;

/* initialized/set in init_timing_params() */
static unsigned int freq = 38000;
static unsigned int duty_cycle = 50;
static unsigned long period;
static unsigned long pulse_width;
static unsigned long space_width;

static void safe_udelay(unsigned long usecs)
{
	while (usecs > MAX_UDELAY_US) {
		udelay(MAX_UDELAY_US);
		usecs -= MAX_UDELAY_US;
	}
	udelay(usecs);
}

static int init_timing_params(unsigned int new_duty_cycle,
	unsigned int new_freq)
{
	/*
	 * period, pulse/space width are kept with 8 binary places -
	 * IE multiplied by 256.
	 */
	if (256 * 1000000L / new_freq * new_duty_cycle / 100 <=
	    LIRC_TRANSMITTER_LATENCY)
		return -EINVAL;
	if (256 * 1000000L / new_freq * (100 - new_duty_cycle) / 100 <=
	    LIRC_TRANSMITTER_LATENCY)
		return -EINVAL;
	duty_cycle = new_duty_cycle;
	freq = new_freq;
	period = 256 * 1000000L / freq;
	pulse_width = period * duty_cycle / 100;
	space_width = period - pulse_width;
	dprintk("in init_timing_params, freq=%d pulse=%ld, "
		"space=%ld\n", freq, pulse_width, space_width);
	return 0;
}

static long send_pulse_softcarrier(unsigned long length)
{
	int flag;
	unsigned long actual, target, d;

	length <<= 8;

	actual = 0; target = 0; flag = 0;
	while (actual < length) {
		if (flag) {
			gpio_set_value(gpio_out_pin, 0);
			target += space_width;
		} else {
			gpio_set_value(gpio_out_pin, 1);
			target += pulse_width;
		}
		d = (target - actual -
		     LIRC_TRANSMITTER_LATENCY + 128) >> 8;
		/*
		 * Note - we've checked in ioctl that the pulse/space
		 * widths are big enough so that d is > 0
		 */
		udelay(d);
		actual += (d << 8) + LIRC_TRANSMITTER_LATENCY;
		flag = !flag;
	}
	return (actual-length) >> 8;
}

static long send_pulse(unsigned long length)
{
	if (length <= 0)
		return 0;

	if (softcarrier) {
		return send_pulse_softcarrier(length);
	} else {
		gpio_set_value(gpio_out_pin, 1);
		safe_udelay(length);
		return 0;
	}
}

static void send_space(long length)
{
	gpio_set_value(gpio_out_pin, 0);
	if (length <= 0)
		return;
	safe_udelay(length);
}

static void rbwrite(int l)
{
	if (lirc_buffer_full(&rbuf)) {
		/* no new signals will be accepted */
		dprintk("Buffer overrun\n");
		return;
	}
	lirc_buffer_write(&rbuf, (void *)&l);
}

static void frbwrite(int l)
{
	/* simple noise filter */
	static int pulse, space;
	static unsigned int ptr;

	if (ptr > 0 && (l & PULSE_BIT)) {
		pulse += l & PULSE_MASK;
		if (pulse > 250) {
			rbwrite(space);
			rbwrite(pulse | PULSE_BIT);
			ptr = 0;
			pulse = 0;
		}
		return;
	}
	if (!(l & PULSE_BIT)) {
		if (ptr == 0) {
			if (l > 20000) {
				space = l;
				ptr++;
				return;
			}
		} else {
			if (l > 20000) {
				space += pulse;
				if (space > PULSE_MASK)
					space = PULSE_MASK;
				space += l;
				if (space > PULSE_MASK)
					space = PULSE_MASK;
				pulse = 0;
				return;
			}
			rbwrite(space);
			rbwrite(pulse | PULSE_BIT);
			ptr = 0;
			pulse = 0;
		}
	}
	rbwrite(l);
}

static irqreturn_t irq_handler(int i, void *blah, struct pt_regs *regs)
{
	struct timeval tv;
	long deltv;
	int data;
	int signal;

	/* use the GPIO signal level */
	signal = gpio_get_value(gpio_in_pin);

	/* unmask the irq */
	irqchip->irq_unmask(irqdata);

	if (sense != -1) {
		/* The HB GPIO input acts like it is an analogue input.
		   Therefor a high signal is 256 and a low signal is 1.
		   For Lirc to properly interpret the spaces and pulses,
		   we need to transform these to ones and zeros. To be
		   on the safe side, every signal higher then 128 will
		   be interpreted as a high and vice versa. */
		if (signal > 128) {
			signal = 1;
		} else {
			signal = 0;
		}
		/* get current time */
		do_gettimeofday(&tv);

		/* calc time since last interrupt in microseconds */
		deltv = tv.tv_sec-lasttv.tv_sec;
		if (tv.tv_sec < lasttv.tv_sec ||
		    (tv.tv_sec == lasttv.tv_sec &&
		     tv.tv_usec < lasttv.tv_usec)) {
			printk(KERN_WARNING LIRC_DRIVER_NAME
			       ": AIEEEE: your clock just jumped backwards\n");
			printk(KERN_WARNING LIRC_DRIVER_NAME
			       ": %d %d %lx %lx %lx %lx\n", signal, sense,
			       tv.tv_sec, lasttv.tv_sec,
			       tv.tv_usec, lasttv.tv_usec);
			data = PULSE_MASK;
		} else if (deltv > 15) {
			data = PULSE_MASK; /* really long time */
			if (!(signal^sense)) {
				/* sanity check */
				printk(KERN_WARNING LIRC_DRIVER_NAME
				       ": AIEEEE: %d %d %lx %lx %lx %lx\n",
				       signal, sense, tv.tv_sec, lasttv.tv_sec,
				       tv.tv_usec, lasttv.tv_usec);
				/*
				 * detecting pulse while this
				 * MUST be a space!
				 */
				sense = sense ? 0 : 1;
			}
		} else {
			data = (int) (deltv*1000000 +
				      (tv.tv_usec - lasttv.tv_usec));
		}
		frbwrite(signal^sense ? data : (data|PULSE_BIT));
		lasttv = tv;
		wake_up_interruptible(&rbuf.wait_poll);
	}

	return IRQ_HANDLED;
}

static int init_port(void)
{
	int i, nlow, nhigh, ret, irq;

	if (gpio_request(gpio_out_pin, LIRC_DRIVER_NAME " ir/out")) {
		printk(KERN_ALERT LIRC_DRIVER_NAME
		       ": cant claim gpio pin %d\n", gpio_out_pin);
		ret = -ENODEV;
		goto exit_init_port;
	}

	if (gpio_request(gpio_in_pin, LIRC_DRIVER_NAME " ir/in")) {
		printk(KERN_ALERT LIRC_DRIVER_NAME
		       ": cant claim gpio pin %d\n", gpio_in_pin);
		ret = -ENODEV;
		goto exit_gpio_free_out_pin;
	}

	gpio_direction_input(gpio_in_pin);
	gpio_direction_output(gpio_out_pin, 1);
	gpio_set_value(gpio_out_pin, 0);

	irq = gpio_to_irq(gpio_in_pin);
	dprintk("to_irq %d\n", irq);
	irqdata = irq_get_irq_data(irq);

	if (irqdata && irqdata->chip) {
		irqchip = irqdata->chip;
	} else {
		ret = -ENODEV;
		goto exit_gpio_free_in_pin;
	}

	/* if pin is high, then this must be an active low receiver. */
	if (sense == -1) {
		/* wait 1/2 sec for the power supply */
		msleep(500);

		/*
		 * probe 9 times every 0.04s, collect "votes" for
		 * active high/low
		 */
		nlow = 0;
		nhigh = 0;
		for (i = 0; i < 9; i++) {
			if (gpio_get_value(gpio_in_pin))
				nlow++;
			else
				nhigh++;
			msleep(40);
		}
		sense = (nlow >= nhigh ? 1 : 0);
		printk(KERN_INFO LIRC_DRIVER_NAME
		       ": auto-detected active %s receiver on GPIO pin %d\n",
		       sense ? "low" : "high", gpio_in_pin);
	} else {
		printk(KERN_INFO LIRC_DRIVER_NAME
		       ": manually using active %s receiver on GPIO pin %d\n",
		       sense ? "low" : "high", gpio_in_pin);
	}

	return 0;

	exit_gpio_free_in_pin:
	gpio_free(gpio_in_pin);

	exit_gpio_free_out_pin:
	gpio_free(gpio_out_pin);

	exit_init_port:
	return ret;
}

// called when the character device is opened
static int set_use_inc(void *data)
{
	int result;
	unsigned long flags;

	/* initialize timestamp */
	do_gettimeofday(&lasttv);

	result = request_irq(gpio_to_irq(gpio_in_pin),
			     (irq_handler_t) irq_handler, 0,
			     LIRC_DRIVER_NAME, (void*) 0);

	switch (result) {
	case -EBUSY:
		printk(KERN_ERR LIRC_DRIVER_NAME
		       ": IRQ %d is busy\n",
		       gpio_to_irq(gpio_in_pin));
		return -EBUSY;
	case -EINVAL:
		printk(KERN_ERR LIRC_DRIVER_NAME
		       ": Bad irq number or handler\n");
		return -EINVAL;
	default:
		dprintk("Interrupt %d obtained\n",
			gpio_to_irq(gpio_in_pin));
		break;
	};

	/* initialize pulse/space widths */
	init_timing_params(duty_cycle, freq);

	spin_lock_irqsave(&lock, flags);

	/* GPIO Pin Falling/Rising Edge Detect Enable */
	irqchip->irq_set_type(irqdata,
			      IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING);

	/* unmask the irq */
	irqchip->irq_unmask(irqdata);

	spin_unlock_irqrestore(&lock, flags);

	return 0;
}

static void set_use_dec(void *data)
{
	unsigned long flags;

	spin_lock_irqsave(&lock, flags);

	/* GPIO Pin Falling/Rising Edge Detect Disable */
	irqchip->irq_set_type(irqdata, 0);
	irqchip->irq_mask(irqdata);

	spin_unlock_irqrestore(&lock, flags);

	free_irq(gpio_to_irq(gpio_in_pin), (void *) 0);

	dprintk(KERN_INFO LIRC_DRIVER_NAME
		": freed IRQ %d\n", gpio_to_irq(gpio_in_pin));
}

static ssize_t lirc_write(struct file *file, const char *buf,
	size_t n, loff_t *ppos)
{
	int i, count;
	unsigned long flags;
	long delta = 0;
	int *wbuf;

	count = n / sizeof(int);
	if (n % sizeof(int) || count % 2 == 0)
		return -EINVAL;
	wbuf = memdup_user(buf, n);
	if (IS_ERR(wbuf))
		return PTR_ERR(wbuf);
	spin_lock_irqsave(&lock, flags);

	for (i = 0; i < count; i++) {
		if (i%2)
			send_space(wbuf[i] - delta);
		else
			delta = send_pulse(wbuf[i]);
	}
	gpio_set_value(gpio_out_pin, 0);

	spin_unlock_irqrestore(&lock, flags);
	kfree(wbuf);
	return n;
}

static long lirc_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	int result;
	__u32 value;

	switch (cmd) {
	case LIRC_GET_SEND_MODE:
		return -ENOIOCTLCMD;
		break;

	case LIRC_SET_SEND_MODE:
		result = get_user(value, (__u32 *) arg);
		if (result)
			return result;
		/* only LIRC_MODE_PULSE supported */
		if (value != LIRC_MODE_PULSE)
			return -ENOSYS;
		break;

	case LIRC_GET_LENGTH:
		return -ENOSYS;
		break;

	case LIRC_SET_SEND_DUTY_CYCLE:
		dprintk("SET_SEND_DUTY_CYCLE\n");
		result = get_user(value, (__u32 *) arg);
		if (result)
			return result;
		if (value <= 0 || value > 100)
			return -EINVAL;
		return init_timing_params(value, freq);
		break;

	case LIRC_SET_SEND_CARRIER:
		dprintk("SET_SEND_CARRIER\n");
		result = get_user(value, (__u32 *) arg);
		if (result)
			return result;
		if (value > 500000 || value < 20000)
			return -EINVAL;
		return init_timing_params(duty_cycle, value);
		break;

	default:
		return lirc_dev_fop_ioctl(filep, cmd, arg);
	}
	return 0;
}

static const struct file_operations lirc_fops = {
	.owner		= THIS_MODULE,
	.write		= lirc_write,
	.unlocked_ioctl	= lirc_ioctl,
	.read		= lirc_dev_fop_read,
	.poll		= lirc_dev_fop_poll,
	.open		= lirc_dev_fop_open,
	.release	= lirc_dev_fop_close,
	.llseek		= no_llseek,
};

static struct lirc_driver driver = {
	.name		= LIRC_DRIVER_NAME,
	.minor		= -1,
	.code_length	= 1,
	.sample_rate	= 0,
	.data		= NULL,
	.add_to_buf	= NULL,
	.rbuf		= &rbuf,
	.set_use_inc	= set_use_inc,
	.set_use_dec	= set_use_dec,
	.fops		= &lirc_fops,
	.dev		= NULL,
	.owner		= THIS_MODULE,
};

static struct platform_driver lirc_hb_driver = {
	.driver = {
		.name   = LIRC_DRIVER_NAME,
		.owner  = THIS_MODULE,
	},
};

static int __init lirc_hb_init(void)
{
	int result;

	/* Init read buffer. */
	result = lirc_buffer_init(&rbuf, sizeof(int), RBUF_LEN);
	if (result < 0)
		return -ENOMEM;

	result = platform_driver_register(&lirc_hb_driver);
	if (result) {
		printk(KERN_ERR LIRC_DRIVER_NAME
		       ": lirc register returned %d\n", result);
		goto exit_buffer_free;
	}

	lirc_hb_dev = platform_device_alloc(LIRC_DRIVER_NAME, 0);
	if (!lirc_hb_dev) {
		result = -ENOMEM;
		goto exit_driver_unregister;
	}

	result = platform_device_add(lirc_hb_dev);
	if (result)
		goto exit_device_put;

	return 0;

	exit_device_put:
	platform_device_put(lirc_hb_dev);

	exit_driver_unregister:
	platform_driver_unregister(&lirc_hb_driver);

	exit_buffer_free:
	lirc_buffer_free(&rbuf);

	return result;
}

static void lirc_hb_exit(void)
{
	gpio_free(gpio_out_pin);
	gpio_free(gpio_in_pin);

	platform_device_unregister(lirc_hb_dev);
	platform_driver_unregister(&lirc_hb_driver);
	lirc_buffer_free(&rbuf);
}

static int __init lirc_hb_init_module(void)
{
	int result, i;

	result = lirc_hb_init();
	if (result)
		return result;

	/* check if the module received valid gpio pin numbers */
	result = 0;
	if (gpio_in_pin != gpio_out_pin) {
		for(i = 0; (i < ARRAY_SIZE(valid_gpio_pins)) && (result != 2); i++) {
			if (gpio_in_pin == valid_gpio_pins[i] ||
			   gpio_out_pin == valid_gpio_pins[i]) {
				result++;
			}
		}
	}

	if (result != 2) {
		result = -EINVAL;
		printk(KERN_ERR LIRC_DRIVER_NAME
		       ": invalid GPIO pin(s) specified!\n");
		goto exit_rpi;
	}

	driver.features = LIRC_CAN_SET_SEND_DUTY_CYCLE |
			  LIRC_CAN_SET_SEND_CARRIER |
			  LIRC_CAN_SEND_PULSE |
			  LIRC_CAN_REC_MODE2;

	driver.dev = &lirc_hb_dev->dev;
	driver.minor = lirc_register_driver(&driver);

	if (driver.minor < 0) {
		printk(KERN_ERR LIRC_DRIVER_NAME
		       ": device registration failed with %d\n", result);
		result = -EIO;
		goto exit_rpi;
	}

	printk(KERN_INFO LIRC_DRIVER_NAME ": driver registered!\n");

	result = init_port();
	if (result < 0)
		goto exit_rpi;

	return 0;

	exit_rpi:
	lirc_hb_exit();

	return result;
}

static void __exit lirc_hb_exit_module(void)
{
	lirc_unregister_driver(driver.minor);
	lirc_hb_exit();

	printk(KERN_INFO LIRC_DRIVER_NAME ": cleaned up module\n");
}

module_init(lirc_hb_init_module);
module_exit(lirc_hb_exit_module);

MODULE_DESCRIPTION("Infra-red receiver and blaster driver for Hummingboard GPIO.");
MODULE_AUTHOR("CurlyMo <development@xbian.org>");
MODULE_AUTHOR("Aron Robert Szabo <aron@reon.hu>");
MODULE_AUTHOR("Michael Bishop <cleverca22@gmail.com>");
MODULE_LICENSE("GPL");

module_param(gpio_out_pin, int, S_IRUGO);
MODULE_PARM_DESC(gpio_out_pin, "GPIO output/transmitter pin number of the BCM"
		 " processor. Valid pin numbers are: 1, 73, 72, 71, 70, 194, 195, 67, default 73");

module_param(gpio_in_pin, int, S_IRUGO);
MODULE_PARM_DESC(gpio_in_pin, "GPIO input pin number of the BCM processor."
		 " Valid pin numbers are: 1, 73, 72, 71, 70, 194, 195, 67, default 72");

module_param(sense, bool, S_IRUGO);
MODULE_PARM_DESC(sense, "Override autodetection of IR receiver circuit"
		 " (0 = active high, 1 = active low )");

module_param(softcarrier, bool, S_IRUGO);
MODULE_PARM_DESC(softcarrier, "Software carrier (0 = off, 1 = on, default on)");

module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Enable debugging messages");
