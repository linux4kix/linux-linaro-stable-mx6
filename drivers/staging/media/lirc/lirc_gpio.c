/*
 * lirc_gpio.c
 *
 * lirc_gpio - Device driver that records pulse- and pause-lengths
 *	      (space-lengths) (just like the lirc_serial driver does)
 *	      between GPIO interrupt events on GPIO capable devices.
 *	      Lots of code has been taken from the lirc_serial and the
 *	      lirc_rpi modules so I would like say thanks to the authors.
 *
 * Copyright (C) 2014 CurlyMo <curlymoo1@gmail.com>
 *			  Aron Robert Szabo <aron@reon.hu>,
 *		      Michael Bishop <cleverca22@gmail.com>
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

/*
	lirc_gpio {
		compatible = "lirc_gpio";
		gpios = <&gpio3 6 1 &gpio3 7 2>;
		pinctrl-names = "default";
		pinctrl-0 = <&pinctrl_hummingboard_gpio3_6>;
		pinctrl-1 = <&pinctrl_hummingboard_gpio3_7>;
		linux,sense = <-1>;
		linux,softcarrier = <1>;
		linux,validgpios = <1 73 72 71 70 194 195 67>;
	};
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
#include <linux/of.h>
#include <linux/of_gpio.h>

#define LIRC_DRIVER_NAME "lirc_gpio"
#define RBUF_LEN 256
#define LIRC_TRANSMITTER_LATENCY 256

#ifndef MAX_UDELAY_MS
#define MAX_UDELAY_US 5000
#else
#define MAX_UDELAY_US (MAX_UDELAY_MS*1000)
#endif

static ssize_t lirc_write(struct file *file, const char *buf, size_t n, loff_t *ppos);
static long lirc_ioctl(struct file *filep, unsigned int cmd, unsigned long arg);
static int set_use_inc(void *data);
static void set_use_dec(void *data);
static int lirc_gpio_probe(struct platform_device *pdev);
static int lirc_gpio_remove(struct platform_device *pdev);

struct lirc_gpio_platform_data {
	int		gpio_rx_nr;
	int		gpio_tx_nr;
	bool	active_rx_low;
	bool	active_tx_low;
	u64		allowed_rx_protos;
	u64		allowed_tx_protos;
	int		sense;
	int		softcarrier;
	int		validgpios[255];
};

struct lirc_gpio_dev {
	int gpio_rx_nr;
	int gpio_tx_nr;
	int sense;
	int softcarrier;
	int validgpios[255];
};

struct lirc_gpio_dev *gpio_dev;

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

struct irq_chip *irqchip;
struct irq_data *irqdata;

static struct timeval lasttv = { 0, 0 };
static struct lirc_buffer rbuf;
static spinlock_t lock;

/* set the default GPIO input pin */
static int gpio_in_pin = -1;
/* set the default GPIO output pin */
static int gpio_out_pin = -1;
/* -1 = auto, 0 = active high, 1 = active low */
static int sense = -2;
/* use softcarrier by default */
static int softcarrier = -1;

/* initialized/set in init_timing_params() */
static unsigned int freq = 38000;
static unsigned int duty_cycle = 50;
static unsigned long period;
static unsigned long pulse_width;
static unsigned long space_width;

static struct lirc_driver driver = {
	.name			= LIRC_DRIVER_NAME,
	.minor			= -1,
	.code_length	= 1,
	.sample_rate	= 0,
	.data			= NULL,
	.add_to_buf		= NULL,
	.rbuf			= &rbuf,
	.set_use_inc	= set_use_inc,
	.set_use_dec	= set_use_dec,
	.fops			= &lirc_fops,
	.dev			= NULL,
	.owner			= THIS_MODULE,
};

static struct of_device_id lirc_gpio_of_match[] = {
	{ .compatible = "lirc_gpio", },
	{}
};

static struct platform_driver lirc_gpio_driver = {
	.probe  = lirc_gpio_probe,
	.remove = lirc_gpio_remove,
	.driver = {
		.name   = LIRC_DRIVER_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = lirc_gpio_of_match,
	},
};

static void safe_udelay(unsigned long usecs) {
	while (usecs > MAX_UDELAY_US) {
		udelay(MAX_UDELAY_US);
		usecs -= MAX_UDELAY_US;
	}
	udelay(usecs);
}

static int init_timing_params(unsigned int new_duty_cycle, unsigned int new_freq) {
	/*
	 * period, pulse/space width are kept with 8 binary places -
	 * IE multiplied by 256.
	 */
	if(256 * 1000000L / new_freq * new_duty_cycle / 100 <=
	    LIRC_TRANSMITTER_LATENCY)
		return -EINVAL;
	if(256 * 1000000L / new_freq * (100 - new_duty_cycle) / 100 <=
	    LIRC_TRANSMITTER_LATENCY)
		return -EINVAL;
	duty_cycle = new_duty_cycle;
	freq = new_freq;
	period = 256 * 1000000L / freq;
	pulse_width = period * duty_cycle / 100;
	space_width = period - pulse_width;
	return 0;
}


static long send_pulse_softcarrier(unsigned long length) {
	int flag;
	unsigned long actual, target, d;

	if(gpio_dev->gpio_tx_nr >= 0) {
		length <<= 8;

		actual = 0; target = 0; flag = 0;
		while(actual < length) {
			if(flag) {
				gpio_set_value(gpio_dev->gpio_tx_nr, 0);
				target += space_width;
			} else {
				gpio_set_value(gpio_dev->gpio_tx_nr, 1);
				target += pulse_width;
			}
			d = (target - actual - LIRC_TRANSMITTER_LATENCY + 128) >> 8;
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
	return 0;
}

static long send_pulse(unsigned long length) {
	if(length <= 0)
		return 0;

	if(gpio_dev->gpio_tx_nr >= 0) {
		if(gpio_dev->softcarrier) {
			return send_pulse_softcarrier(length);
		} else {
			gpio_set_value(gpio_dev->gpio_tx_nr, 1);
			safe_udelay(length);
			return 0;
		}
	}
	return 0;
}

static void send_space(long length) {
	if(gpio_dev->gpio_tx_nr >= 0) {
		gpio_set_value(gpio_dev->gpio_tx_nr, 0);
		if(length <= 0)
			return;
		safe_udelay(length);
	}
}

static void rbwrite(int l) {
	if (lirc_buffer_full(&rbuf)) {
		/* no new signals will be accepted */
		return;
	}
	lirc_buffer_write(&rbuf, (void *)&l);
}

static void frbwrite(int l) {
	/* simple noise filter */
	static int pulse, space;
	static unsigned int ptr;

	if(ptr > 0 && (l & PULSE_BIT)) {
		pulse += l & PULSE_MASK;
		if(pulse > 250) {
			rbwrite(space);
			rbwrite(pulse | PULSE_BIT);
			ptr = 0;
			pulse = 0;
		}
		return;
	}
	if(!(l & PULSE_BIT)) {
		if(ptr == 0) {
			if (l > 20000) {
				space = l;
				ptr++;
				return;
			}
		} else {
			if(l > 20000) {
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

static irqreturn_t irq_handler(int i, void *blah, struct pt_regs *regs) {
	struct timeval tv;
	long deltv;
	int data;
	int signal;

	/* use the GPIO signal level */
	signal = gpio_get_value(gpio_dev->gpio_rx_nr);

	/* unmask the irq */
	irqchip->irq_unmask(irqdata);

	if(gpio_dev->sense != -1) {
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
		if(tv.tv_sec < lasttv.tv_sec ||
		    (tv.tv_sec == lasttv.tv_sec &&
		     tv.tv_usec < lasttv.tv_usec)) {
			printk(KERN_WARNING LIRC_DRIVER_NAME
			       ": AIEEEE: your clock just jumped backwards\n");
			printk(KERN_WARNING LIRC_DRIVER_NAME
			       ": %d %d %lx %lx %lx %lx\n", signal, gpio_dev->sense,
			       tv.tv_sec, lasttv.tv_sec,
			       tv.tv_usec, lasttv.tv_usec);
			data = PULSE_MASK;
		} else if (deltv > 15) {
			data = PULSE_MASK; /* really long time */
			if(!(signal^gpio_dev->sense)) {
				/* sanity check */
				printk(KERN_WARNING LIRC_DRIVER_NAME
				       ": AIEEEE: %d %d %lx %lx %lx %lx\n",
				       signal, gpio_dev->sense, tv.tv_sec, lasttv.tv_sec,
				       tv.tv_usec, lasttv.tv_usec);
				/*
				 * detecting pulse while this
				 * MUST be a space!
				 */
				gpio_dev->sense = gpio_dev->sense ? 0 : 1;
			}
		} else {
			data = (int) (deltv*1000000 +
				      (tv.tv_usec - lasttv.tv_usec));
		}
		frbwrite(signal^gpio_dev->sense ? data : (data|PULSE_BIT));
		lasttv = tv;
		wake_up_interruptible(&rbuf.wait_poll);
	}

	return IRQ_HANDLED;
}

// called when the character device is opened
static int set_use_inc(void *data) {
	int result;
	unsigned long flags;

	/* initialize timestamp */
	do_gettimeofday(&lasttv);

	if(gpio_dev->gpio_rx_nr >= 0) {
		result = request_irq(gpio_to_irq(gpio_dev->gpio_rx_nr),
					 (irq_handler_t) irq_handler, 0,
					 LIRC_DRIVER_NAME, (void*) 0);

		switch (result) {
		case -EBUSY:
			printk(KERN_ERR LIRC_DRIVER_NAME
				   ": IRQ %d is busy\n",
				   gpio_to_irq(gpio_dev->gpio_rx_nr));
			return -EBUSY;
		case -EINVAL:
			printk(KERN_ERR LIRC_DRIVER_NAME
				   ": Bad irq number or handler\n");
			return -EINVAL;
		default:
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
	}

	return 0;
}

static void set_use_dec(void *data) {
	unsigned long flags;
	if(gpio_dev->gpio_rx_nr >= 0) {
		spin_lock_irqsave(&lock, flags);

		/* GPIO Pin Falling/Rising Edge Detect Disable */
		irqchip->irq_set_type(irqdata, 0);
		irqchip->irq_mask(irqdata);

		spin_unlock_irqrestore(&lock, flags);

		free_irq(gpio_to_irq(gpio_dev->gpio_rx_nr), (void *) 0);
	}
}

static ssize_t lirc_write(struct file *file, const char *buf, size_t n, loff_t *ppos) {
	int i, count;
	unsigned long flags;
	long delta = 0;
	int *wbuf;

	if(gpio_dev->gpio_tx_nr >= 0) {
		count = n / sizeof(int);
		if(n % sizeof(int) || count % 2 == 0)
			return -EINVAL;
		wbuf = memdup_user(buf, n);
		if(IS_ERR(wbuf))
			return PTR_ERR(wbuf);
		spin_lock_irqsave(&lock, flags);

		for(i = 0; i < count; i++) {
			if(i%2)
				send_space(wbuf[i] - delta);
			else
				delta = send_pulse(wbuf[i]);
		}
		gpio_set_value(gpio_dev->gpio_tx_nr, 0);

		spin_unlock_irqrestore(&lock, flags);
		kfree(wbuf);
		return n;
	}
	return 0;
}


static long lirc_ioctl(struct file *filep, unsigned int cmd, unsigned long arg) {
	int result;
	__u32 value;

	switch(cmd) {
		case LIRC_GET_SEND_MODE:
			return -ENOIOCTLCMD;
			break;

		case LIRC_SET_SEND_MODE:
			result = get_user(value, (__u32 *) arg);
			if(result)
				return result;
			/* only LIRC_MODE_PULSE supported */
			if(value != LIRC_MODE_PULSE)
				return -ENOSYS;
			break;

		case LIRC_GET_LENGTH:
			return -ENOSYS;
			break;

		case LIRC_SET_SEND_DUTY_CYCLE:
			result = get_user(value, (__u32 *) arg);
			if (result)
				return result;
			if (value <= 0 || value > 100)
				return -EINVAL;
			return init_timing_params(value, freq);
			break;

		case LIRC_SET_SEND_CARRIER:
			result = get_user(value, (__u32 *) arg);
			if(result)
				return result;
			if(value > 500000 || value < 20000)
				return -EINVAL;
			return init_timing_params(duty_cycle, value);
			break;

		default:
			return lirc_dev_fop_ioctl(filep, cmd, arg);
	}
	return 0;
}

static int lirc_gpio_get_devtree_pdata(struct device *dev, struct lirc_gpio_platform_data *pdata) {
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags;
	struct property *prop;
	const __be32 *cur;
	int gpio = -1;
	int ret = 0;
	int i = 0;

	if(np) {
		gpio = of_get_gpio_flags(np, 0, &flags);
		if(gpio < 0) {
			if(gpio != -EPROBE_DEFER)
				dev_err(dev, "RX gpio not defined (%d)\n", gpio);

			pdata->gpio_rx_nr = -1;
			pdata->active_rx_low = 0;
			pdata->allowed_rx_protos = 0;
		} else {
			pdata->gpio_rx_nr = gpio;
			pdata->active_rx_low = (flags & OF_GPIO_ACTIVE_LOW);
			pdata->allowed_rx_protos = 0;
		}

		gpio = of_get_gpio_flags(np, 1, &flags);
		if(gpio < 0) {
			if(gpio != -EPROBE_DEFER)
				dev_err(dev, "TX gpio not defined (%d)\n", gpio);

			pdata->gpio_tx_nr = -1;
			pdata->active_tx_low = 0;
			pdata->allowed_tx_protos = 0;
		} else {
			pdata->gpio_tx_nr = gpio;
			pdata->active_tx_low = (flags & OF_GPIO_ACTIVE_LOW);
			pdata->allowed_tx_protos = 0;
		}
		ret = of_property_read_u32(np, "linux,sense", &pdata->sense);
		if(ret) {
			pdata->sense = -1;
		}
		ret = of_property_read_u32(np, "linux,softcarrier", &pdata->softcarrier);
		if(ret) {
			pdata->softcarrier = 1;
		}
		i = 0;
		printk(KERN_DEBUG LIRC_DRIVER_NAME ": valid gpios");
		of_property_for_each_u32(np, "linux,validgpios", prop, cur, gpio) {
			printk(" %d", gpio);
			pdata->validgpios[i++] = gpio;
		}
		printk("\n");
		pdata->validgpios[i] = -1;
	}

	return 0;
}

static int init_port(void) {
	int i, nlow, nhigh, ret, irq;

	if(gpio_dev->gpio_tx_nr >= 0) {
		if(gpio_request(gpio_dev->gpio_tx_nr, LIRC_DRIVER_NAME " ir/out")) {
			printk(KERN_ALERT LIRC_DRIVER_NAME ": cant claim gpio pin %d\n", gpio_dev->gpio_tx_nr);
			ret = -ENODEV;
			goto exit_init_port;
		}
	}

	if(gpio_dev->gpio_rx_nr >= 0) {
		if(gpio_request(gpio_dev->gpio_rx_nr, LIRC_DRIVER_NAME " ir/in")) {
			printk(KERN_ALERT LIRC_DRIVER_NAME ": cant claim gpio pin %d\n", gpio_dev->gpio_rx_nr);
			ret = -ENODEV;
			goto exit_gpio_free_out_pin;
		}
	}

	if(gpio_dev->gpio_rx_nr >= 0) {
		gpio_direction_input(gpio_dev->gpio_rx_nr);
	}
	if(gpio_dev->gpio_tx_nr >= 0) {
		gpio_direction_output(gpio_dev->gpio_tx_nr, 1);
		gpio_set_value(gpio_dev->gpio_tx_nr, 0);
	}

	if(gpio_dev->gpio_rx_nr >= 0) {
		irq = gpio_to_irq(gpio_dev->gpio_rx_nr);
		irqdata = irq_get_irq_data(irq);

		if(irqdata && irqdata->chip) {
			irqchip = irqdata->chip;
		} else {
			ret = -ENODEV;
			goto exit_gpio_free_in_pin;
		}

		/* if pin is high, then this must be an active low receiver. */
		if(gpio_dev->sense == -1) {
			/* wait 1/2 sec for the power supply */
			msleep(500);

			/*
			 * probe 9 times every 0.04s, collect "votes" for
			 * active high/low
			 */
			nlow = 0;
			nhigh = 0;
			for(i = 0; i < 9; i++) {
				if(gpio_get_value(gpio_dev->gpio_rx_nr))
					nlow++;
				else
					nhigh++;
				msleep(40);
			}
			gpio_dev->sense = (nlow >= nhigh ? 1 : 0);
			printk(KERN_INFO LIRC_DRIVER_NAME ": auto-detected active %s receiver on GPIO pin %d\n",
				   gpio_dev->sense ? "low" : "high", gpio_dev->gpio_rx_nr);
		} else {
			printk(KERN_INFO LIRC_DRIVER_NAME ": manually using active %s receiver on GPIO pin %d\n",
				   gpio_dev->sense ? "low" : "high", gpio_dev->gpio_rx_nr);
		}
	}

	return 0;

exit_gpio_free_in_pin:
	gpio_free(gpio_dev->gpio_rx_nr);

exit_gpio_free_out_pin:
	gpio_free(gpio_dev->gpio_tx_nr);

exit_init_port:
	return ret;
}

static void lirc_gpio_exit(void) {
	if(gpio_dev->gpio_tx_nr >= 0) {
		gpio_free(gpio_dev->gpio_tx_nr);
	}
	if(gpio_dev->gpio_rx_nr >= 0) {
		gpio_free(gpio_dev->gpio_rx_nr);
	}

	lirc_unregister_driver(driver.minor);
	lirc_buffer_free(&rbuf);
}

static int lirc_gpio_probe(struct platform_device *pdev) {
	const struct lirc_gpio_platform_data *pdata =
					pdev->dev.platform_data;
	int rc;
	int result = 0;
	int match = 0;
	int i = 0;

	if(pdev->dev.of_node) {
		struct lirc_gpio_platform_data *dtpdata = devm_kzalloc(&pdev->dev, sizeof(*dtpdata), GFP_KERNEL);
		if(!dtpdata)
			return -ENOMEM;
		rc = lirc_gpio_get_devtree_pdata(&pdev->dev, dtpdata);
		if(rc)
			return rc;
		pdata = dtpdata;
	}

	if(!pdata)
		return -EINVAL;

	gpio_dev = kzalloc(sizeof(struct lirc_gpio_dev), GFP_KERNEL);
	if(!gpio_dev)
		return -ENOMEM;

	gpio_dev->gpio_rx_nr = pdata->gpio_rx_nr;
	gpio_dev->gpio_tx_nr = pdata->gpio_tx_nr;
	gpio_dev->sense = pdata->sense;
	gpio_dev->softcarrier = pdata->softcarrier;
	memcpy(gpio_dev->validgpios, pdata->validgpios, 255);

	if(gpio_in_pin != gpio_out_pin) {
		match = 0;
		for(i = 0; (i < ARRAY_SIZE(gpio_dev->validgpios)) && (!match) && (gpio_dev->validgpios[i] != -1); i++) {
			if(gpio_in_pin == gpio_dev->validgpios[i]) {
				match = 1;
				break;
			}
		}
		if(gpio_in_pin > -1) {
			if(!match) {
				printk(KERN_ERR LIRC_DRIVER_NAME
				   ": invalid RX GPIO pin specified!\n");
				return -EINVAL;
			} else {
				gpio_dev->gpio_rx_nr = gpio_in_pin;
			}
		}
		match = 0;
		for(i = 0; (i < ARRAY_SIZE(gpio_dev->validgpios)) && (!match) && (gpio_dev->validgpios[i] != -1); i++) {
			if(gpio_out_pin == gpio_dev->validgpios[i]) {
				match = 1;
				break;
			}
		}
		if(gpio_out_pin > -1) {
			if(!match) {
				printk(KERN_ERR LIRC_DRIVER_NAME
				   ": invalid TX GPIO pin specified!\n");
				return -EINVAL;
			} else {
				gpio_dev->gpio_tx_nr = gpio_out_pin;
			}
		}
	}
	if(sense > -2) {
		gpio_dev->sense = sense;
	}
	if(softcarrier >= 0) {
		gpio_dev->softcarrier = softcarrier;
	}

	printk(KERN_DEBUG LIRC_DRIVER_NAME ": rx %d, tx %d, sense %d, softcarrier %d\n",
		   gpio_dev->gpio_rx_nr, gpio_dev->gpio_tx_nr, gpio_dev->sense, gpio_dev->softcarrier);

	platform_set_drvdata(pdev, gpio_dev);

	result = lirc_buffer_init(&rbuf, sizeof(int), RBUF_LEN);
	if(result < 0)
		return -ENOMEM;

	driver.features = LIRC_CAN_SET_SEND_DUTY_CYCLE |
			  LIRC_CAN_SET_SEND_CARRIER |
			  LIRC_CAN_SEND_PULSE |
			  LIRC_CAN_REC_MODE2;

	driver.dev = &pdev->dev;
	driver.minor = lirc_register_driver(&driver);

	if(driver.minor < 0) {
		printk(KERN_ERR LIRC_DRIVER_NAME ": device registration failed with %d\n", result);
		result = -EIO;
		goto exit_gpio;
	}

	result = init_port();
	if(result < 0)
		goto exit_gpio;

	return 0;

exit_gpio:
	lirc_gpio_exit();

	return result;
}

static int lirc_gpio_remove(struct platform_device *pdev) {
	struct lirc_gpio_dev *gpio_dev = platform_get_drvdata(pdev);

	lirc_gpio_exit();

	kfree(gpio_dev);

	return 0;
}

MODULE_DEVICE_TABLE(of, lirc_gpio_of_match);
module_platform_driver(lirc_gpio_driver);

MODULE_DESCRIPTION("Infra-red GPIO receiver and blaster driver.");
MODULE_AUTHOR("CurlyMo <development@xbian.org>");
MODULE_AUTHOR("Aron Robert Szabo <aron@reon.hu>");
MODULE_AUTHOR("Michael Bishop <cleverca22@gmail.com>");
MODULE_LICENSE("GPL");

module_param(gpio_out_pin, int, S_IRUGO);
MODULE_PARM_DESC(gpio_out_pin, "GPIO output/transmitter pin number");

module_param(gpio_in_pin, int, S_IRUGO);
MODULE_PARM_DESC(gpio_in_pin, "GPIO input/receiver pin number.");

module_param(sense, int, S_IRUGO);
MODULE_PARM_DESC(sense, "Override autodetection of IR receiver circuit"
		 " (0 = active high, 1 = active low )");

module_param(softcarrier, int, S_IRUGO);
MODULE_PARM_DESC(softcarrier, "Software carrier (0 = off, 1 = on, default on)");

