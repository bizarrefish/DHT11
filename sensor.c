/**

	Quick and dirty temp+humidity interface module for Bifferboard.
	Do NOT enable kernel preemption.

	Creates /proc/sensor . Open this and read it for current values.
*/


#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/io.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/current.h>

#define PROC_FILE_NAME "sensor"

#define HIGH 1
#define LOW 0
#define MS(t) (u16)(t * 1000L)
#define US(t) (u16)(t)


// Registers to configure Timer 2
#define TIMER2_COUNT		0x42
#define TIMER_CTL			0x43
#define NMI_STATUS		0x61

#define TIMER2_LATCH_CMD	0x80
#define TIMER2_INIT_COUNT	0xFFFF
#define TIMER2_INIT_CMD	0xB8
/*
SC1:	1		// Timer 2
SC0:	0
RW1:	1		// 16-Bit mode
RW0:	1
M2:		1		// Mode 4
M1:		0
M0:		0
BCD:	0		// Binary
*/

// Timer 2 clock params
#define TIMER2_HZ			(1193000L)

#define TICKS_10US		12L
#define TICKS_100US		120L
#define TICKS_1MS			1193L
#define TICKS_18MS		21474L
	

static int gpioNum = 11;
module_param(gpioNum, int, S_IRUGO);


struct sampleData {
	int tempInt;
	int tempFrac;

	int humidityInt;
	int humidityFrac;

	// 0 on success
	int error;
	
	// Should be close to 50
	u16 ackTime;
};

static inline void resetCounter() {

	// Write counter init command
	outb(TIMER2_INIT_CMD, TIMER_CTL);
	
	// Write 16-bit initial count
	outb(TIMER2_INIT_COUNT, TIMER2_COUNT);
	outb((TIMER2_INIT_COUNT >> 8), TIMER2_COUNT);
}

static void initCounter() {
	// Enable Timer 2 gate
	u8 nmi = inb(NMI_STATUS);
	nmi |= 0x01;
	outb(nmi, NMI_STATUS);
	
	resetCounter();
}

// Read current counter value
static inline u32 readCounter() {
	// Write counter latch command
	outb(TIMER2_LATCH_CMD, TIMER_CTL);
	
	// Read in current value
	u8 lsb = inb(TIMER2_COUNT);
	u8 msb = inb(TIMER2_COUNT);
	u16 count = (msb << 8) | lsb;

	// Return current value
	return count;
}

// Record how long it takes (in usec) before the GPIO = the given value
// Return -1 on timeout
static inline s16 waitGPIO(int value, u16 timeout) {
	u16 timeoutTicks = (timeout * TICKS_1MS) / 1000L;
	resetCounter();
	u16 curr, diff;
	while(gpio_get_value(gpioNum) != value) {
		curr = readCounter();
		diff = TIMER2_INIT_COUNT - curr;
		if(diff > timeoutTicks) {
			return -1;
		}
	}

	return (diff * 1000L) / TICKS_1MS;
}

static inline void waitTicks(u16 ticks) {
	while(1) {
		u16 curr = readCounter();
		u16 diff = TIMER2_INIT_COUNT - curr;
		if(diff >= ticks) {
			break;
		}
	}
}

#define WAIT(value, timeout, errcode) if(waitGPIO(value, timeout) < 0) { result.error = errcode; goto err; }


static struct sampleData readSensor() {
	struct sampleData result;
	
	unsigned long flags;
	
	result.error = 0;
	
	local_irq_save(flags);
	
	// Start the counter
	resetCounter();

	// Dip the GPIO low
	gpio_direction_output(gpioNum, LOW);

	// Hold it low for 18ms
	waitTicks(TICKS_18MS);
	
	// Release the GPIO
	gpio_direction_input(gpioNum);

	// Wait for the ACK 'dip' - should be 80us
	waitGPIO(LOW, 100);
	result.ackTime = waitGPIO(HIGH, 100);

	// The low before the first data bit (low for 50us)
	waitGPIO(LOW, 100);
	
	
	// Now read 40 bits
	u64 bits = 0;
	int i;
	for(i=0;;i++) {
		// Input currently low

		// Wait for the dip to end - doesn't matter how long
		WAIT(HIGH, 100, 4)

		if(i == 40) {
			// That was the 'stop dip'
			// We're done receiving
			break;
		}


		// High for variable length
		u16 len = waitGPIO(LOW, 100);
		if(len < 0) { result.error = 5; goto err; }

		// Longer than 50us means high bit
		bits = (bits << 1);
		if(len > 40)
			bits = bits | 1;

	}


	// So far so good...
	result.humidityInt = (bits >> 32) & 0xFF;
	result.humidityFrac = (bits >> 24) & 0xFF;
	result.tempInt = (bits >> 16) & 0xFF;
	result.tempFrac = (bits >> 8) & 0xFF;
	
	// Do the checksum
	u64 checksum = bits & 0xFF;
	u64 total = (u64)result.humidityInt + (u64)result.humidityFrac + (u64)result.tempInt + (u64)result.tempFrac;
	if((total & 0xFF) != checksum) {
		result.error = 6;
		goto err;
	}


	// And we're good!

err:
	
	// Wait 18 more msec
	waitTicks(TICKS_18MS);

	local_irq_restore(flags);
	
	return result;
}


static int sample_show(struct seq_file *s, void *p) {
	struct sampleData *data = (struct sampleData*)s->private;

	seq_printf(s, "T\t%d.%d\n", data->tempInt, data->tempFrac);
	seq_printf(s, "H\t%d.%d\n", data->humidityInt, data->humidityFrac);
	seq_printf(s, "E\t%d\n", data->error);
	seq_printf(s, "A\t%d\n", data->ackTime);


	kfree(s->private);
	return 0;
}

static int sensor_open(struct inode *inode, struct file *file) {
	struct sampleData *data = kmalloc(sizeof(struct sampleData), GFP_KERNEL);

	*data = readSensor();

	return single_open(file, sample_show, data);
}


static struct file_operations ct_file_ops = {
	.owner = THIS_MODULE,
	.open = sensor_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release
};

static int sensor_init(void) {

	int ret;

	printk(KERN_ALERT "Sensor Module Initialized\n");

	ret = gpio_request(gpioNum, "proc-sensor");
	if(ret) {
		printk(KERN_ALERT "Unable to request GPIO");
		return ret;
	}


	
	initCounter();
	
	printk(KERN_INFO "Counter before: %d", readCounter());
	printk(KERN_INFO "Counter before: %d", readCounter());

	
	// Set output value to 0
	gpio_set_value(gpioNum, 0);

	// Set GPIO to input
	gpio_direction_input(gpioNum);
	

	struct proc_dir_entry *entry;
	entry = create_proc_entry(PROC_FILE_NAME, 0, NULL);
	if(entry)
		entry->proc_fops = &ct_file_ops;



	return 0;
}

static void sensor_exit(void) {
	printk(KERN_INFO "Counter after: %d", readCounter());
	printk(KERN_ALERT "Sensor Module Exited\n");

	gpio_free(gpioNum);

	remove_proc_entry(PROC_FILE_NAME, NULL);
}


module_init(sensor_init);
module_exit(sensor_exit);

MODULE_LICENSE("LGPL");
MODULE_AUTHOR("Lee Marshall");
