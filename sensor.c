/**

	Quick and dirty DHT11 temp+humidity sensor interface module for Bifferboard.
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
#include <linux/hrtimer.h>
#include <linux/ktime.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/current.h>


#define PROC_FILE_NAME "sensor"

static int gpioNum = 16;
module_param(gpioNum, int, S_IRUGO);

struct sampleData {
	int tempInt;
	int tempFrac;

	int humidityInt;
	int humidityFrac;

	// 0 on success
	int error;
};


// Record how long it takes (in ns) before the GPIO = the given value
// Return -1 on timeout
static s64 waitGPIO(int value, u64 timeout) {

	u64 startTime = ktime_to_ns(ktime_get());

	while(gpio_get_value(gpioNum) != value) {
		
		u64 curr = ktime_to_ns(ktime_get());
		u64 diff = curr - startTime;
		if(diff >= timeout)
			return -1;
	}

	return ktime_to_ns(ktime_get()) - startTime;
}


#define HIGH 1
#define LOW 0
#define MS(t) (u64)(t * 1000000L)
#define US(t) (u64)(t * 1000L)
#define WAIT(value, timeout, errcode) if(waitGPIO(value, timeout) < 0) { result.error = errcode; goto err; }

static struct sampleData readSensor() {
	struct sampleData result;
	

	unsigned long flags;
	
	// Grab the current time
	u64 startTime = ktime_to_ns(ktime_get());

	// Dip the GPIO low
	gpio_direction_output(gpioNum, LOW);

	// Hold it low for 18ms
	
	int i;
	while(1) {
		u64 curr = ktime_to_ns(ktime_get());
		u64 diff = curr - startTime;
		if(diff >= MS(18)) {
	//		printk(KERN_INFO "Diff: %llu / %llu", diff, MS(18));
			break;
		}
	}

	// Release the GPIO
	gpio_direction_input(gpioNum);

	// Wait for the ACK 'dip' - should be 80us
	WAIT(LOW, US(100), 1);
	WAIT(HIGH, US(100), 2);
//	s64 dipLen = waitGPIO(HIGH, US(100));
//	if(dipLen < 0) { result.error = 2; goto err; }



	// The low before the first data bit (low for 50us)
	WAIT(LOW, US(100), 3);

	// Now read 40 bits
	u64 bits = 0;
	for(i=0;;i++) {
		// Input currently low

		// Wait for the dip to end - doesn't matter how long
		WAIT(HIGH, US(100), 4)

		if(i == 40) {
			// That was the 'stop dip'
			// We're done receiving
			break;
		}


		// High for variable length
		int len = waitGPIO(LOW, US(100));
		if(len < 0) { result.error = 5; goto err; }

		// Longer than 50us means high bit
		bits = (bits << 1);
		if(len > US(50))
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
	return result;
}


static int sample_show(struct seq_file *s, void *p) {
	struct sampleData *data = (struct sampleData*)s->private;

	seq_printf(s, "T\t%d.%d\n", data->tempInt, data->tempFrac);
	seq_printf(s, "H\t%d.%d\n", data->humidityInt, data->humidityFrac);
	seq_printf(s, "E\t%d\n", data->error);

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
	printk(KERN_ALERT "Sensor Module Exited\n");

	remove_proc_entry(PROC_FILE_NAME, NULL);
	
	gpio_free(gpioNum);
}


module_init(sensor_init);
module_exit(sensor_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lee Marshall");
