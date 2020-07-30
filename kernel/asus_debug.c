#include <linux/types.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/rtc.h>
#include <linux/list.h>
#include <linux/syscalls.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm/io.h>

char messages[256];
static ssize_t asusevtlog_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	if (count > 256)
		count = 256;

	memset(messages, 0, sizeof(messages));
	if (copy_from_user(messages, buf, count))
		return -EFAULT;

	printk(KERN_INFO "%s", messages);

	return count;
}

static const struct file_operations proc_asusevtlog_operations = {
	.write	  = asusevtlog_write,
};

static int __init proc_asusdebug_init(void)
{	
	proc_create("asusevtlog", S_IRWXUGO, NULL, &proc_asusevtlog_operations);
	return 0;
}
module_init(proc_asusdebug_init);