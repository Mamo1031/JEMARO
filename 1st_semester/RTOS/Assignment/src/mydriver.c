/**
 * mydriver.c - Simple Kernel Driver
 *
 * Functionality: Defines only three operations - open, close, and write, and outputs the string written by the user to the kernel log.
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>

#define MYDRIVER_NAME "mydriver"

static dev_t mydriver_dev;
static struct cdev mydriver_cdev;
static struct class *mydriver_class = NULL;

/* open */
static int mydriver_open(struct inode *inode, struct file *file) {
    printk(KERN_INFO "mydriver: open\n");
    return 0;
}

/* close */
static int mydriver_release(struct inode *inode, struct file *file) {
    printk(KERN_INFO "mydriver: release\n");
    return 0;
}

/* write */
static ssize_t mydriver_write(struct file *file, const char __user *buf,
                              size_t count, loff_t *ppos) {
    char kbuf[256];
    size_t len = (count < 255) ? count : 255;

    if (copy_from_user(kbuf, buf, len)) {
        return -EFAULT;
    }
    kbuf[len] = '\0';
    /* Output to kernel log */
    printk(KERN_INFO "mydriver received: %s\n", kbuf);
    return count;
}

/* file_operations */
static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = mydriver_open,
    .release = mydriver_release,
    .write = mydriver_write,
};

static int __init mydriver_init(void) {
    /* 1) Allocate device number */
    int ret = alloc_chrdev_region(&mydriver_dev, 0, 1, MYDRIVER_NAME);
    if (ret < 0) {
        printk(KERN_ERR "mydriver: failed to alloc_chrdev_region\n");
        return ret;
    }
    /* 2) Initialize cdev */
    cdev_init(&mydriver_cdev, &fops);
    mydriver_cdev.owner = THIS_MODULE;
    ret = cdev_add(&mydriver_cdev, mydriver_dev, 1);
    if (ret < 0) {
        unregister_chrdev_region(mydriver_dev, 1);
        printk(KERN_ERR "mydriver: failed to cdev_add\n");
        return ret;
    }

    /* 3) Create class */
    mydriver_class = class_create(THIS_MODULE, MYDRIVER_NAME);
    if (IS_ERR(mydriver_class)) {
        cdev_del(&mydriver_cdev);
        unregister_chrdev_region(mydriver_dev, 1);
        printk(KERN_ERR "mydriver: failed to create class\n");
        return PTR_ERR(mydriver_class);
    }

    /* 4) Create device file /dev/mydriver */
    device_create(mydriver_class, NULL, mydriver_dev, NULL, MYDRIVER_NAME);

    printk(KERN_INFO "mydriver: loaded\n");
    return 0;
}

static void __exit mydriver_exit(void) {
    device_destroy(mydriver_class, mydriver_dev);
    class_destroy(mydriver_class);
    cdev_del(&mydriver_cdev);
    unregister_chrdev_region(mydriver_dev, 1);
    printk(KERN_INFO "mydriver: unloaded\n");
}

module_init(mydriver_init);
module_exit(mydriver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("A simple driver with open/close/write");
