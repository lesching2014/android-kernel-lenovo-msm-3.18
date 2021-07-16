#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#include <linux/kthread.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/spi/spidev.h>
#include <linux/semaphore.h>
#include <linux/poll.h>
#include <linux/fcntl.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/input.h>
#include <linux/signal.h>
#include <linux/gpio.h>
#include <linux/mm.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/of_irq.h>
#include <linux/regulator/consumer.h>
#include <linux/fb.h>
#include <linux/notifier.h>

//Huaqin add for fingprint sensor name by zhaojunhai1 at 2018/03/06 start
#if defined(CONFIG_HQ_DEV_INFO)
#include <linux/dev_info.h>
#endif
//Huaqin add for fingprint sensor name by zhaojunhai1 at 2018/03/06 end

typedef struct key_report {
    int key;
	int value;
}key_report_t;


#define DEVICE_NAME "fpsdev0"
#define CDFINGER_IOCTL_MAGIC_NO          0xFB
#define CDFINGER_INIT                    _IOW(CDFINGER_IOCTL_MAGIC_NO, 0, uint8_t)
#define CDFINGER_GETIMAGE                _IOW(CDFINGER_IOCTL_MAGIC_NO, 1, uint8_t)
#define CDFINGER_INITERRUPT_MODE	 	_IOW(CDFINGER_IOCTL_MAGIC_NO, 2, uint8_t)
#define CDFINGER_INITERRUPT_KEYMODE      _IOW(CDFINGER_IOCTL_MAGIC_NO, 3, uint8_t)
#define CDFINGER_INITERRUPT_FINGERUPMODE _IOW(CDFINGER_IOCTL_MAGIC_NO, 4, uint8_t)
#define CDFINGER_RELEASE_WAKELOCK        _IO(CDFINGER_IOCTL_MAGIC_NO, 5)
#define CDFINGER_CHECK_INTERRUPT         _IO(CDFINGER_IOCTL_MAGIC_NO, 6)
#define CDFINGER_SET_SPI_SPEED           _IOW(CDFINGER_IOCTL_MAGIC_NO, 7, uint32_t)
#define CDFINGER_POWERDOWN               _IO(CDFINGER_IOCTL_MAGIC_NO, 11)
#define	CDFINGER_GETID			 		 _IO(CDFINGER_IOCTL_MAGIC_NO,12)

#define CDFINGER_HW_RESET               _IOW(CDFINGER_IOCTL_MAGIC_NO, 14, uint8_t)
#define CDFINGER_GET_STATUS               _IO(CDFINGER_IOCTL_MAGIC_NO, 15)
#define CDFINGER_NEW_KEYMODE		_IOW(CDFINGER_IOCTL_MAGIC_NO, 37, uint8_t)

#define	CDFINGER_REPORT_KEY		  _IOW(CDFINGER_IOCTL_MAGIC_NO,19,key_report_t)
#define CDFINGER_INIT_GPIO		 _IO(CDFINGER_IOCTL_MAGIC_NO,20)
#define CDFINGER_INIT_IRQ		 _IO(CDFINGER_IOCTL_MAGIC_NO,21)
#define CDFINGER_POWER_ON		 _IO(CDFINGER_IOCTL_MAGIC_NO,22)
#define CDFINGER_RESET		 	 _IO(CDFINGER_IOCTL_MAGIC_NO,23)
#define CDFINGER_RELEASE_DEVICE	 _IO(CDFINGER_IOCTL_MAGIC_NO,25)
#define CDFINGER_WAKE_LOCK	 _IOW(CDFINGER_IOCTL_MAGIC_NO,26,uint8_t)

static int isInKeyMode = 0; // key mode
static int screen_status = 1; // screen on
static u8 cdfinger_debug = 0x01;
#define CDFINGER_DBG(fmt, args...) \
	do{ \
		if(cdfinger_debug & 0x01) \
		printk( "[DBG][cdfinger]:%5d: <%s>" fmt, __LINE__,__func__,##args ); \
	}while(0)
#define CDFINGER_ERR(fmt, args...) \
	do{ \
		printk( "[DBG][cdfinger]:%5d: <%s>" fmt, __LINE__,__func__,##args ); \
	}while(0)



struct cdfingerfp_data {
	struct platform_device *cdfinger_dev;
	struct miscdevice *miscdev;
	u32 irq_num;
	u32 reset_num;
//	u32 pwr_num;
	struct fasync_struct *async_queue;
	struct wake_lock cdfinger_lock;
	struct input_dev* cdfinger_input;
	struct notifier_block notifier;
	struct mutex buf_lock;
}*g_cdfingerfp_data;

static int cdfinger_init_gpio(struct cdfingerfp_data *cdfinger)
{
	int err = 0;
#if 0
	if (gpio_is_valid(cdfinger->pwr_num)) {
		err = gpio_request(cdfinger->pwr_num, "cdfinger-pwr");
		if (err) {
			CDFINGER_DBG("Could not request pwr gpio.\n");
			return err;
		}
	}
	else {
		CDFINGER_DBG("not valid pwr gpio\n");
		return -EIO;
	}
#endif
/*Huaqin modify for Don't request gpio by zhaojunhai1 at 2018/03/02 start*/
/*
	if (gpio_is_valid(cdfinger->reset_num)) {
		err = gpio_request(cdfinger->reset_num, "cdfinger-reset");

		if (err) {
			CDFINGER_DBG("Could not request reset gpio.\n");
			return err;
		}
		CDFINGER_DBG("request reset gpio ok\n");
	}
	else {
		CDFINGER_DBG("not valid reset gpio\n");
		return -EIO;
	}

	if (gpio_is_valid(cdfinger->irq_num)) {
		err = pinctrl_request_gpio(cdfinger->irq_num);

		if (err) {
			CDFINGER_DBG("Could not request irq gpio.\n");
			gpio_free(cdfinger->reset_num);
			return err;
		}
		CDFINGER_DBG("request irq gpio ok\n");
	}
	else {
		CDFINGER_DBG(KERN_ERR "not valid irq gpio\n");
		gpio_free(cdfinger->reset_num);
		return -EIO;
	}
	pinctrl_gpio_direction_input(cdfinger->irq_num);
*/
/*Huaqin modify for Don't request gpio by zhaojunhai1 at 2018/03/02 end*/
	CDFINGER_DBG("%s(..) ok! exit.\n", __FUNCTION__);
	return err;
}

static int cdfinger_free_gpio(struct cdfingerfp_data *cdfinger)
{
	int err = 0;
	CDFINGER_DBG("%s(..) enter.\n", __FUNCTION__);

	if (gpio_is_valid(cdfinger->irq_num)) {
		pinctrl_free_gpio(cdfinger->irq_num);
		free_irq(gpio_to_irq(cdfinger->irq_num), (void*)cdfinger);
	}

	if (gpio_is_valid(cdfinger->reset_num)) {
		gpio_free(cdfinger->reset_num);
	}
#if 0
	if (gpio_is_valid(cdfinger->pwr_num)) {
		gpio_free(cdfinger->pwr_num);
	}
#endif

	CDFINGER_DBG("%s(..) ok! exit.\n", __FUNCTION__);

	return err;
}

static int cdfinger_parse_dts(struct device *dev,struct cdfingerfp_data *cdfinger)
{
	int err = 0;

	//cdfinger->pwr_num = of_get_named_gpio(dev->of_node, "qcom,pwr-gpio", 0);
	cdfinger->reset_num = of_get_named_gpio(dev->of_node, "cdfinger,gpio_reset", 0);
	cdfinger->irq_num = of_get_named_gpio(dev->of_node, "cdfinger,gpio_irq", 0);


	CDFINGER_DBG("cdfinger of node reset[%d] irq[%d]\n",cdfinger->reset_num, cdfinger->irq_num);

	return err;
}

static int cdfinger_power_on(struct cdfingerfp_data *pdata)
{

#if 0
	gpio_direction_output(pdata->pwr_num, 1);
	mdelay(1);
#endif
	gpio_set_value(pdata->reset_num, 1);
	msleep(10);
	return 0;

}

static int cdfinger_open(struct inode *inode,struct file *file)
{
	printk("cdfinger cdfinger_open\n");
	file->private_data = g_cdfingerfp_data;
	return 0;
}

static int cdfinger_async_fasync(int fd,struct file *file,int mode)
{
	struct cdfingerfp_data *cdfingerfp = g_cdfingerfp_data;
	CDFINGER_DBG("cdfinger irq wake");
	return fasync_helper(fd,file,mode,&cdfingerfp->async_queue);
}

static int cdfinger_release(struct inode *inode,struct file *file)
{
	struct cdfingerfp_data *cdfingerfp = file->private_data;
	printk("cdfinger cdfinger_release\n");
	if(NULL == cdfingerfp)
	{
		return -EIO;
	}
	file->private_data = NULL;
	return 0;
}

static void cdfinger_async_report(void)
{
	struct cdfingerfp_data *cdfingerfp = g_cdfingerfp_data;
	wake_lock_timeout(&cdfingerfp->cdfinger_lock, msecs_to_jiffies(1000));
	kill_fasync(&cdfingerfp->async_queue,SIGIO,POLL_IN);
}

static irqreturn_t cdfinger_eint_handler(int irq, void *dev_id)
{
	cdfinger_async_report();
	return IRQ_HANDLED;
}

static void cdfinger_reset_gpio_init(struct cdfingerfp_data *pdata, int ms)
{
	gpio_direction_output(pdata->reset_num, 1);
	mdelay(ms);
	gpio_set_value(pdata->reset_num, 0);
	mdelay(ms);
	gpio_set_value(pdata->reset_num, 1);
	mdelay(ms);
}

static int cdfinger_eint_gpio_init(struct cdfingerfp_data *pdata)
{
	int error = 0;

	error =request_irq(gpio_to_irq(pdata->irq_num),cdfinger_eint_handler,IRQF_TRIGGER_RISING,"cdfinger_eint", NULL);
	if (error < 0)
	{
		CDFINGER_ERR("cdfinger_eint_gpio_init error----------\n");
		return error;
	}
	CDFINGER_DBG("cdfinger_eint_gpio_init ok----------\n");
	enable_irq_wake(gpio_to_irq(pdata->irq_num));

	//Huaqin add for fingprint sensor name by zhaojunhai1 at 2018/03/06 end
#if defined(CONFIG_HQ_DEV_INFO)
       dev_info_register("Finger","fps998e");
#endif
	//Huaqin add for fingprint sensor name by zhaojunhai1 at 2018/03/06 end

	return error;
}

static void cdfinger_wake_lock(struct cdfingerfp_data *pdata,int arg)
{
	CDFINGER_DBG("cdfinger_wake_lock enter----------\n");
	if(arg)
	{
		wake_lock(&pdata->cdfinger_lock);
	}
	else
	{
		wake_unlock(&pdata->cdfinger_lock);
		wake_lock_timeout(&pdata->cdfinger_lock, msecs_to_jiffies(3000));
	}
}

static int cdfinger_report_key(struct cdfingerfp_data *cdfinger, unsigned long arg)
{
	key_report_t  report;
	if(copy_from_user(&report,(char *) arg, sizeof(key_report_t)))
		return -1;

	input_report_key(cdfinger->cdfinger_input, report.key, !!report.value);
	input_sync(cdfinger->cdfinger_input);

	return 0;
}

static long cdfinger_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct cdfingerfp_data *cdfinger = filp->private_data;
	mutex_lock(&cdfinger->buf_lock);
	printk("cdfinger cmd = %u\n", cmd);
	switch (cmd) {
		case CDFINGER_INIT_GPIO:
			printk("cdfinger CDFINGER_INIT_GPIO\n");
			err = cdfinger_init_gpio(cdfinger);
			break;
		case CDFINGER_INIT_IRQ:
			printk("cdfinger CDFINGER_INIT_IRQ\n");
			err = cdfinger_eint_gpio_init(cdfinger);
			break;
		case CDFINGER_WAKE_LOCK:
			printk("cdfinger CDFINGER_WAKE_LOCK\n");
			cdfinger_wake_lock(cdfinger,arg);
			break;
		case CDFINGER_RELEASE_DEVICE:
			printk("cdfinger CDFINGER_RELEASE_DEVICE\n");
			cdfinger_free_gpio(cdfinger);
			if (cdfinger->cdfinger_input != NULL) {
			}
			misc_deregister(cdfinger->miscdev);
			break;
		case CDFINGER_POWER_ON:
			printk("cdfinger CDFINGER_POWER_ON\n");
			cdfinger_power_on(cdfinger);
			break;
		case CDFINGER_RESET:
			printk("cdfinger CDFINGER_RESET\n");
			cdfinger_reset_gpio_init(cdfinger,10);
			break;
		case CDFINGER_REPORT_KEY:
			printk("cdfinger CDFINGER_REPORT_KEY\n");
			err = cdfinger_report_key(cdfinger,arg);
			break;
		case CDFINGER_NEW_KEYMODE:
			printk("cdfinger CDFINGER_NEW_KEYMODE\n");
			isInKeyMode = 0;
			break;
		case CDFINGER_INITERRUPT_MODE:
			printk("cdfinger CDFINGER_INITERRUPT_MODE\n");
			isInKeyMode = 1;  // not key mode
			cdfinger_reset_gpio_init(cdfinger,1);
			break;
		case CDFINGER_HW_RESET:
			printk("cdfinger CDFINGER_HW_RESET\n");
			cdfinger_reset_gpio_init(cdfinger,arg);
			break;
		case CDFINGER_GET_STATUS:
			printk("cdfinger CDFINGER_GET_STATUS\n");
			err = screen_status;
			break;
		default:
			printk("cdfinger default\n");
			break;
	}
	mutex_unlock(&cdfinger->buf_lock);
	return err;
}

static const struct file_operations cdfinger_fops = {
	.owner	 = THIS_MODULE,
	.open	 = cdfinger_open,
	.unlocked_ioctl = cdfinger_ioctl,
	.release = cdfinger_release,
	.fasync  = cdfinger_async_fasync,
};

static struct miscdevice st_cdfinger_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &cdfinger_fops,
};

static int cdfinger_fb_notifier_callback(struct notifier_block* self,
                                        unsigned long event, void* data)
{
    struct fb_event* evdata = data;
    unsigned int blank;
    int retval = 0;

    if (event != FB_EVENT_BLANK /* FB_EARLY_EVENT_BLANK */) {
        return 0;
    }
    blank = *(int*)evdata->data;
    switch (blank) {
        case FB_BLANK_UNBLANK:
		mutex_lock(&g_cdfingerfp_data->buf_lock);
		screen_status = 1;
		if (isInKeyMode == 0)
			cdfinger_async_report();
		mutex_unlock(&g_cdfingerfp_data->buf_lock);
		printk("sunlin==FB_BLANK_UNBLANK==\n");
            break;
        case FB_BLANK_POWERDOWN:
		mutex_lock(&g_cdfingerfp_data->buf_lock);
		screen_status = 0;
		if (isInKeyMode == 0)
			cdfinger_async_report();
		mutex_unlock(&g_cdfingerfp_data->buf_lock);
		printk("sunlin==FB_BLANK_POWERDOWN==\n");
            break;
        default:
            break;
    }

    return retval;
}

static int cdfinger_probe(struct platform_device *pdev)
{
	struct cdfingerfp_data *cdfingerdev= NULL;
	int status = -ENODEV;
	printk("cdfinger cdfinger_probe\n");
	status = misc_register(&st_cdfinger_dev);
	if (status) {
		CDFINGER_DBG("cdfinger misc register err%d\n",status);
		return -1;
	}

	cdfingerdev = kzalloc(sizeof(struct cdfingerfp_data),GFP_KERNEL);
	cdfingerdev->miscdev = &st_cdfinger_dev;
	cdfingerdev->cdfinger_dev = pdev;
	mutex_init(&cdfingerdev->buf_lock);
	wake_lock_init(&cdfingerdev->cdfinger_lock, WAKE_LOCK_SUSPEND, "cdfinger wakelock");
	status=cdfinger_parse_dts(&cdfingerdev->cdfinger_dev->dev, cdfingerdev);
	if (status != 0) {
		CDFINGER_DBG("cdfinger parse err %d\n",status);
		goto unregister_dev;
	}

	cdfingerdev->cdfinger_input = input_allocate_device();
	if(!cdfingerdev->cdfinger_input){
		CDFINGER_ERR("crate cdfinger_input faile!\n");
		goto unregister_dev;
	}
	__set_bit(EV_KEY,cdfingerdev->cdfinger_input->evbit);
	__set_bit(KEY_F11,cdfingerdev->cdfinger_input->keybit);
	__set_bit(KEY_HOME,cdfingerdev->cdfinger_input->keybit);
	cdfingerdev->cdfinger_input->id.bustype = BUS_HOST;
	cdfingerdev->cdfinger_input->name="cdfinger_input";

	if(input_register_device(cdfingerdev->cdfinger_input))
	{
	  input_free_device(cdfingerdev->cdfinger_input);
		goto unregister_dev;
	}

	cdfingerdev->notifier.notifier_call = cdfinger_fb_notifier_callback;
    	fb_register_client(&cdfingerdev->notifier);
	//cdfinger_power_on(cdfingerdev);
	//cdfinger_reset_gpio_init(cdfingerdev, 10);
	//cdfinger_eint_gpio_init(cdfingerdev);
	g_cdfingerfp_data = cdfingerdev;
	return 0;
unregister_dev:
	misc_deregister(&st_cdfinger_dev);
	return  status;
}


static const struct of_device_id cdfinger_of_match[] = {
	{ .compatible = "cdfinger,fingerprint", },
	{},
};

static const struct platform_device_id cdfinger_id[] = {
	{"cdfinger_fp", 0},
	{}
};

static struct platform_driver cdfinger_driver = {
	.driver = {
		.name = "cdfinger_fp",
		.of_match_table = cdfinger_of_match,
	},
	.id_table = cdfinger_id,
	.probe = cdfinger_probe,
};

static int __init cdfinger_fp_init(void)
{
	return platform_driver_register(&cdfinger_driver);
}

static void __exit cdfinger_fp_exit(void)
{
	platform_driver_unregister(&cdfinger_driver);
}

module_init(cdfinger_fp_init);

module_exit(cdfinger_fp_exit);

MODULE_DESCRIPTION("cdfinger spi Driver");
MODULE_AUTHOR("cdfinger@cdfinger.com");
MODULE_LICENSE("GPL");
MODULE_ALIAS("cdfinger");


