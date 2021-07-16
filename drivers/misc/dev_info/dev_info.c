#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/debugfs.h>
#include <linux/dev_info.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#define DEV_INFO_HEAD_NAME   "Device info :\n"

struct dev_info * dev_info_head = NULL;

struct platform_device * dev_info_dev = NULL;


static int dev_info_init_head(void)
{
	dev_info_head = (struct dev_info *)kmalloc(sizeof(struct dev_info),GFP_KERNEL);
	if (!dev_info_head) {
		pr_err("Fail to allocate device info head memory\n");
		return -1;
	}
	dev_info_head->name = (char *)kmalloc(sizeof(DEV_INFO_HEAD_NAME),GFP_KERNEL);
	if (!dev_info_head->name) {
		pr_err("Fail to allocate device info head name \n");
		return -1;
	}

	INIT_LIST_HEAD((struct list_head *)dev_info_head);
	return 0;
}




static int dev_info_add(char * name)
{
	struct dev_info * new_dev;
	new_dev=kmalloc(sizeof(struct dev_info),GFP_KERNEL);
	if (!new_dev) {
		pr_err("Fail to allocate device info add  memory\n");
		return -1;
	}
	new_dev->name=kmalloc(strlen(name)+1,GFP_KERNEL);
	if (!new_dev->name) {
		pr_err("Fail to allocate device info add name memory\n");
		return -1;
	}
	sprintf(new_dev->name,"%s\n",name);
	list_add((struct list_head *)new_dev,(struct list_head *)dev_info_head);
	return 0;
}


int dev_info_register(char *type,char * name)
{
	struct dev_info * new_dev;
	int ret;

	if(!dev_info_head) {
		ret=dev_info_init_head();
		if(ret) {
			pr_err( "%s() - ERROR: dev_info_init_head() failed.",  __func__);
			return -1;
		}
	}

	new_dev=kmalloc(sizeof(struct dev_info),GFP_KERNEL);
	if (!new_dev) {
		pr_err("Fail to allocate device info add  memory\n");
		return -1;
	}
	new_dev->name=kmalloc(strlen(name)+strlen(type)+2,GFP_KERNEL);
	if (!new_dev->name) {
		pr_err("Fail to allocate device info add name memory\n");
		return -1;
	}
	sprintf(new_dev->name,"%s:%s\n",type,name);
	//strcpy(new_dev->name, dename);
	list_add_tail((struct list_head *)new_dev,(struct list_head *)dev_info_head);
	return 0;
}

EXPORT_SYMBOL(dev_info_register);

static char * dev_info_get_all(void)
{
	struct dev_info * device;
	struct list_head *list_ptr;
	int str_len=0,i=0;
	char * dev_info_string=NULL;
	list_for_each(list_ptr, &(dev_info_head->list)) {
		device = list_entry(list_ptr, struct dev_info, list);
		str_len+=strlen(device->name)+1;
	}
	dev_info_string = (char *)kmalloc(str_len+1,GFP_KERNEL);
	if(!dev_info_string) {
		pr_err("Fail to allocate dev_info_string\n");
		return NULL;
	}
	list_for_each(list_ptr, &(dev_info_head->list)) {
		device = list_entry(list_ptr, struct dev_info, list);
		i+=sprintf(dev_info_string+i,"%s",device->name);
	}

	return dev_info_string;

}

static void dev_info_remove(void)
{

}

static ssize_t dev_info_name_show(struct device *dev,struct device_attribute *attr,char *buf)
{	
 	char *str_name=NULL;
	int count = 0;
	str_name=dev_info_get_all();
	if(!str_name) {
		pr_err("dev_info_get_all  Fail \n");
		count = sprintf(buf, "%s\n", "NULL");
		return count;
	}

	count = sprintf(buf, "%s\n", str_name);//return test  result
	kfree(str_name);
	return count;
}

static ssize_t dev_info_name_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	int ret;

	if(count > 512) {
		pr_err("dev_info_name_store  name too long >512 \n");
		return 0;
	}
	//buf[count]='\0';
	ret=dev_info_add((char *)buf);

	if(ret) {
		pr_err("dev_info_name_store dev_info_add  Fail \n");
		return 0;
	}

	return count;
}

static DEVICE_ATTR(name, 0660, dev_info_name_show, dev_info_name_store);

static struct attribute *dev_info_attributes[] ={

	&dev_attr_name.attr,
	NULL
};

static struct attribute_group dev_info_attribute_group = {

.attrs = dev_info_attributes

};

int dev_info_add_attribute(struct platform_device *dev)
{
    int err=0;
    err = sysfs_create_group(&dev->dev.kobj, &dev_info_attribute_group);
    if (err)
    {
        pr_err( "%s() - ERROR: sysfs_create_group() failed.",  __func__);
        sysfs_remove_group(&dev->dev.kobj, &dev_info_attribute_group);
        return -EIO;
    }
	
    return err;
}

static int dev_info_probe(struct platform_device *dev)
{
	int ret;
	dev_info_dev = dev;
	if(!dev_info_head) {
		ret=dev_info_init_head();
		if(ret) {
			pr_err( "%s() - ERROR: dev_info_init_head() failed.",  __func__);
			return -1;
		}
	}
	ret=dev_info_add_attribute(dev);
	if(ret) {
		pr_err( "%s() - ERROR: dev_info_add_attribute() failed.",  __func__);
		return -1;
	}

	return 0;
}

static struct platform_device dev_info_device= {
	.name = "hq_dev_info",
	.id = -1,
};

static struct platform_driver dev_info_driver = {
	.probe		= dev_info_probe,
	.driver		= {
	.name		= "hq_dev_info",
	.owner		= THIS_MODULE,
	},
};

static int __init dev_info_init(void)
{
	int ret;
	ret = platform_device_register(&dev_info_device);
	if(ret) {
		pr_err(" dev_info platform_device_register Fail !\n");
		return -1;
	}
	ret = platform_driver_register(&dev_info_driver);
	if(ret) {
		pr_err(" dev_info platform_driver_register Fail !\n");
		return -1;
	}

	return 0;
}
module_init(dev_info_init);

static void __exit dev_info_exit(void)
{
	platform_driver_unregister(&dev_info_driver);
	dev_info_remove();
}
module_exit(dev_info_exit);

MODULE_DESCRIPTION("HQ Device info platform driver");
MODULE_LICENSE("GPL v2");
