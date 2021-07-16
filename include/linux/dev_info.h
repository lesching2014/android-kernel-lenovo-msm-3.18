#ifndef __DEV_INFO_H__

#define __DEV_INFO_H__
 

struct dev_info {
	struct list_head list;
	char * name;
};


extern int dev_info_register(char *type,char * name);
#endif
