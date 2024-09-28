/*-----------------------------------------------------------------------------*/
/* File Name : zyt_info.c
/* for kernel debug info
/* Author : wangming
/* Date : 2015-1-18
/*-----------------------------------------------------------------------------*/
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/device.h>
#include <asm/uaccess.h>

#include <linux/stat.h>

#define BUFFER_LEN 1024
static char zyt_info_buffer[BUFFER_LEN]={0};
void zyt_info(char* content)
{

	if(content == NULL) return;
	unsigned int content_len = strlen(content);
	unsigned int buffer_used = strlen(zyt_info_buffer);
	if((buffer_used + content_len) > BUFFER_LEN) return;
	strcat(zyt_info_buffer,content);
}

void zyt_info_s(char* c1){zyt_info(c1);zyt_info("\n");}
void zyt_info_s2(char* c1,char* c2){zyt_info(c1);zyt_info(c2);zyt_info("\n");}
void zyt_info_sx(char* c1,int x){char temp[50];sprintf(temp,"%s 0x%x",c1,x);zyt_info(temp);zyt_info("\n");}

extern char* zyt_get_lcm_info();
extern char* get_sensor_info(int sensor_id);/*sensor_drv_k.c*/
extern char * zyt_battery_info();
static int zyt_info_proc_show(struct seq_file *m, void *v) {
	seq_printf(m, "[LCD] : %s\n[MainSensor] : %s\n[SubSensor] : %s\n%s%s\n",
					zyt_get_lcm_info(),
					get_sensor_info(0),
					get_sensor_info(1),
					zyt_info_buffer,
					zyt_battery_info()
					);
	return 0;
}

static int zyt_info_proc_open(struct inode *inode, struct file *file) {
 return single_open(file, zyt_info_proc_show, NULL);
}

static const struct file_operations zyt_info_proc_fops = {
 .owner = THIS_MODULE,
 .open = zyt_info_proc_open,
 .read = seq_read,
 .llseek = seq_lseek,
 .release = single_release,
};


int zyt_adb_debug=0;  
module_param(zyt_adb_debug,int,S_IRUGO|S_IWUSR);


static int __init zyt_info_proc_init(void) {
 proc_create("zyt_info", 0, NULL, &zyt_info_proc_fops);
 return 0;
}

static void __exit zyt_info_proc_exit(void) {
 remove_proc_entry("zyt_info", NULL);
}

MODULE_LICENSE("GPL");
module_init(zyt_info_proc_init);
module_exit(zyt_info_proc_exit);
