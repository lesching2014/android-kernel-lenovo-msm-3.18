#ifndef SLSPI_H
#define SLSPI_H

#include <linux/types.h>
#include <linux/spi/spi.h>

#define SPI_CPHA		0x01
#define SPI_CPOL		0x02

#define SPI_MODE_0		(0|0)
#define SPI_MODE_1		(0|SPI_CPHA)
#define SPI_MODE_2		(SPI_CPOL|0)
#define SPI_MODE_3		(SPI_CPOL|SPI_CPHA)

#define SPI_CS_HIGH		0x04
#define SPI_LSB_FIRST		0x08
#define SPI_3WIRE		0x10
#define SPI_LOOP		0x20
#define SPI_NO_CS		0x40
#define SPI_READY		0x80
#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY)

#define SPI_IOC_MAGIC			'k'
#define SL_READ  0x00 
#define SL_WRITE 0xFF 

#define GSL_FP_RESET_ACTIVE "silead_active"
#define GSL_FP_RESET_DEACTIVE "silead_suspend"
#define GSL_FP_IRQ_ACTIVE "silead_irq"

//#define GSL_FP_POWER_CTRL
//#define GSL_FP_REGULATOR_POWER_CTRL
//#define GSL_FP_GPIO_POWER_CTRL

#ifdef  GSL_FP_POWER_CTRL
typedef union pwr_val {
	struct {
		int vdd_val;
		int vio_val;
	} rgltor_val;
	
	int gpio_val;
} PWR_VAL_U;
#ifdef  GSL_FP_REGULATOR_POWER_CTRL
#define GSL_FP_VDD_MIN_UV	0000000
#define GSL_FP_VDD_MAX_UV	3300000
#define GSL_FP_VIO_MIN_UV	1750000
#define GSL_FP_VIO_MAX_UV	1950000
#endif
#endif

struct spi_ioc_transfer {
	__u64		tx_buf;
	__u64		rx_buf;

	__u32		len;
	__u32		speed_hz;

	__u16		delay_usecs;
	__u8		bits_per_word;
	__u8		cs_change;
	__u32		pad;
};

/* not all platforms use <asm-generic/ioctl.h> or _IOC_TYPECHECK() ... */
#define SPI_MSGSIZE(N) \
	((((N)*(sizeof (struct spi_ioc_transfer))) < (1 << _IOC_SIZEBITS)) \
		? ((N)*(sizeof (struct spi_ioc_transfer))) : 0)
#define SPI_IOC_MESSAGE(N) _IOW(SPI_IOC_MAGIC, 0, char[SPI_MSGSIZE(N)])


#define SPI_SYNC_READ        _IOR(SPI_IOC_MAGIC, 10, __u32)
#define SPI_ASYNC_READ_PRE   _IOR(SPI_IOC_MAGIC, 30, __u32)
#define SPI_ASYNC_READ       _IOR(SPI_IOC_MAGIC, 40, __u32)
#define SPI_GET_BUFFER_SIZE  _IOR(SPI_IOC_MAGIC, 50, __u32)

/* Read / Write of SPI mode (SPI_MODE_0..SPI_MODE_3) */
#define SPI_IOC_RD_MODE			_IOR(SPI_IOC_MAGIC, 1, __u8)
#define SPI_IOC_WR_MODE			_IOW(SPI_IOC_MAGIC, 1, __u8)

/* Read / Write SPI bit justification */
#define SPI_IOC_RD_LSB_FIRST		_IOR(SPI_IOC_MAGIC, 2, __u8)
#define SPI_IOC_WR_LSB_FIRST		_IOW(SPI_IOC_MAGIC, 2, __u8)

/* Read / Write SPI device word length (1..N) */
#define SPI_IOC_RD_BITS_PER_WORD	_IOR(SPI_IOC_MAGIC, 3, __u8)
#define SPI_IOC_WR_BITS_PER_WORD	_IOW(SPI_IOC_MAGIC, 3, __u8)

/* Read / Write SPI device default max speed hz */
#define SPI_IOC_RD_MAX_SPEED_HZ		_IOR(SPI_IOC_MAGIC, 4, __u32)
#define SPI_IOC_WR_MAX_SPEED_HZ		_IOW(SPI_IOC_MAGIC, 4, __u32)

#define SPI_HW_RESET			_IOW(SPI_IOC_MAGIC, 70, __u32)
#define SPI_HW_SHUTDOWN			_IOW(SPI_IOC_MAGIC, 71, __u32)
#define SPI_CLOSE_CLOCK          _IOW(SPI_IOC_MAGIC, 72, __u32)
#define SPI_OPEN_CLOCK          _IOW(SPI_IOC_MAGIC, 73, __u32)
#define SPI_GET_KERNEL_INT_INFO _IOW(SPI_IOC_MAGIC, 74, __u32)

#define SPI_HW_POWEROFF         _IOW(SPI_IOC_MAGIC, 75, __u32)
#define SPI_HW_POWERON          _IOW(SPI_IOC_MAGIC, 76, __u32)
#define SPI_HW_POWER_GET_VAL	_IOW(SPI_IOC_MAGIC, 79, __u32)

#define SPI_HW_SET_APP_VER		_IOW(SPI_IOC_MAGIC, 77, __u32)
#define SPI_HW_IRQ_REQUEST		_IOW(SPI_IOC_MAGIC, 78, __u32)
#define SPI_HW_IRQ_ENBALE		_IOW(SPI_IOC_MAGIC, 80, __u32)
#define SPI_HW_FINGER_STATE_INFO	_IOW(SPI_IOC_MAGIC, 81, __u32)
#define SPI_HW_VIRTUAL_KEY_INFO     _IOW(SPI_IOC_MAGIC, 82, __u32)
#define SPI_HW_IRQ_REGISTER		_IOW(SPI_IOC_MAGIC, 85, __u32)
//////@ 2015 add by joker
#define SL_HEAD_SIZE 3
#define SL_PAGE_SIZE 128 
#define SL_ONE_FRAME_PAGES (110*118/SL_PAGE_SIZE)

#define SPI_SPEED			(2 * 1000 * 1000)
#define SPI_BITS				8
#define SPI_DELAY			0
#define SPI_BUF_SIZE	 4096	

#define SL_LOG_TAG "SLCODE"
#undef SL_KLOGD
#ifndef SL_KLOGD
#define SL_KLOGD(fmt,args...)	\
	printk(KERN_ERR SL_LOG_TAG "||%-40s ||%-6d "fmt"\n",__func__,__LINE__,##args)
#endif
#undef SL_KLOGE
#ifndef SL_KLOGE
#define SL_KLOGE(fmt,args...)	\
	printk(KERN_ALERT SL_LOG_TAG "||%-40s ||%-6d "fmt"\n",__func__,__LINE__,##args)
#endif
//#define LSB_TO_MSB
#define SL_INT_NAME "silead_int"


struct spidev_data {
    dev_t			devt;
    spinlock_t		spi_lock;
    struct GSL_DEV_SEL_device	*spi;
    struct list_head	device_entry;
    /* buffer is NULL unless this device is open (users > 0) */
    struct mutex		buf_lock;
    unsigned		users;
    u8			*buffer;
#ifdef SLPT
	struct clk * clk;
#endif
    struct work_struct work;
    struct work_struct irq_work;
    struct workqueue_struct *wqueue;
    u8    *mmap_buf;
    u8    *tx_mmap_buf;
    u8    *u_mmap_buf;
    u8    *k_mmap_buf;
    atomic_t frame_num;
    atomic_t is_opened;
    unsigned int max_buf_size;
    unsigned int max_frame_num;
    struct wake_lock wake_lock;
    int irq;
    struct device_node *irq_node;
    unsigned int hw_reset_gpio;
    unsigned int hw_int_gpio;
    atomic_t is_cal_mode;
    atomic_t is_suspend;
#ifdef  GSL_FP_POWER_CTRL
#ifdef GSL_FP_REGULATOR_POWER_CTRL 
    struct regulator *vio;
    struct regulator *vdd;
#elif defined(GSL_FP_GPIO_POWER_CTRL)
    unsigned int hw_pwr_gpio;
#endif
#endif
    struct pinctrl *fp_pinctrl;
    struct pinctrl_state *pinctrl_state_active;
    struct pinctrl_state *pinctrl_state_suspend;
    struct pinctrl_state *pinctrl_state_interrupt;
};

#ifndef SLPT
unsigned int spidev_read_reg(struct spidev_data *spidev, unsigned char reg);
int spidev_write_reg(struct spidev_data *spidev, unsigned int data, unsigned char reg);
void init_frame(struct spidev_data *spidev);
ssize_t spidev_sync(struct spidev_data *spidev, struct spi_message *message);
#endif


typedef struct sl_page{
    unsigned char head[SL_HEAD_SIZE];
    unsigned char data[SL_PAGE_SIZE];
}sl_page_t;

typedef struct sl_frame{
    sl_page_t pages[SL_ONE_FRAME_PAGES];
}sl_frame_t;
typedef struct sl_frames{
    struct sl_frame *frame;
    int is_alloced;
}sl_frames_t;

#endif /* SPIDEV_H */
