
#include <linux/interrupt.h>
#include "mtv_gpio.h"

  

extern irqreturn_t mtv_isr(int irq, void *param);

#if defined(CONFIG_ARCH_S5PV310)// for Hardkernel
#include <plat/gpio-cfg.h>
#include <mach/regs-gpio.h>

#define PORT_CFG_OUTPUT                 1
#define MTV_PWR_EN                      S5PV310_GPD0(3)         
#define MTV_PWR_EN_CFG_VAL              S3C_GPIO_SFN(PORT_CFG_OUTPUT)
#define RAONTV_IRQ_INT                  S5PV310_GPX0(2)

static inline int mtv_configure_gpio(void)
{
	if(gpio_request(MTV_PWR_EN, "MTV_PWR_EN"))		
		DMBMSG("MTV_PWR_EN Port request error!!!\n");
	else	
	{
		// MTV_EN
		s3c_gpio_cfgpin(MTV_PWR_EN, MTV_PWR_EN_CFG_VAL);
		s3c_gpio_setpull(MTV_PWR_EN, S3C_GPIO_PULL_NONE);
		gpio_direction_output(MTV_PWR_EN, 0); // power down
	}

	return 0;
}

#elif defined(CONFIG_ARCH_S5PV210)//for MV210
#include <plat/gpio-cfg.h>
#include <mach/regs-gpio.h>
#include <mach/gpio.h>

#define S5PV210_GPD0_PWM_TOUT_0         (0x1 << 0)
#define MTV_PWR_EN                      S5PV210_GPD0(0) 
#define MTV_PWR_EN_CFG_VAL              S5PV210_GPD0_PWM_TOUT_0 

#define RAONTV_IRQ_INT                  IRQ_EINT6
#ifdef gpio_to_irq
	#undef gpio_to_irq
	#define	gpio_to_irq(x)		(x)
#endif

static inline int mtv_configure_gpio(void)
{
#if 0
	// MTV_EN
	s3c_gpio_cfgpin(MTV_PWR_EN, MTV_PWR_EN_CFG_VAL);
	s3c_gpio_setpull(MTV_PWR_EN, S3C_GPIO_PULL_NONE);
	gpio_direction_output(MTV_PWR_EN, 0); // power down

#else
	if(gpio_request(MTV_PWR_EN, "MTV_PWR_EN")) {
		//DMBMSG("MTV_PWR_EN Port request error!!!\n");
		printk("MTV_PWR_EN Port request error!!!\n");
	}
	else	
	{
		// MTV_EN
		s3c_gpio_cfgpin(MTV_PWR_EN, MTV_PWR_EN_CFG_VAL);
		s3c_gpio_setpull(MTV_PWR_EN, S3C_GPIO_PULL_NONE);
		gpio_direction_output(MTV_PWR_EN, 0); // power down
	}
#endif
	return 0;
}

#elif defined(CONFIG_ARCH_S5PC1XX)//for c100  S5PC1XX_PA_SPI0
#include <plat/gpio-cfg.h>
#include <plat/regs-gpio.h>

//#define S5PV210_GPD0_PWM_TOUT_0         (0x1 << 0)
#define MTV_PWR_EN                      S5PC1XX_GPB(5) 
#define MTV_PWR_EN_CFG_VAL              1<<8 

#define RAONTV_IRQ_INT                  IRQ_EINT1
#ifdef gpio_to_irq
	#undef gpio_to_irq
	#define	gpio_to_irq(x)		(x)
#endif

static inline int mtv_configure_gpio(void)
{
	printk("MTV_PWR_EN:%d\n",MTV_PWR_EN);
	
	if(gpio_request(MTV_PWR_EN, "MTV_PWR_EN"))		
		DMBMSG("MTV_PWR_EN Port request error!!!\n");
	else	
	{
		// MTV_EN
		s3c_gpio_cfgpin(MTV_PWR_EN, MTV_PWR_EN_CFG_VAL);
		s3c_gpio_setpull(MTV_PWR_EN, S3C_GPIO_PULL_NONE);
		gpio_direction_output(MTV_PWR_EN, 0); // power down
	}

	return 0;
}
#elif defined(CONFIG_ARCH_AWXX)//for A13,A20,A23,A31S
#include <linux/gpio.h>
#include <mach/gpio.h>
#include <mach/sys_config.h> 
static u32 mtv_pwr_irq;
static u32 mtv_pwr_en;

int mtv_configure_board(void)
{
// configure EN pin
    script_item_u   item;
    script_item_value_type_e   type;

    type = script_get_item("mtv_para", "mtv_power_en", &item);
    if(SCIRPT_ITEM_VALUE_TYPE_PIO != type) {
        printk("[csq]script_get_item mtv_power_en err\n");
    }
    mtv_pwr_en = item.gpio.gpio;

    if(gpio_request_one(mtv_pwr_en,GPIOF_OUT_INIT_LOW,NULL)){
        printk("[csq]mtv_pwr_en pin set to output function failure!\n");
    }
	
    type = script_get_item("mtv_para", "mtv_irq", &item);
    if(SCIRPT_ITEM_VALUE_TYPE_PIO != type) {
        DMBMSG("[csq]script_get_item mtv_irq err\n");
    }
    mtv_pwr_irq = item.gpio.gpio;	
	


// configure board information	
	
	return 0;
}
int mtv_confiture_irq(void)
{
    int irq, ret;
    irq = gpio_to_irq(mtv_pwr_irq);
	if (IS_ERR_VALUE(irq)) {
        DMBMSG(" %s() line:%d map gpio [%d] to virq failed, errno = %d\n", __func__, __LINE__, mtv_pwr_irq, irq);
        return -ENODEV;
    }
	ret = request_irq(irq, mtv_isr,
	#ifdef RTV_INTR_POLARITY_LOW_ACTIVE
					IRQ_TYPE_EDGE_FALLING,
	#else
					IRQ_TYPE_EDGE_RISING,
	#endif
					RAONTV_DEV_NAME, NULL);
	if (ret != 0) {
		DMBERR("Failed to install irq (%d)\n", ret);
		goto err;
	}
	disable_irq(irq); /* Must disabled */  
err: 
    return irq;
}
int mtv_free_irq(int irq)
{
	free_irq(irq, NULL);
}
int mtv_disable_irq(int irq)
{
    disable_irq(irq);
    return 0;
}
int mtv_enable_irq(int irq)
{
	enable_irq(irq);
	return 0;
}
void mtv_poweron(int on)
{
   if(on)
   	 gpio_direction_output(mtv_pwr_en, 1);
   else
   	 gpio_direction_output(mtv_pwr_en, 0);
}

#elif defined(CONFIG_ARCH_MTKXXXX)//for mt6572,mt6575,mt6577,mt8377,mt8389,mt8312,mt8382
#include "cust_gpio_usage.h"
#include <cust_eint.h>
#include <mach/mt_gpio.h>

#define MTV_PWR_EN GPIO34
#define MTV_INT_PIN GPIO51
#define RAONTV_IRQ_INT 19

extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_set_hw_debounce ( unsigned int eint_num, unsigned int ms );
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);

#if 0
	static struct spi_board_info spi_board_info[] = {
    {
        .modalias = "mtvspi",
        .bus_num = 0,
        .chip_select = 0,
        .mode = SPI_MODE_0,
    },
	};
#endif	

int mtv_configure_board(void)
{
// configure EN pin
	//PWR Enable
	mt_set_gpio_mode(MTV_PWR_EN | 0x80000000, GPIO_MODE_00);
	mt_set_gpio_dir(MTV_PWR_EN | 0x80000000, GPIO_DIR_OUT);
	mt_set_gpio_pull_enable(MTV_PWR_EN | 0x80000000, true);
	mt_set_gpio_out(MTV_PWR_EN | 0x80000000, 0);
	
	//irq
	mt_eint_set_sens(RAONTV_IRQ_INT, 1);
	mt_eint_set_hw_debounce(RAONTV_IRQ_INT, 0);
	
	mt_set_gpio_mode(MTV_INT_PIN | 0x80000000, GPIO_MODE_00);
	mt_set_gpio_dir(MTV_INT_PIN | 0x80000000, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(MTV_INT_PIN | 0x80000000, 1);
    mt_set_gpio_pull_select(MTV_INT_PIN | 0x80000000, 1);	
	
	//SPI 
	mt_set_gpio_mode(GPIO_SPI_CS_PIN, GPIO_SPI_CS_PIN_M_SPI_CS);
	mt_set_gpio_pull_enable(GPIO_SPI_CS_PIN, GPIO_PULL_ENABLE);
	//mt_set_gpio_pull_select(GPIO_SPI_CS_PIN, GPIO_PULL_UP);
			
	mt_set_gpio_mode(GPIO_SPI_SCK_PIN, GPIO_SPI_SCK_PIN_M_SPI_CK);
	mt_set_gpio_pull_enable(GPIO_SPI_SCK_PIN, GPIO_PULL_ENABLE);
	//mt_set_gpio_pull_select(GPIO_SPI_SCK_PIN, GPIO_PULL_DOWN);
			
	mt_set_gpio_mode(GPIO_SPI_MISO_PIN, GPIO_SPI_MISO_PIN_M_SPI_MI);
	mt_set_gpio_pull_enable(GPIO_SPI_MISO_PIN, GPIO_PULL_ENABLE);
	//mt_set_gpio_pull_select(GPIO_SPI_MISO_PIN, GPIO_PULL_DOWN);
			
	mt_set_gpio_mode(GPIO_SPI_MOSI_PIN, GPIO_SPI_MOSI_PIN_M_SPI_MO);
	mt_set_gpio_pull_enable(GPIO_SPI_MOSI_PIN, GPIO_PULL_ENABLE);
	//mt_set_gpio_pull_select(GPIO_SPI_MOSI_PIN, GPIO_PULL_DOWN);

	
// configure board information
#if 0
	spi_register_board_info(spi_board_info,ARRAY_SIZE(spi_board_info));
#endif	
	return 0;
}
int mtv_confiture_irq(void)
{
    int irq=0;
	// irq register
	mt_eint_registration(RAONTV_IRQ_INT, CUST_EINTF_TRIGGER_FALLING , mtv_isr, 1);//CUST_EINT_POLARITY_LOW
	
	// disable irq
	mt_eint_mask(RAONTV_IRQ_INT); /* Must disabled */
    return irq;
} 
int mtv_free_irq(int irq)
{
	//free_irq(irq, NULL);
	//i++;
	return 0;
}
int mtv_disable_irq(int irq)
{
    mt_eint_mask(RAONTV_IRQ_INT);
    return 0;
}
int mtv_enable_irq(int irq)
{
	mt_eint_unmask(RAONTV_IRQ_INT);
	return 0;
}
void mtv_poweron(int on)
{
   if(on)
   {
		mt_set_gpio_out(MTV_PWR_EN | 0x80000000, GPIO_OUT_ONE);
	}
   else
	{
		mt_set_gpio_out(MTV_PWR_EN | 0x80000000, GPIO_OUT_ZERO);
	}	
}

#elif defined(CONFIG_ARCH_RKXXXX)//for rk2928,rk3066,rk3168,rk3028,rk3028a,rk3026
#include <mach/gpio.h>
#include <mach/iomux.h>

#define MTV_PWR_EN                      RK30_PIN3_PD5
#define RAONTV_IRQ_INT                  RK30_PIN0_PC3
#if 0
static struct spi_board_info spi_board_info[] = {
    {
        .modalias  = "mtvspi",
        .bus_num = 0,   //0 or 1
        .max_speed_hz  = 6 * 1000 * 1000,
        .chip_select   = 0,     
    },  
};
#endif

int mtv_configure_board(void)
{
// configure EN pin
    if(gpio_request(MTV_PWR_EN, "MTV_PWR_EN"))		
        DMBMSG("MTV_PWR_EN Port request error!!!\n");
    gpio_direction_output(MTV_PWR_EN, 0);
	gpio_set_value(MTV_PWR_EN, 0);
    iomux_set(SPI0_TXD);
    iomux_set(SPI0_RXD);
    iomux_set(SPI0_CLK);
    iomux_set(SPI0_CS0);

// configure board information	
#if 0	
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
#endif	

	
	return 0;
}
int mtv_confiture_irq(void)
{
    int irq, ret;
    irq = gpio_to_irq(RAONTV_IRQ_INT); 	
	ret = request_irq(irq, mtv_isr,
	#ifdef RTV_INTR_POLARITY_LOW_ACTIVE
					IRQ_TYPE_EDGE_FALLING,
	#else
					IRQ_TYPE_EDGE_RISING,
	#endif
					RAONTV_DEV_NAME, NULL);
	if (ret != 0) {
		DMBERR("Failed to install irq (%d)\n", ret);
		goto err;
	}
	disable_irq(irq); /* Must disabled */  
err: 
    return irq;
}
int mtv_free_irq(int irq)
{
	free_irq(irq, NULL);
}
int mtv_disable_irq(int irq)
{
    disable_irq(irq);
    return 0;
}
int mtv_enable_irq(int irq)
{
	enable_irq(irq);
	return 0;
}
void mtv_poweron(int on)
{
   if(on)
   	gpio_set_value(MTV_PWR_EN, 1);
   else
   	gpio_set_value(MTV_PWR_EN, 0);
}

#elif defined(CONFIG_ARCH_BCMXXXX)//for bcm21663

#define MTV_PWR_EN                      (0x09)
#define RAONTV_IRQ_INT                  (56)
#if 0
static struct spi_board_info spi_board_info[] __initdata = {
	{
	 .modalias = "mtvspi",	/* use spidev generic driver */
	 .max_speed_hz = 6000000,	/* use max speed */

	 .bus_num = 0,		/* framework bus number */
	 .chip_select = 0,	/* for each slave */
	 .platform_data = NULL,	/* no spi_driver specific */
	 .irq = 0,		/* IRQ for this device */
	 .mode = SPI_LOOP,	/* SPI mode 0 */
	 },
};
#endif
int mtv_configure_board(void)
{
// configure EN pin
    if(gpio_request(MTV_PWR_EN, "MTV_PWR_EN"))		
        DMBMSG("MTV_PWR_EN Port request error!!!\n");
    gpio_direction_output(MTV_PWR_EN, 0);
	gpio_set_value(MTV_PWR_EN, 0);


// configure board information	
#if 0	
	spi_register_board_info(spi_board_info,ARRAY_SIZE(spi_board_info));
#endif		
	
	return 0;
}
int mtv_confiture_irq(void)
{
    int irq, ret;
    irq = gpio_to_irq(RAONTV_IRQ_INT); 	
	ret = request_irq(irq, mtv_isr,
	#ifdef RTV_INTR_POLARITY_LOW_ACTIVE
					IRQ_TYPE_EDGE_FALLING,
	#else
					IRQ_TYPE_EDGE_RISING,
	#endif
					RAONTV_DEV_NAME, NULL);
	if (ret != 0) {
		DMBERR("Failed to install irq (%d)\n", ret);
		goto err;
	}
	disable_irq(irq); /* Must disabled */  
err: 
    return irq;
}
int mtv_free_irq(int irq)
{
	free_irq(irq, NULL);
}
int mtv_disable_irq(int irq)
{
    disable_irq(irq);
    return 0;
}
int mtv_enable_irq(int irq)
{
	enable_irq(irq);
	return 0;
}
void mtv_poweron(int on)
{
   if(on)
   	gpio_set_value(MTV_PWR_EN, 1);
   else
   	gpio_set_value(MTV_PWR_EN, 0);
}

#elif defined(CONFIG_ARCH_SPRDXXXX)//
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/gpio.h>
#include "mtv_ioctl.h"

#define MTV_PWR_EN                     234// 215
#define RAONTV_IRQ_INT                 235 // 216
#if 1
	static struct spi_board_info spi_board_info[] = {
    {
        .modalias = "mtvspi",
        .bus_num = 1,
        .chip_select = 0,
        .max_speed_hz = 6000000,
        .mode = SPI_MODE_0,
    },
	};
#endif
int mtv_configure_board(void)
{
// configure EN pin
    if(gpio_request(MTV_PWR_EN, "MTV_PWR_EN"))		
        printk("MTV_PWR_EN Port request error!!!\n");
    gpio_direction_output(MTV_PWR_EN, 0);
	gpio_set_value(MTV_PWR_EN, 0);


// configure board information	
#if 1	
	spi_register_board_info(spi_board_info,ARRAY_SIZE(spi_board_info));
#endif		
	
	return 0;
}
int mtv_confiture_irq(void)
{
    int irq, ret;
   gpio_request(RAONTV_IRQ_INT, "RAONTV IRQ");
   
   gpio_direction_input(RAONTV_IRQ_INT);
	
    irq = gpio_to_irq(RAONTV_IRQ_INT); 	

	printk("mtv_confiture_irq install irq (%d)\n", irq);
	ret = request_irq(irq, mtv_isr,
	#ifdef RTV_INTR_POLARITY_LOW_ACTIVE
					IRQF_TRIGGER_FALLING,
	#else
					IRQF_TRIGGER_RISING,
	#endif
					RAONTV_DEV_NAME, NULL);
	if (ret != 0) {
		printk("Failed to install irq (%d)\n", ret);
		goto err;
	}
	disable_irq(irq); /* Must disabled */  
err: 
    return irq;
}
int mtv_free_irq(int irq)
{
	free_irq(irq, NULL);
}
int mtv_disable_irq(int irq)
{
    disable_irq(irq);
    return 0;
}
int mtv_enable_irq(int irq)
{
	enable_irq(irq);
	return 0;
}
void mtv_poweron(int on)
{
   if(on)
   	gpio_set_value(MTV_PWR_EN, 1);
   else
   	gpio_set_value(MTV_PWR_EN, 0);
}

#elif defined(CONFIG_ARCH_QUALLXXXX)//for QUALCOMM MSM7227A,MSM8225A/Q
#define MTV_PWR_EN                      (27)
#define RAONTV_IRQ_INT                  MSM8625_INT_TSIF_IRQ

int mtv_configure_board(void)
{
// configure EN pin
    int rc = 0;
    rc = gpio_request(MTV_PWR_EN, "MTV_PWR_EN");
    if (rc < 0) {
        pr_err("%s: gpio_request---GPIO_SKU1_CAM_5MP_SHDN failed!", __func__);
		return rc;
    }

    rc = gpio_tlmm_config(GPIO_CFG(MTV_PWR_EN, 0,
            GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
            GPIO_CFG_2MA), GPIO_CFG_ENABLE);

    if (rc < 0) {
        pr_err("%s: unable to config MTV_PWR_EN!\n", __func__);
        gpio_free(MTV_PWR_EN);
		return rc;
    }

	gpio_direction_output(MTV_PWR_EN, 0); // power down
	
	return 0;
}
int mtv_confiture_irq(void)
{
    int irq, ret;
    irq = gpio_to_irq(RAONTV_IRQ_INT); 	
	ret = request_irq(irq, mtv_isr,
	#ifdef RTV_INTR_POLARITY_LOW_ACTIVE
					IRQ_TYPE_EDGE_FALLING,
	#else
					IRQ_TYPE_EDGE_RISING,
	#endif
					RAONTV_DEV_NAME, NULL);
	if (ret != 0) {
		DMBERR("Failed to install irq (%d)\n", ret);
		goto err;
	}
	disable_irq(irq); /* Must disabled */  
err: 
    return irq;
}
int mtv_free_irq(int irq)
{
	free_irq(irq, NULL);
}
int mtv_disable_irq(int irq)
{
    disable_irq(irq);
    return 0;
}
int mtv_enable_irq(int irq)
{
	enable_irq(irq);
	return 0;
}
void mtv_poweron(int on)
{
   if(on)
   	gpio_set_value(MTV_PWR_EN, 1);
   else
   	gpio_set_value(MTV_PWR_EN, 0);
}

#else
	#error "code not present"
#endif