#ifndef __MTV_GPIO_H__
#define __MTV_GPIO_H__


#ifdef __cplusplus 
extern "C"{ 
#endif  

//#define CONFIG_ARCH_RKXXXX 1
//#define CONFIG_ARCH_MTKXXXX 1
//#define CONFIG_ARCH_AWXX 1
//#define CONFIG_ARCH_BCMXXXX 1
//#define CONFIG_ARCH_QUALLXXXX 1
#define CONFIG_ARCH_SPRDXXXX 1

int mtv_configure_board(void);
int mtv_confiture_irq(void);
int mtv_free_irq(int irq);
int mtv_disable_irq(int irq);
int mtv_enable_irq(int irq);
void mtv_poweron(int on);

#ifdef __cplusplus 
} 
#endif 

#endif /* __MTV_GPIO_H__*/
