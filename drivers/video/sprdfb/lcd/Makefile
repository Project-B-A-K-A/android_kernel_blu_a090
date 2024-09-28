#change by wangming for auto detect
zyt_LCD_LISTS := $(subst ",,$(CONFIG_ZYT_LCM_LIST))
obj-y += $(foreach item,$(zyt_LCD_LISTS),$(item).o)

obj-$(CONFIG_FB_LCD_DUMMY) += lcd_dummy.o
