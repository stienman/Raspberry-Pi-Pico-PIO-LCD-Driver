#ifndef _PINOUT_H_
#define _PINOUT_H_



#define PIN_SDIO_CLK	5		//sdio mode only
#define PIN_LCD_DnC		8
#define PIN_LCD_CS		9		//active low
#define PIN_SPI_CLK		10		//	|
#define PIN_SPI_MOSI	11		//	 | -spi1
#define PIN_SPI_MISO	12		//	|
#define PIN_LCD_BL		13		//backlight enable: Active high, PWM-able
#define PIN_LCD_RESET	15		//active low
#define PIN_TOUCH_CS	16		//active low
#define PIN_TOUCH_IRQ	17
#define PIN_SDIO_CMD	18		//sdio mode only
#define PIN_SDIO_D0		19		//sdio mode only
#define PIN_SDIO_D1		20		//sdio mode only
#define PIN_SDIO_D2		21		//sdio mode only
#define PIN_SDIO_D3_CS	22		//sdio mode only as D3, active low as SD nCS
#define PIN_REG_MODE	23		//power supply mode (0 = better efficiency whic is especialyl noticeable at low loads, 1 = less ripple)
#define PIN_VBUS_SENSE	24		//vbus sense (Active high)
#define PIN_LED			25		//led - active high
#define PIN_VSYS_DIV3	29		//ADC: VSYS/3



#endif

