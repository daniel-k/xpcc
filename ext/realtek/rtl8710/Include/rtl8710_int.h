#ifndef _RTL8710_INT_H_
#define _RTL8710_INT_H_

typedef enum {
	NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                        */
	HardFault_IRQn              = -13,    /*!< 3 Cortex-M0 Hard Fault Interrupt                                */
	SVC_IRQn                    = -5,     /*!< 11 Cortex-M0 SV Call Interrupt                                  */
	PendSV_IRQn                 = -2,     /*!< 14 Cortex-M0 Pend SV Interrupt                                  */
	SysTick_IRQn                = -1,     /*!< 15 Cortex-M0 System Tick Interrupt                              */

	SYSTEM_ON_INT       = 0,
	WDG_INT             = 1,
	TIMER0_INT          = 2,
	TIMER1_INT          = 3,
	I2C3_INT            = 4,
	TIMER2_7_INT        = 5,
	SPI0_INT            = 6,
	GPIO_INT            = 7,
	UART0_INT           = 8,
	SPI_FLASH_INT       = 9,
	USB_OTG_INT         = 10,
	SDIO_HOST_INT       = 11,
	SDIO_DEVICE_INT     = 12,
	I2S0_PCM0_INT       = 13,
	I2S1_PCM1_INT       = 14,
	WL_DMA_INT          = 15,
	WL_PROTOCOL_INT     = 16,
	CRYPTO_INT          = 17,
	GMAC_INT	        = 18,
	PERIPHERAL_INT      = 19,
	GDMA0_CHANNEL0_INT  = 20,
	GDMA0_CHANNEL1_INT  = 21,
	GDMA0_CHANNEL2_INT  = 22,
	GDMA0_CHANNEL3_INT  = 23,
	GDMA0_CHANNEL4_INT  = 24,
	GDMA0_CHANNEL5_INT  = 25,
	GDMA1_CHANNEL0_INT  = 26,
	GDMA1_CHANNEL1_INT  = 27,
	GDMA1_CHANNEL2_INT  = 28,
	GDMA1_CHANNEL3_INT  = 29,
	GDMA1_CHANNEL4_INT  = 30,
	GDMA1_CHANNEL5_INT  = 31,
	I2C0_INT            = 64,
	I2C1_INT            = 65,
	I2C2_INT            = 66,
	SPI1_INT            = 72,
	SPI2_INT            = 73,
	UART1_INT           = 80,
	UART2_INT           = 81,
	LOG_UART_INT        = 88,
	ADC_INT             = 89,
	DAC0_INT            = 91,
	DAC1_INT            = 92,
	LP_EXTENSION_INT    = 93,
	PTA_TRX_INT         = 95,
	RXI300_INT          = 96,
	NFC_INT             = 97
} IRQn_Type;

#endif // _RTL8710_INT_H_

