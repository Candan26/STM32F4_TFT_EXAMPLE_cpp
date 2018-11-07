
#ifndef ILI9341_STM32_DRIVER_H
#define ILI9341_STM32_DRIVER_H
////////////////////////////////////////////////////////////////////////
/////////////HAL LIBRARIES ///////////////////////////////////
#include "main.h"
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
//////////////////////////////////////////////////////////////
/////////////CPP LIBRARIES ///////////////////////////////////

/*
	TFT Drivers pinout diagram 
	The image of TFT on data base , Device communicates with half slave spi therefore there are only SCK MOSI CS 
	TFT has different 8 pinout which are 
	LED-> GPIO OUTPUT(ENABLES BACKLIGHT LED ON DEVICE)
	SCK-> SPI SCK(CLOCK)
	SDA-> SPI MOSI(MASTER OUTPUT SLAVE INPUT)
	AO -> GPIO OUTPUT(DATA OR REGISTER PIN (SELECTS WHETHER DATA OR CONFIGURATION INFO) )
	RESET-> GPIO OUTPUT(HARD RESETS DEVICE)
	CS-> GPIO OUTPUT(SPI CHIP SELECT ACTIVE LOW)
	GND-> GND (GROUND)
	VCC-> 3.3 V POWER SUPPLY IF J1 IS SOLDERED 5V (ON SCREEN  THERE ARE INTERNAL REGULATOR FOR 3.3 V)

*/

#define ILI9341_SCREEN_HEIGHT 128 ///128 
#define ILI9341_SCREEN_WIDTH 	128///128

//SPI INSTANCE
#define HSPI_INSTANCE							&hspi1

//CHIP SELECT PIN AND PORT, STANDARD GPIO
#define LCD_CS_PORT								GPIOC
#define LCD_CS_PIN								CS_Pin

//DATA COMMAND PIN AND PORT, STANDARD GPIO
#define LCD_DC_PORT								GPIOC
#define LCD_DC_PIN								DC_Pin

//RESET PIN AND PORT, STANDARD GPIO
#define	LCD_RST_PORT							GPIOC
#define	LCD_RST_PIN								RST_Pin


#define BURST_MAX_SIZE 	500

#define BLACK       0x0000      
#define NAVY        0x000F      
#define DARKGREEN   0x03E0      
#define DARKCYAN    0x03EF      
#define MAROON      0x7800      
#define PURPLE      0x780F      
#define OLIVE       0x7BE0      
#define LIGHTGREY   0xC618      
#define DARKGREY    0x7BEF      
#define BLUE        0x001F      
#define GREEN       0x07E0      
#define CYAN        0x07FF      
#define RED         0xF800     
#define MAGENTA     0xF81F      
#define YELLOW      0xFFE0      
#define WHITE       0xFFFF      
#define ORANGE      0xFD20      
#define GREENYELLOW 0xAFE5     
#define PINK        0xF81F

#define SCREEN_VERTICAL_1			0
#define SCREEN_HORIZONTAL_1		1
#define SCREEN_VERTICAL_2			2
#define SCREEN_HORIZONTAL_2		3
#define SCREEN_VERTICAL_3 		4

////////////////////////////////////////////////
//////////////////TFT ADDRESS MAPS 
	#define TFT_ADDRESS_MEMORY_ACCESS_CONTROL 0x36
	#define TFT_ADDRESS_SOFTWARE_RESET 0x01
	#define TFT_ADDRESS_VCOM_CONTROL1	0xC5
	#define TFT_ADDRESS_VCOM_OFFSET_CONTROL	0xC7
	#define TFT_ADDRESS_INTERFACE_PIXEL_FORMAT 0x3A
	#define TFT_ADDRESS_FRAME_RATE_CONTROL 0xB1
	#define TFT_ADDRESS_GAM_R_SEL 0xF2
	#define TFT_ADDRESS_GAMA_CURVE_SEL 0x26
	#define TFT_ADDRESS_POSITIVE_GAMMA_CORRELATION_SETTING 0xE0
	#define TFT_ADDRESS_NEGATIVE_GAMMA_CORRELATION_SETTING 0xE1
	#define TFT_ADDRESS_SLEEP_OUT 0x11
	#define TFT_ADDRESS_DISPLAY_ON 0x29
	#define TFT_ADDRESS_POWER_CONTROL1 0xC0
	#define TFT_ADDRESS_POWER_CONTROL2 0xC1
	
//////////////////////////////////////////////////////
	#define TFT_SET_BIT7 0x80
	#define TFT_SET_BIT6 0x40
	#define TFT_SET_BIT5 0x20
	#define TFT_SET_BIT4 0x10
	#define TFT_SET_BIT3 0x08
	#define TFT_SET_BIT2 0x04
	#define TFT_SET_BIT1 0x02
	#define TFT_SET_BIT0 0x01
////////////////////////////////////////////////////////
/*
	Power Control 
	Set the GVDD and voltage
	param 1 is VRH[4:0] -> VDD
	param 2 is VC[2:0]  -> VC1
	VRH 0000 to 1111
	[0] 00000 refers 5V
	[1] 00001 refers 4.75V
	[2] 00010 refers 4.70V
	[3] 00011 refers 4.65
	...
	...
	[23]10111 refers 3.65V
	...
	...
	[31]11111 refers 3V

*/
#define TST_SET_VOLT_TO_4_65V 0x23
/*
gama curve set 
This command is used to select the desired Gamma curve for the current display. A maximum of 4 fixed gamma curves
can be selected. The curves are defined Gamma Curve Correction Power Supply Circuit. The curve is selected by setting
the appropriate bit in the parameter as described in the Table:
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||	
||GC[7..0]|																																								 ||
||--------|-----------------------|--------------------------------------------------------||
||01h 		|GC0										|	 Gamma Curve 1																				 ||
||--------|-----------------------|--------------------------------------------------------||
||02h   	|GC1                    |	 Gamma Curve 2								      									 ||
||--------|--------------------------------------------------------------------------------||		
||03h   	|GC2                    |	 Gamma Curve 3					      												 ||	 
||--------|--------------------------------------------------------------------------------||
||04h   	|GC3            				|	 Gamma Curve 4          									 						 ||																
||--------|-----------------------|--------------------------------------------------------||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
	

*/
#define TFT_SET_GC7 TFT_SET_BIT7
#define TFT_SET_GC6 TFT_SET_BIT6
#define TFT_SET_GC5 TFT_SET_BIT5
#define TFT_SET_GC4 TFT_SET_BIT4
#define TFT_SET_GC3 TFT_SET_BIT3
#define TFT_SET_GC2 TFT_SET_BIT2
#define TFT_SET_GC1 TFT_SET_BIT1
#define TFT_SET_GC0 TFT_SET_BIT0

///gama ray sel 
/*
GAM_R_SEL: Gamma adjustment E0h and E1h enable control
0: Disable (Default)
1: Enable
*/
#define TFT_GAMMA_R_DISABLE 0x00
#define TFT_GAMMA_R_ENABLE 0x01
//////////////////////////////////////////////////////
/*
Sets the division ratio for internal clocks of Normal mode at CPU interface mode.
DIVA[4:0]: division ratio for internal clocks when Normal mode.
VPA[5:0]: Vsync porch for internal clocks when Normal mode
Frmae_rate= 200kHz/(Line+VPA[5:0])(DIVA[4:0]+4)
(1) When GM=101(132*132)
In Normal mode, line=132, Default value DIVA[4:0]=17, VPA[5:0]=20, Frame rate=62.7Hz
(2) When GM=100(130*130)
In Normal mode, line=130, Default value DIVA[4:0]=17, VPA[5:0]=20, Frame rate=63.5Hz
(3) When GM=011(128*160)
In Normal mode, line=160, Default value DIVA[4:0]=14, VPA[5:0]=20, Frame rate=61.7Hz
(4) When GM=010(120*160)
In Normal mode, line=160, Default value DIVA[4:0]=14, VPA[5:0]=20, Frame rate=61.7Hz
(5) When GM=001(128*128)
In Normal mode, line=128, Default value DIVA[4:0]=17, VPA[5:0]=20, Frame rate=64.4Hz
(6) When GM=000(132*162)
In Normal mode, line=162, Default value DIVA[4:0]=14, VPA[5:0]=20, Frame rate=61Hz

*/
#define TFT_SET_DIVA4	TFT_SET_BIT4
#define TFT_SET_DIVA3	TFT_SET_BIT3
#define TFT_SET_DIVA2	TFT_SET_BIT2
#define TFT_SET_DIVA1	TFT_SET_BIT1
#define TFT_SET_DIVA0	TFT_SET_BIT0

#define TFT_SET_VPA5 TFT_SET_BIT5
#define TFT_SET_VPA4 TFT_SET_BIT4
#define TFT_SET_VPA3 TFT_SET_BIT3
#define TFT_SET_VPA2 TFT_SET_BIT2
#define TFT_SET_VPA1 TFT_SET_BIT1
#define TFT_SET_VPA0 TFT_SET_BIT0

//////////////////////////////////////////////////////
/*
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||	
||------|-----------------------|--------------------------------------------------------||
||D3 		|											  |	NOT USED ....																					 ||
||------|-----------------------|--------------------------------------------------------||
||IFPF2 |                       |		"011"=12 bit pixel																	 ||
||------|												|		"101"=16 bit pixel																	 ||		
||IFPF1 |Control Interface Color|		"110"=18 bit pixel																	 ||	 
||------|				Format					| 																											 ||
||IFPF0 |               				|	the others = not defined 									 						 ||																
||------|-----------------------|--------------------------------------------------------||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*/
///////////////////////////////////////////////////////
#define TFT_SET_IFPF2	0x04
#define TFT_SET_IFPF1	0x02
#define TFT_SET_IFPF0	0x01
//////////////////////////////////////////////////////	

	
	
//////////////////////////////////////////////////
	#define TFT_SET_VMF0 0x01
	#define TFT_SET_VMF1 0x02
	#define TFT_SET_VMF2 0x04
	#define TFT_SET_VMF3 0x08
	#define TFT_SET_VMF4 0x10
	#define TFT_SET_VMF5 0x20
	#define TFT_SET_VMF6 0x40
	#define TFT_SET_nVM  0x80
/*
Description:
If “VMH”+xd or “VML”+xd is less than 0d, it becomes 0d
If “VMH”+xd or “VML”+xd is large than 100d, it becomes 100d
VMF[5:0] are stored in NV memoy to contrast
-Select the VMF[6:0]value :0 ->VCOM offset value from NV memory
													:1 ->VCOM offset value in the VMF[6:0] registers
Restriction:	
-If this register not use the register need be reserved.
-To control the VCOM output voltage with VMF[5::0] command, nVM parameter should be set ‘1’
Default value: 0x40
	
*/
	


/////////////////////////////////////////////////
	#define TFT_SET_VCOMH62 0x3E
	/*
		VMH[6:0] = 0111110
		Hex:0x3E= Decimal:62
		VCOMH= 4.050
	*/
	#define TFT_SET_VCOMH40 0x28
		/*
		VML[6:0] = 0101000
		Hex:0x28= Decimal:40
		VCOMH= -1.500
	*/
	
	
	
/////////////////////////////////////////////////
	#define TFT_SET_RGB 0x08
	#define TFT_SET_MY 0x80
	#define TFT_SET_MX 0x40
	#define TFT_SET_MV 0x20
	#define TFT_SET_ML 0x10
/*
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
||---------------------------------------------------------------------------------------||
||MY |	Row Address Order		  |																													 ||
||----------------------------																													 ||
||MX |	Column Address Order  |	These 3 bits controls MPU to memory write/read direction.||
||----------------------------																													 ||		
||MV |	Page/Column Selection |																													 ||	 
||---------------------------------------------------------------------------------------||
||ML |	Vertical Order 				|	LCD Vertical refresh direction control									 ||																
||---------------------------------------------------------------------------------------||
||	 |  											|	Color selector switch control														 ||																	 
||RGB|	RGB/BGR Order					|	0=RGB color filter panel																 ||
||	 |												|	1=BGR color filter panel																 ||
||----------------------------|----------------------------------------------------------||
||	 |												| '1’=LCD Refresh right to left														 ||
||MH |Display data latch order|	‘0’=LCD Refresh left to right														 ||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*/


typedef enum {TFT_OK=0,TFT_ERROR=1,TFT_BUSY=2,TFT_TIMEOUT=3}TFTIli9341TypeDef;

class ILI9341 {

	
	public:
		
		ILI9341(){}
	protected:	
		void ILI9341_SPIInit(void);
		void ILI9341_Reset(void);
		void ILI9341_Enable(void);
		void ILI9341_Init(void);
		
	public:
		TFTIli9341TypeDef ILI9341_SPISend(unsigned char spi_data);
		TFTIli9341TypeDef ILI9341_WriteCommand(unsigned char command);
		TFTIli9341TypeDef ILI9341_WriteData(unsigned char data);
		TFTIli9341TypeDef ILI9341_SetAddress(unsigned short X1,unsigned short Y1, unsigned short X2, unsigned short Y2);
		TFTIli9341TypeDef ILI9341_SetRotation(unsigned char rotation);
		TFTIli9341TypeDef ILI9341_FillScrenn(unsigned short colour);
		TFTIli9341TypeDef ILI9341_DrawColour(unsigned short colour);
		TFTIli9341TypeDef ILI9341_DrawPixel(unsigned short X, unsigned short Y, unsigned short colour);
		TFTIli9341TypeDef ILI9341_DrawColourBurst(unsigned short colour, unsigned int size);
	
		TFTIli9341TypeDef ILI9341_DrawHorizontalLine(unsigned short X, unsigned short Y, unsigned short width,unsigned short colour);
		TFTIli9341TypeDef ILI9341_DrawVerticalLine(unsigned short X, unsigned short Y, unsigned short height,unsigned short colour);																						
		TFTIli9341TypeDef	ILI9341_DrawLine(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint32_t color);




};



#endif

