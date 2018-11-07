//-----------------------------------
//	ILI9341 GFX library for STM32
//-----------------------------------
//
//	Very simple GFX library built upon ILI9342_STM32_Driver library.
//	Adds basic shapes, image and font drawing capabilities to ILI9341
//
//	Library is written for STM32 HAL library and supports STM32CUBEMX. To use the library with Cube software
//	you need to tick the box that generates peripheral initialization code in their own respective .c and .h file
//
//
//-----------------------------------
//	How to use this library
//-----------------------------------
//
//	-If using MCUs other than STM32F7 you will have to change the #include "stm32f7xx_hal.h" in the ILI9341_GFX.h to your respective .h file
//
//	If using "ILI9341_STM32_Driver" then all other prequisites to use the library have allready been met
//	Simply include the library and it is ready to be used
//
//-----------------------------------
#ifndef ILI9341_GFX_H
#define ILI9341_GFX_H



/////////////////////HAL LIBRARIES ///////////////////////////////////
#include "main.h"
#include "stm32f4xx_hal.h"


#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
//////////////////////////////////////////////////////////////////////
////////////////////CPP LIBRARIES 


//////////////////////////////////////////////////////////////////////
///////////////////USER DEFINED LIBRARIES ///////////////////////////
#include "ILI9341_STM32_Driver.h"





#define HORIZONTAL_IMAGE	0
#define VERTICAL_IMAGE		1

typedef enum {GFX_OK=0,GFX_ERROR=1,GFX_BUSY=2,GFX_TIMEOUT=3}GFXTypeDef;

class GFX:public ILI9341 {
	
	/* GFX is sublibrariy of ILI9341 driver */
	public:
		GFX(){};
	public:
		void TFT_Init(void);

		// #########################################################################
		// Draw circle segments
		// #########################################################################

		// x,y == coords of centre of circle
		// start_angle = 0 - 359
		// sub_angle   = 0 - 360 = subtended angle
		// r = radius
		// colour = 16 bit colour value

		int TFT_FillSegment(int x, int y, int start_angle, int sub_angle, int r, unsigned int colour);
		
		//Analog Meter is for analog driver 
		void TFT_AnalogMeter();
		
		// Update needle position
		// This function is blocking while needle moves, time depends on ms_delay
		// 10ms minimises needle flicker if text is drawn within needle sweep area
		// Smaller values OK if text not in sweep area, zero for instant movement but
		// does not look realistic... (note: 100 increments for full scale deflection)
		void TFT_PlotNeedle(int value, unsigned char ms_delay);
		// ring Meter is main ring function
		void TFT_RingMeter(int value, int vmin, int vmax, int x, int y, int r, char *units, unsigned char  scheme);
		// draw rainbow 
		unsigned int TFT_Rainbow(unsigned char value);
		// rectangle 
		void TFT_DrawRectangle(unsigned short X, unsigned short Y, unsigned short width,unsigned short height,unsigned short colour);
		// Triangle 
		void TFT_DrawTriangle( int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint32_t color);//// WILL BE TESTED
		void TFT_FillTriangle( int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint32_t color);
		// circle
		void TFT_DrawHollowCircle(unsigned short X, unsigned short Y, unsigned short radius, unsigned short colour);
		void TFT_DrawFilledCircle(unsigned short X, unsigned short Y, unsigned short radius, unsigned short colour);	
		// ellipce 
		void TFT_DrawEllipse(unsigned short x0, unsigned short y0, unsigned short rx, unsigned short ry,unsigned short color); //// WILL BE TESTED
		void TFT_FillEllipse(unsigned short x0, unsigned short y0, unsigned short rx, unsigned short ry,unsigned short color); //// WILL BE TESTED
		
		//rectangle 	
		void TFT_DrawHollowRectangleCoord(unsigned short X0, unsigned short Y0, unsigned short X1, unsigned short Y1, unsigned short colour);
		void TFT_DrawFilledRectangleCoord(unsigned short X0, unsigned short Y0, unsigned short X1, unsigned short Y1, unsigned short colour);
		void TFT_Draw_FilledRectangleSizeText(unsigned short  X0, unsigned short  Y0, unsigned short  size_X, unsigned short  size_Y, unsigned short  colour);	
	
		//USING CONVERTER: http://www.digole.com/tools/PicturetoC_Hex_converter.php
		//65K colour (2Bytes / Pixel)
		void TFT_Draw_Image(const char* image_array, unsigned char orientation);
		GFXTypeDef TFT_DrawChar(char character, unsigned char X, unsigned char Y, unsigned short colour, unsigned short size, unsigned short background_colour);
		GFXTypeDef TFT_DrawText(const char* text, unsigned char X, unsigned char Y, unsigned short colour, unsigned short size, unsigned short background_colour);

};















#endif
