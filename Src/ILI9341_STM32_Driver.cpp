#include "ILI9341_STM32_Driver.h"
#include <stdlib.h>     /* abs */
#define TFT_RST_Pin GPIO_PIN_0 //TFT_LITE_Pin orjinal pin
#define TFT_RST_GPIO_Port  GPIOB //TFT_LITE_GPIO_Port orjinal port 


volatile unsigned short LCD_HEIGHT=ILI9341_SCREEN_HEIGHT;
volatile unsigned short LCD_WIDTH=ILI9341_SCREEN_WIDTH;
template <typename T> static inline void
swap_coord(T& a, T& b) { T t = a; a = b; b = t; };

void ILI9341:: ILI9341_SPIInit(void){
	
	MX_SPI1_Init();																							//SPI INIT
	MX_GPIO_Init();																							//GPIO INIT
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);	//CS OFF

}
/*send data to device  TFT screen */
TFTIli9341TypeDef ILI9341:: ILI9341_SPISend(unsigned char spi_data){

return (TFTIli9341TypeDef)	HAL_SPI_Transmit(&hspi1, &spi_data, 1, 50);

}

/* Send command (char) to LCD */
TFTIli9341TypeDef ILI9341 :: ILI9341_WriteCommand(unsigned char command){

	// DC pin for changing data format to command mode to data mode 
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TFT_D_C_GPIO_Port, TFT_D_C_Pin, GPIO_PIN_RESET);	
	ILI9341_SPISend(command);
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);	
	return TFT_OK;
}
/* Send Data (char) to LCD */
TFTIli9341TypeDef ILI9341::ILI9341_WriteData(unsigned char data){

	HAL_GPIO_WritePin(TFT_D_C_GPIO_Port, TFT_D_C_Pin, GPIO_PIN_SET);	
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);
	ILI9341_SPISend(data);	
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);
	return TFT_OK;
}
/* Set Address - Location block - to draw into */
TFTIli9341TypeDef ILI9341::ILI9341_SetAddress(unsigned short X1,unsigned short Y1, unsigned short X2, unsigned short Y2){

ILI9341_WriteCommand(0x2A);
ILI9341_WriteData(X1>>8);
ILI9341_WriteData(X1);
ILI9341_WriteData(X2>>8);
ILI9341_WriteData(X2);

ILI9341_WriteCommand(0x2B);
ILI9341_WriteData(Y1>>8);
ILI9341_WriteData(Y1);
ILI9341_WriteData(Y2>>8);
ILI9341_WriteData(Y2);

ILI9341_WriteCommand(0x2C);
return TFT_OK;
}
/*HARDWARE RESET*/
void ILI9341::ILI9341_Reset(void)
{
HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_RESET);
HAL_Delay(200);
HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);
HAL_Delay(200);
HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_SET);	
}
/*Ser rotation of the screen - changes x0 and y0*/


TFTIli9341TypeDef ILI9341:: ILI9341_SetRotation(unsigned char  rotation) 
{

uint8_t screen_rotation = rotation;


	
ILI9341_WriteCommand(TFT_ADDRESS_MEMORY_ACCESS_CONTROL);
HAL_Delay(1);
	
switch(screen_rotation) 
	{
		case SCREEN_VERTICAL_1:
			ILI9341_WriteData(TFT_SET_MX|TFT_SET_RGB);
			LCD_WIDTH = ILI9341_SCREEN_HEIGHT;
			LCD_HEIGHT = ILI9341_SCREEN_WIDTH;
			break;
		case SCREEN_HORIZONTAL_1:
			ILI9341_WriteData(TFT_SET_MV|TFT_SET_RGB);
			LCD_WIDTH  = ILI9341_SCREEN_HEIGHT;
			LCD_HEIGHT = ILI9341_SCREEN_WIDTH;
			break;
		case SCREEN_VERTICAL_2:
			ILI9341_WriteData(TFT_SET_MY|TFT_SET_RGB);
			LCD_WIDTH  = ILI9341_SCREEN_HEIGHT;
			LCD_HEIGHT = ILI9341_SCREEN_WIDTH;
			break;
		case SCREEN_HORIZONTAL_2:
			ILI9341_WriteData(TFT_SET_MX|TFT_SET_MY|TFT_SET_MV|TFT_SET_RGB);
			LCD_WIDTH  = ILI9341_SCREEN_HEIGHT;
			LCD_HEIGHT = ILI9341_SCREEN_WIDTH;
			break;

		case SCREEN_VERTICAL_3:
		ILI9341_WriteData(TFT_SET_RGB);
		LCD_WIDTH  = ILI9341_SCREEN_HEIGHT;
		LCD_HEIGHT = ILI9341_SCREEN_WIDTH;
		break;
		
		default:
		break;
			//EXIT IF SCREEN ROTATION NOT VALID!
	}
	return TFT_OK;
}
/*Enable LCD display*/
void ILI9341::ILI9341_Enable(void)
{
HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_SET);
}

/*Initialize LCD display*/
void ILI9341::ILI9341_Init(void)
{

ILI9341_Enable();
ILI9341_SPIInit();
ILI9341_Reset();

//SOFTWARE RESET
ILI9341_WriteCommand(TFT_ADDRESS_SOFTWARE_RESET);
HAL_Delay(1000);
////////////////////////////////////////////////////////////////////////
///// optional power control doesn ot exist in ili9163.pdf
//POWER CONTROL A
ILI9341_WriteCommand(0xCB);
ILI9341_WriteData(0x39);
ILI9341_WriteData(0x2C);
ILI9341_WriteData(0x00);
ILI9341_WriteData(0x34);
ILI9341_WriteData(0x02);

//POWER CONTROL B
ILI9341_WriteCommand(0xCF);
ILI9341_WriteData(0x00);
ILI9341_WriteData(0xC1);
ILI9341_WriteData(0x30);

//DRIVER TIMING CONTROL A
ILI9341_WriteCommand(0xE8);
ILI9341_WriteData(0x85);
ILI9341_WriteData(0x00);
ILI9341_WriteData(0x78);

//DRIVER TIMING CONTROL B
ILI9341_WriteCommand(0xEA);
ILI9341_WriteData(0x00);
ILI9341_WriteData(0x00);

//POWER ON SEQUENCE CONTROL
ILI9341_WriteCommand(0xED);
ILI9341_WriteData(0x64);
ILI9341_WriteData(0x03);
ILI9341_WriteData(0x12);
ILI9341_WriteData(0x81);

//PUMP RATIO CONTROL
ILI9341_WriteCommand(0xF7);
ILI9341_WriteData(0x20);

//POWER CONTROL,VRH[5:0]
ILI9341_WriteCommand(TFT_ADDRESS_POWER_CONTROL1);
ILI9341_WriteData(TST_SET_VOLT_TO_4_65V);

//POWER CONTROL,SAP[2:0];BT[3:0]
ILI9341_WriteCommand(TFT_ADDRESS_POWER_CONTROL2);
ILI9341_WriteData(0x10); //Default values wirtten in datasheet
/////////////////////////////////////////////////////////////////
//VCM CONTROL
ILI9341_WriteCommand(TFT_ADDRESS_VCOM_CONTROL1);
ILI9341_WriteData(TFT_SET_VCOMH62);
ILI9341_WriteData(TFT_SET_VCOMH40);

//VCM CONTROL 2
ILI9341_WriteCommand(TFT_ADDRESS_VCOM_OFFSET_CONTROL);
ILI9341_WriteData(TFT_SET_nVM|TFT_SET_VMF2|TFT_SET_VMF1); // normally 0x86

//MEMORY ACCESS CONTROL
ILI9341_WriteCommand(TFT_ADDRESS_MEMORY_ACCESS_CONTROL);
ILI9341_WriteData(TFT_SET_MX|TFT_SET_RGB);

//PIXEL FORMAT
ILI9341_WriteCommand(TFT_ADDRESS_INTERFACE_PIXEL_FORMAT);
ILI9341_WriteData(TFT_SET_BIT6|TFT_SET_BIT4|TFT_SET_IFPF2|TFT_SET_IFPF0);

//FRAME RATIO CONTROL, STANDARD RGB COLOR
ILI9341_WriteCommand(TFT_ADDRESS_FRAME_RATE_CONTROL);
ILI9341_WriteData(0x00);
ILI9341_WriteData(TFT_SET_VPA4|TFT_SET_VPA3);

//DISPLAY FUNCTION CONTROL
ILI9341_WriteCommand(0xB6);
ILI9341_WriteData(0x08);
ILI9341_WriteData(0x82);
ILI9341_WriteData(0x27);

//3GAMMA FUNCTION DISABLE
ILI9341_WriteCommand(TFT_ADDRESS_GAM_R_SEL);
ILI9341_WriteData(TFT_GAMMA_R_DISABLE);

//GAMMA CURVE SELECTED
ILI9341_WriteCommand(TFT_ADDRESS_GAMA_CURVE_SEL);
ILI9341_WriteData(TFT_SET_GC0);

//POSITIVE GAMMA CORRECTION
ILI9341_WriteCommand(TFT_ADDRESS_POSITIVE_GAMMA_CORRELATION_SETTING);
ILI9341_WriteData(0x0F);
ILI9341_WriteData(0x31);
ILI9341_WriteData(0x2B);
ILI9341_WriteData(0x0C);
ILI9341_WriteData(0x0E); /// gamma positive correlations settings in default parmareters state in data sheet 
ILI9341_WriteData(0x08);
ILI9341_WriteData(0x4E);
ILI9341_WriteData(0xF1);
ILI9341_WriteData(0x37);
ILI9341_WriteData(0x07);
ILI9341_WriteData(0x10);
ILI9341_WriteData(0x03);
ILI9341_WriteData(0x0E);
ILI9341_WriteData(0x09);
ILI9341_WriteData(0x00);

//NEGATIVE GAMMA CORRECTION
ILI9341_WriteCommand(TFT_ADDRESS_NEGATIVE_GAMMA_CORRELATION_SETTING);
ILI9341_WriteData(0x00);
ILI9341_WriteData(0x0E);
ILI9341_WriteData(0x14);
ILI9341_WriteData(0x03);
ILI9341_WriteData(0x11);/// gamma negative correlations settings in default parmareters state in data sheet 
ILI9341_WriteData(0x07);
ILI9341_WriteData(0x31);
ILI9341_WriteData(0xC1);
ILI9341_WriteData(0x48);
ILI9341_WriteData(0x08);
ILI9341_WriteData(0x0F);
ILI9341_WriteData(0x0C);
ILI9341_WriteData(0x31);
ILI9341_WriteData(0x36);
ILI9341_WriteData(0x0F);


//EXIT SLEEP
ILI9341_WriteCommand(TFT_ADDRESS_SLEEP_OUT);
HAL_Delay(120);

//TURN ON DISPLAY
ILI9341_WriteCommand(TFT_ADDRESS_DISPLAY_ON);

//STARTING ROTATION
ILI9341_SetRotation(SCREEN_VERTICAL_1);
}


//INTERNAL FUNCTION OF LIBRARY, USAGE NOT RECOMENDED, USE Draw_Pixel INSTEAD
/*Sends single pixel colour information to LCD*/
TFTIli9341TypeDef ILI9341::ILI9341_DrawColour(unsigned short colour){

	//SENDS COLOUR
unsigned char TempBuffer[2] = {colour>>8, colour};	
HAL_GPIO_WritePin(TFT_D_C_GPIO_Port, TFT_D_C_Pin, GPIO_PIN_SET);	
HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);
HAL_SPI_Transmit(&hspi1, TempBuffer, 2, 50);	
HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);
return TFT_OK;
}
//INTERNAL FUNCTION OF LIBRARY
/*Sends block colour information to LCD*/
TFTIli9341TypeDef ILI9341::ILI9341_DrawColourBurst(unsigned short colour, unsigned int size){

//SENDS COLOUR
uint32_t Buffer_Size = 1;
	if((size*2) < BURST_MAX_SIZE)
	{
		Buffer_Size = size;
	}
	else
	{
		Buffer_Size = BURST_MAX_SIZE;
	}
		
	HAL_GPIO_WritePin(TFT_D_C_GPIO_Port, TFT_D_C_Pin, GPIO_PIN_SET);	
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);

	unsigned char chifted = 	colour>>8;;
	unsigned char burst_buffer[500];
	for(uint32_t j = 0; j < Buffer_Size; j+=2)
		{
			burst_buffer[j] = 	chifted;
			burst_buffer[j+1] = colour;
		}

	uint32_t Sending_Size = size*2;
	uint32_t Sending_in_Block = Sending_Size/Buffer_Size;
	uint32_t Remainder_from_block = Sending_Size%Buffer_Size;

	if(Sending_in_Block != 0)
	{
		for(uint32_t j = 0; j < (Sending_in_Block); j++)
			{
				HAL_SPI_Transmit(&hspi1, (unsigned char *)burst_buffer, Buffer_Size, 50);	
			}
	}

	//REMAINDER!	
	HAL_SPI_Transmit(&hspi1, (unsigned char *)burst_buffer, Remainder_from_block, 50);	
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);
	return TFT_OK;
}

//FILL THE ENTIRE SCREEN WITH SELECTED COLOUR (either #define-d ones or custom 16bit)
/*Sets address (entire screen) and Sends Height*Width ammount of colour information to LCD*/
TFTIli9341TypeDef ILI9341::ILI9341_FillScrenn(unsigned short colour){
	
	ILI9341_SetAddress(0,0,LCD_WIDTH,LCD_HEIGHT);	
	ILI9341_DrawColourBurst(colour, LCD_WIDTH*LCD_HEIGHT);	
	return TFT_OK;
}

//DRAW PIXEL AT XY POSITION WITH SELECTED COLOUR
//
//Location is dependant on screen orientation. x0 and y0 locations change with orientations.
//Using pixels to draw big simple structures is not recommended as it is really slow
//Try using either rectangles or lines if possible
//

TFTIli9341TypeDef ILI9341::ILI9341_DrawPixel(unsigned short X, unsigned short Y, unsigned short colour){

	if((X >=LCD_WIDTH) || (Y >=LCD_HEIGHT)) 
		return TFT_ERROR;	//OUT OF BOUNDS!
	
//ADDRESS
	HAL_GPIO_WritePin(TFT_D_C_GPIO_Port, TFT_D_C_Pin, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);
	ILI9341_SPISend(0x2A);
	HAL_GPIO_WritePin(TFT_D_C_GPIO_Port, TFT_D_C_Pin, GPIO_PIN_SET);	
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);		

//XDATA
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);	
	unsigned char Temp_Buffer[4] = {X>>8,X, (X+1)>>8, (X+1)};
	HAL_SPI_Transmit(&hspi1, Temp_Buffer, 4, 50);
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);

//ADDRESS
	HAL_GPIO_WritePin(TFT_D_C_GPIO_Port, TFT_D_C_Pin, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);	
	ILI9341_SPISend(0x2B);
	HAL_GPIO_WritePin(TFT_D_C_GPIO_Port, TFT_D_C_Pin, GPIO_PIN_SET);			
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);			

//YDATA
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);
	unsigned char Temp_Buffer1[4] = {Y>>8,Y, (Y+1)>>8, (Y+1)};
	HAL_SPI_Transmit(&hspi1, Temp_Buffer1, 4, 50);
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);

//ADDRESS	
	HAL_GPIO_WritePin(TFT_D_C_GPIO_Port, TFT_D_C_Pin, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);	
	ILI9341_SPISend(0x2C);
	HAL_GPIO_WritePin(TFT_D_C_GPIO_Port, TFT_D_C_Pin, GPIO_PIN_SET);			
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);			

//COLOUR	
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);
	unsigned char Temp_Buffer2[2] = {colour>>8, colour};
	HAL_SPI_Transmit(&hspi1, Temp_Buffer2, 2, 50);
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);
	return TFT_OK;

}



/////////// Draw line witohut vast
TFTIli9341TypeDef	ILI9341::ILI9341_DrawLine(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint32_t color){

	bool steep= abs(y1-y0)>abs(x1-x0);
	if(steep){
		swap_coord(x0,y0);
		swap_coord(x1,y1);
	}
  if (x0 > x1) {
    swap_coord(x0, x1);
    swap_coord(y0, y1);
  }

  int32_t dx = x1 - x0, dy = abs(y1 - y0);;

  int32_t err = dx >> 1, ystep = -1, xs = x0, dlen = 0;
	  if (y0 < y1) ystep = 1;

  // Split into steep and not steep for FastH/V separation
  if (steep) {
    for (; x0 <= x1; x0++) {
      dlen++;
      err -= dy;
      if (err < 0) {
        err += dx;
        if (dlen == 1) ILI9341_DrawPixel(y0, xs, color);
				else ILI9341_DrawVerticalLine(y0, xs, dlen, color);
        dlen = 0; y0 += ystep; xs = x0 + 1;
      }
    }
	    if (dlen) ILI9341_DrawVerticalLine(y0, xs, dlen, color);
  } else
  {
    for (; x0 <= x1; x0++) {
      dlen++;
      err -= dy;
      if (err < 0) {
        err += dx;
        if (dlen == 1) ILI9341_DrawPixel(xs, y0, color);
        else ILI9341_DrawHorizontalLine(xs, y0, dlen, color);
        dlen = 0; y0 += ystep; xs = x0 + 1;
      }
    }
    if (dlen) ILI9341_DrawHorizontalLine(xs, y0, dlen, color);
  }	
	return TFT_OK;
}




//DRAW LINE FROM X,Y LOCATION to X+Width,Y LOCATION
TFTIli9341TypeDef ILI9341::ILI9341_DrawHorizontalLine(unsigned short X, unsigned short Y, unsigned short width,unsigned short colour){


	if((X >=LCD_WIDTH) || (Y >=LCD_HEIGHT)||(width<1)) return TFT_ERROR;
	if((X+width-1)>=LCD_WIDTH){
		width=LCD_WIDTH-X;
	}
	ILI9341_SetAddress(X, Y, X+width-1, Y);
	ILI9341_DrawColourBurst(colour, width);
	return TFT_OK;
}

//DRAW LINE FROM X,Y LOCATION to X,Y+Height LOCATION
TFTIli9341TypeDef ILI9341::ILI9341_DrawVerticalLine(unsigned short X, unsigned short Y, unsigned short height,unsigned short colour){

	if((X >=LCD_WIDTH) || (Y >=LCD_HEIGHT)) return TFT_ERROR;
	if((Y+height-1)>=LCD_HEIGHT){
		height=LCD_HEIGHT-Y;
	}
	ILI9341_SetAddress(X, Y, X, Y+height-1);
	ILI9341_DrawColourBurst(colour, height);
	return TFT_OK;
}																					
	
