# STM32F4_TFT_EXAMPLE_cpp
This project is for driving TFT screen which has ILI9341 driver. The code written on cpp language with stm32f4 micro controller. 
The code background created on CubeMx program which provided by ST company 
The micro controller is STM32F407VGTX LQFP100 
The program written in Cpp language with Keil-mVision IDE.
To able to write Cpp code on STM32Fxx micro controllers do following 

    1) Create a project on CubeMX program 
    2) Adjust necesarry peripherals and pinouts (UART, SPI, GPIO, TIMER etc)
    3) Compile project 
    4) For Cpp change main.c with main.cpp file 
    5) Add extra files with the .cpp extension such as ILI9341_GFX.cpp
    6) Compile project and run
NOT: For tft adjust SPI 1 Half-dublex Master 
The TFT pinout and the connection diagram as follows:
ON TFT 				ON STMF4-Discovery
------------------------------------------------------------------------------
1)	VCC                                                    Connect to 3.3 pin
2)	GND                                                   Connect to GND
3)	CS                                                       Connect to PC5 as GPIO_Output
4)	RESET                                                 Connect to PB0 as GPIO_Output
5)	AO                                                      Connect to PC4 as GPIO_Output
6)	SDA                                                    Connect to PA7 as SPI1_MOSI
7)	SCK                                                     Connect to PA5 as SPI1_SCK
8)	LED                                                     Connect to 3.3 pin 