#include "ILI9341_GFX.h"
#include "5x5_font.h"
#include "spi.h"
#include <stdlib.h>     /* abs */
#include <math.h>       /* cos */
///////////////////////////////////////////////////////////////////////////
//Defines 
#define DEG2RAD 0.0174532925
//////////////////////////////////////////////////////////////////////////
//Extern Definitions
extern volatile unsigned short LCD_HEIGHT; // orj on ili9341_stm32_driver cpp
extern volatile unsigned short LCD_WIDTH;
/////////////////////////////////////////////////////////////////////////////
template <typename T> static inline void
swap_coord(T& a, T& b) { T t = a; a = b; b = t; };
// Define meter size
float M_SIZE=0.667;
float ltx = 0;    // Saved x coord of bottom of needle
int value[6] = {0, 0, 0, 0, 0, 0};
int old_value[6] = { -1, -1, -1, -1, -1, -1};
int d = 0;
int old_analog =  -999; // Value last displayed
unsigned short osx = M_SIZE*120, osy = M_SIZE*120; // Saved x & y coords
unsigned int updateTime = 0;       // time for next update
unsigned short drawtri;
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//// class methods only 
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
char *dtostrf (double val, signed char width, unsigned char prec, char *sout) {
  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

int GFX:: TFT_FillSegment(int x, int y, int start_angle, int sub_angle, int r, unsigned int colour){

	// Calculate first pair of coordinates for segment start
  float sx = cos((start_angle - 90) * DEG2RAD);
  float sy = sin((start_angle - 90) * DEG2RAD);
  uint16_t x1 = sx * r + x;
  uint16_t y1 = sy * r + y;
  // Draw colour blocks every inc degrees
  for (int i = start_angle; i < start_angle + sub_angle; i++) {

    // Calculate pair of coordinates for segment end
    int x2 = cos((i + 1 - 90) * DEG2RAD) * r + x;
    int y2 = sin((i + 1 - 90) * DEG2RAD) * r + y;
    TFT_FillTriangle(x1, y1, x2, y2, x, y, colour);
    // Copy segment end to sgement start for next segment
    x1 = x2;
    y1 = y2;

	}
}
void GFX::TFT_PlotNeedle(int value, unsigned char ms_delay){
	
	 char buf[8]; dtostrf(value, 4, 0, buf);
	// drawRightString(buf, 33, M_SIZE*(119 - 20), 2);
	TFT_DrawText(buf,66, M_SIZE*(140),BLACK, 2,WHITE); // // Comment out to avoid font 4	
	
  if (value < -10) value = -10; // Limit value to emulate needle end stops
  if (value > 110) value = 110;
	
  // Move the needle until new value reached
  while (!(value == old_analog)) {
			if (old_analog < value) old_analog++;
			else old_analog--;

			if (ms_delay == 0) old_analog = value; // Update immediately if delay is 0

			float sdeg = map(old_analog, -10, 110, -150, -30); // Map value to angle
			// Calculate tip of needle coords
			float sx = cos(sdeg * 0.0174532925);
			float sy = sin(sdeg * 0.0174532925);

			// Calculate x delta of needle start (does not start at pivot point)
			float tx = tan((sdeg + 90) * 0.0174532925);

			// Erase old needle image
			ILI9341_DrawLine (M_SIZE*(120 + 24 * ltx) - 1, M_SIZE*(150 - 24), osx - 1, osy, WHITE);
			ILI9341_DrawLine(M_SIZE*(120 + 24 * ltx), M_SIZE*(150 - 24), osx, osy, WHITE);
			ILI9341_DrawLine(M_SIZE*(120 + 24 * ltx) + 1, M_SIZE*(150 - 24), osx + 1, osy, WHITE);

			// Re-plot text under needle 
			// tft.drawCentreString("%RH", M_SIZE*120, M_SIZE*75, 4); // // Comment out to avoid font 4
			
			TFT_DrawText("%RH", M_SIZE*30, M_SIZE*140,BLACK, 2,WHITE); // // Comment out to avoid font 4
			 
			// Store new needle end coords for next erase
			ltx = tx;
			osx = M_SIZE*(sx * 98 + 120);
			osy = M_SIZE*(sy * 98 + 150);

			// Draw the needle in the new postion, magenta makes needle a bit bolder
			// draws 3 lines to thicken needle
			ILI9341_DrawLine(M_SIZE*(120 + 24 * ltx) - 1, M_SIZE*(150 - 24), osx - 1, osy, RED);
			ILI9341_DrawLine(M_SIZE*(120 + 24 * ltx), M_SIZE*(150 - 24), osx, osy, MAGENTA);
			ILI9341_DrawLine(M_SIZE*(120 + 24 * ltx) + 1, M_SIZE*(150 - 24), osx + 1, osy, RED);

			// Slow needle down slightly as it approaches new postion
			if (std::abs((old_analog - value)) < 10) ms_delay += ms_delay / 5;
		
			// Wait before next update
			HAL_Delay(ms_delay);
	
	}
}
		
void GFX::TFT_AnalogMeter(){
// Meter outline
	TFT_DrawRectangle(0, 0, M_SIZE*120, M_SIZE*110,DARKGREY);
  TFT_DrawRectangle(0, 0, M_SIZE*120, M_SIZE*110, DARKGREY);
  TFT_DrawRectangle(1, M_SIZE*3, M_SIZE*234, M_SIZE*125, WHITE);

  //tft.setTextColor(TFT_BLACK);  // Text colour

  // Draw ticks every 5 degrees from -50 to +50 degrees (100 deg. FSD swing)
  for (int i = -50; i < 51; i += 5) {
    // Long scale tick length
    int tl = 15;

    // Coodinates of tick to draw
    float sx = cos((i - 90) * 0.0174532925);
    float sy = sin((i - 90) * 0.0174532925);
    uint16_t x0 = sx * (M_SIZE*100 + tl) + M_SIZE*120;
    uint16_t y0 = sy * (M_SIZE*100 + tl) + M_SIZE*150;
    uint16_t x1 = sx * M_SIZE*100 + M_SIZE*120;
    uint16_t y1 = sy * M_SIZE*100 + M_SIZE*150;

    // Coordinates of next tick for zone fill
    float sx2 = cos((i + 5 - 90) * 0.0174532925);
    float sy2 = sin((i + 5 - 90) * 0.0174532925);
    int x2 = sx2 * (M_SIZE*100 + tl) + M_SIZE*120;
    int y2 = sy2 * (M_SIZE*100 + tl) + M_SIZE*150;
    int x3 = sx2 * M_SIZE*100 + M_SIZE*120;
    int y3 = sy2 * M_SIZE*100 + M_SIZE*150;

    // Yellow zone limits
    //if (i >= -50 && i < 0) {
    //  tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_YELLOW);
    //  tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_YELLOW);
    //}

    // Green zone limits
    if (i >= 0 && i < 25) {
			TFT_FillTriangle(x0, y0, x1, y1, x2, y2, GREEN);
      TFT_FillTriangle(x0, y0, x1, y1, x2, y2, GREEN);
      TFT_FillTriangle(x1, y1, x2, y2, x3, y3, GREEN);
    }

    // Orange zone limits
    if (i >= 25 && i < 50) {
      TFT_FillTriangle(x0, y0, x1, y1, x2, y2, ORANGE);
      TFT_FillTriangle(x1, y1, x2, y2, x3, y3, ORANGE);
    }

    // Short scale tick length
    if (i % 25 != 0) tl = 8;

    // Recalculate coords incase tick lenght changed
    x0 = sx * (M_SIZE*100 + tl) + M_SIZE*120;
    y0 = sy * (M_SIZE*100 + tl) + M_SIZE*150;
    x1 = sx * M_SIZE*100 + M_SIZE*120;
    y1 = sy * M_SIZE*100 + M_SIZE*150;

    // Draw tick
		ILI9341_DrawLine(x0, y0, x1, y1, BLACK);
 
    // Check if labels should be drawn, with position tweaks
    if (i % 25 == 0) {
      // Calculate label positions
      x0 = sx * (M_SIZE*100 + tl + 10) + M_SIZE*120;
      y0 = sy * (M_SIZE*100 + tl + 10) + M_SIZE*150;
      switch (i / 25) {
						
		 
        case -2: TFT_DrawText("0", x0+4, y0-4,BLACK, 1,WHITE); break;
				
        case -1: TFT_DrawText("25", x0+2, y0,BLACK, 1,WHITE);  break;
				
        case 0 : TFT_DrawText("50", x0, y0,BLACK, 1,WHITE);  break;
				 
        case 1 : TFT_DrawText("75", x0, y0,BLACK, 1,WHITE); break;
				
        case 2 : TFT_DrawText("100", x0-2, y0-4,BLACK, 1,WHITE); break;
      }
    }

    // Now draw the arc of the scale
    sx = cos((i + 5 - 90) * 0.0174532925);
    sy = sin((i + 5 - 90) * 0.0174532925);
    x0 = sx * M_SIZE*100 + M_SIZE*120;
    y0 = sy * M_SIZE*100 + M_SIZE*150;
    // Draw scale arc, don't draw the last part
		
    if (i < 50) ILI9341_DrawLine(x0, y0, x1, y1, BLACK);
  }

	 TFT_DrawText("%RH", M_SIZE*(3 + 230 - 40), M_SIZE*(119 - 20),BLACK,2,WHITE);
	
	 TFT_DrawText("%RH", M_SIZE*120, M_SIZE*75,BLACK,4,WHITE);
	
	 TFT_DrawRectangle(1, M_SIZE*3, M_SIZE*236, M_SIZE*126, BLACK); // Draw bezel line

   TFT_PlotNeedle(0, 0); // Put meter needle at 0


}
		
unsigned int GFX::TFT_Rainbow(unsigned char value)
{
  // Value is expected to be in range 0-127
  // The value is converted to a spectrum colour from 0 = blue through to 127 = red

  unsigned int red = 0; // Red is the top 5 bits of a 16 bit colour value
  unsigned int green = 0;// Green is the middle 6 bits
  unsigned int blue = 0; // Blue is the bottom 5 bits

  unsigned char quadrant = value / 32;

  if (quadrant == 0) {
    blue = 31;
    green = 2 * (value % 32);
    red = 0;
  }
  if (quadrant == 1) {
    blue = 31 - (value % 32);
    green = 63;
    red = 0;
  }
  if (quadrant == 2) {
    blue = 0;
    green = 63;
    red = value % 32;
  }
  if (quadrant == 3) {
    blue = 0;
    green = 63 - 2*(value % 32);
    red = 31;
  }
  return (red << 11) + (green << 5) + blue;
}
void GFX::TFT_RingMeter(int value, int vmin, int vmax, int x, int y, int r, char *units, unsigned char  scheme)
{
  // Minimum value of r is about 52 before value text intrudes on ring
  // drawing the text first is an option
  
  x += r; y += r;   // Calculate coords of centre of ring

  int w = r / 4;    // Width of outer ring is 1/4 of radius
  
  int angle = 90;  // Half the sweep angle of meter (300 degrees)

  int text_colour = 0; // To hold the text colour
  int empty_colour = 0; // To hold the text colour
  int full_colour = 0; // To hold the text colour

  int v = map(value, vmin, vmax, -angle, angle); // Map the value to an angle v

  unsigned char seg = 2; // Segments are 5 degrees wide = 60 segments for 300 degrees
  unsigned char inc = 2; // Draw segments every 5 degrees, increase to 10 for segmented ring

  // Draw colour blocks every inc degrees
  for (int i = -angle; i < angle; i += inc) {

    // Choose colour from scheme
    int colour = 0;
    switch (scheme) {
      case 0:
							colour = RED; 
							empty_colour = RED;
							full_colour = RED;
			break; // Fixed colour
      
			case 1:
							colour = GREEN;
							empty_colour = GREEN;
							full_colour = GREEN;
			break; // Fixed colour
      
			case 2: 
							colour = BLUE; 
							empty_colour = BLUE;
							full_colour = BLUE;
				break; // Fixed colour
			
      case 3:
							colour = TFT_Rainbow(map(i, -angle, angle, 0, 127)); 
							full_colour = TFT_Rainbow(map(angle, -angle, angle, 0, 127));
							empty_colour = TFT_Rainbow(map(-angle, -angle, angle, 0, 127));
							break; // Full spectrum blue to red
			
      case 4: 
							colour = TFT_Rainbow(map(i, -angle, angle, 63, 127)); 
							full_colour = TFT_Rainbow(map(angle, -angle, angle, 63, 127));
							empty_colour = TFT_Rainbow(map(-angle, -angle, angle, 63, 127)); 
							break; // Green to red (high temperature etc)
      case 5: 
							colour = TFT_Rainbow(map(i, -angle, angle, 127, 63));  
							full_colour = TFT_Rainbow(map(angle, -angle, angle, 127, 63));
							empty_colour = TFT_Rainbow(map(-angle, -angle, angle, 127, 63));
							break; // Red to green (low battery etc)
      default: colour = BLUE; break; // Fixed colour
    }

    // Calculate pair of coordinates for segment start
    float sx = cos((i - 90) * 0.0174532925);
    float sy = sin((i - 90) * 0.0174532925);
    uint16_t x0 = sx * (r - w) + x;
    uint16_t y0 = sy * (r - w) + y;
    uint16_t x1 = sx * r + x;
    uint16_t y1 = sy * r + y;

    // Calculate pair of coordinates for segment end
    float sx2 = cos((i + seg - 90) * 0.0174532925);
    float sy2 = sin((i + seg - 90) * 0.0174532925);
    int x2 = sx2 * (r - w) + x;
    int y2 = sy2 * (r - w) + y;
    int x3 = sx2 * r + x;
    int y3 = sy2 * r + y;

    if (i < v) { // Fill in coloured segments with 2 triangles
      TFT_FillTriangle(x0, y0, x1, y1, x2, y2, colour);
      TFT_FillTriangle(x1, y1, x2, y2, x3, y3, colour);
      drawtri = 1;
    }
    else // Fill in blank segments
    {
      TFT_FillTriangle(x0, y0, x1, y1, x2, y2, DARKGREY);
      TFT_FillTriangle(x1, y1, x2, y2, x3, y3, DARKGREY);
      if (drawtri)
      {
        drawtri = 0;
				//TFT.fillTriangle(x0, y0, x1, y1, x2, y2, colour);
        text_colour = colour; // Save the last colour drawn
      }
    }
  }

  // Convert value to a string
 // char buf[10];
 //unsigned char len = 4; if (value > 999) len = 5;
//  dtostrf(value, len, 0, buf);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GFX::TFT_FillEllipse(unsigned short x0, unsigned short y0, unsigned short rx, unsigned short ry,unsigned short color){
  if (rx<2) return;
  if (ry<2) return;
  int32_t x, y;
  int32_t rx2 = rx * rx;
  int32_t ry2 = ry * ry;
  int32_t fx2 = 4 * rx2;
  int32_t fy2 = 4 * ry2;
  int32_t s;

  for (x = 0, y = ry, s = 2*ry2+rx2*(1-2*ry); ry2*x <= rx2*y; x++)
  {
    ILI9341_DrawHorizontalLine(x0 - x, y0 - y, x + x + 1, color);
    ILI9341_DrawHorizontalLine(x0 - x, y0 + y, x + x + 1, color);

    if (s >= 0)
    {
      s += fx2 * (1 - y);
      y--;
    }
    s += ry2 * ((4 * x) + 6);
  }

  for (x = rx, y = 0, s = 2*rx2+ry2*(1-2*rx); rx2*y <= ry2*x; y++)
  {
    ILI9341_DrawHorizontalLine(x0 - x, y0 - y, x + x + 1, color);
    ILI9341_DrawHorizontalLine(x0 - x, y0 + y, x + x + 1, color);

    if (s >= 0)
    {
      s += fy2 * (1 - x);
      x--;
    }
    s += rx2 * ((4 * y) + 6);
  }


}
void GFX::TFT_DrawEllipse(unsigned short x0, unsigned short y0, unsigned short rx, unsigned short ry,unsigned short color){
	if (rx<2) return;
  if (ry<2) return;
  int32_t x, y;
  int32_t rx2 = rx * rx;
  int32_t ry2 = ry * ry;
  int32_t fx2 = 4 * rx2;
  int32_t fy2 = 4 * ry2;
  int32_t s;
	for (x = 0, y = ry, s = 2*ry2+rx2*(1-2*ry); ry2*x <= rx2*y; x++)
		{
			// These are ordered to minimise coordinate changes in x or y
			// drawPixel can then send fewer bounding box commands
			ILI9341_DrawPixel(x0 + x, y0 + y, color);
			ILI9341_DrawPixel(x0 - x, y0 + y, color);
			ILI9341_DrawPixel(x0 - x, y0 - y, color);
			ILI9341_DrawPixel(x0 + x, y0 - y, color);
			if (s >= 0)
			{
				s += fx2 * (1 - y);
				y--;
			}
			s += ry2 * ((4 * x) + 6);
		}

		for (x = rx, y = 0, s = 2*rx2+ry2*(1-2*rx); rx2*y <= ry2*x; y++)
		{
			// These are ordered to minimise coordinate changes in x or y
			// drawPixel can then send fewer bounding box commands
			ILI9341_DrawPixel(x0 + x, y0 + y, color);
			ILI9341_DrawPixel(x0 - x, y0 + y, color);
			ILI9341_DrawPixel(x0 - x, y0 - y, color);
			ILI9341_DrawPixel(x0 + x, y0 - y, color);
			if (s >= 0)
			{
				s += fy2 * (1 - x);
				x--;
			}
			s += rx2 * ((4 * y) + 6);
		}
}

void GFX::TFT_DrawTriangle( int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint32_t color){

	ILI9341_DrawLine(x0, y0, x1, y1, color);
	ILI9341_DrawLine(x1, y1, x2, y2, color);
	ILI9341_DrawLine(x2, y2, x0, y0, color);

}
//DRAW RECTANGLE OF SET SIZE AND HEIGTH AT X and Y POSITION WITH CUSTOM COLOUR
//
//Rectangle is hollow. X and Y positions mark the upper left corner of rectangle
//As with all other draw calls x0 and y0 locations dependant on screen orientation
//
void GFX::TFT_DrawRectangle(unsigned short X, unsigned short Y, unsigned short width,unsigned short height,unsigned short colour){

	if((X >=LCD_WIDTH) || (Y >=LCD_HEIGHT)) return ;

	if((X+width-1)>=LCD_WIDTH)
	{
		width=LCD_WIDTH-X;
	}
	if((Y+height-1)>=LCD_HEIGHT)
	{
		height=LCD_HEIGHT-Y;
	}
	ILI9341_SetAddress(X, Y, X+width-1, Y+height-1);
	ILI9341_DrawColourBurst(colour, height*width);

}

void GFX::TFT_FillTriangle ( int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint32_t color){
	 int32_t a, b, y, last;
	if (y0 > y1) {
    swap_coord(y0, y1); swap_coord(x0, x1);
  }
  if (y1 > y2) {
    swap_coord(y2, y1); swap_coord(x2, x1);
  }
  if (y0 > y1) {
    swap_coord(y0, y1); swap_coord(x0, x1);
  }

  if (y0 == y2) { // Handle awkward all-on-same-line case as its own thing
    a = b = x0;
    if (x1 < a)      a = x1;
    else if (x1 > b) b = x1;
    if (x2 < a)      a = x2;
    else if (x2 > b) b = x2;
    ILI9341_DrawHorizontalLine(a, y0, b - a + 1, color);
    return;
	}
		  int32_t
  dx01 = x1 - x0,
  dy01 = y1 - y0,
  dx02 = x2 - x0,
  dy02 = y2 - y0,
  dx12 = x2 - x1,
  dy12 = y2 - y1,
  sa   = 0,
  sb   = 0;
	// For upper part of triangle, find scanline crossings for segments
  // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
  // is included here (and second loop will be skipped, avoiding a /0
  // error there), otherwise scanline y1 is skipped here and handled
  // in the second loop...which also avoids a /0 error here if y0=y1
  // (flat-topped triangle).
  if (y1 == y2) last = y1;  // Include y1 scanline
  else         last = y1 - 1; // Skip it

  for (y = y0; y <= last; y++) {
    a   = x0 + sa / dy01;
    b   = x0 + sb / dy02;
    sa += dx01;
    sb += dx02;

    if (a > b) swap_coord(a, b);
    ILI9341_DrawHorizontalLine(a, y, b - a + 1, color);
  }

  // For lower part of triangle, find scanline crossings for segments
  // 0-2 and 1-2.  This loop is skipped if y1=y2.
  sa = dx12 * (y - y1);
  sb = dx02 * (y - y0);
  for (; y <= y2; y++) {
    a   = x1 + sa / dy12;
    b   = x0 + sb / dy02;
    sa += dx12;
    sb += dx02;

    if (a > b) swap_coord(a, b);
    ILI9341_DrawHorizontalLine(a, y, b - a + 1, color);
  }	
		
		
		
 }

void GFX::TFT_Init(void){
		
		ILI9341_Init();
	}

/*Draw hollow circle at X,Y location with specified radius and colour. X and Y represent circles center */
void GFX::TFT_DrawHollowCircle(unsigned short X, unsigned short Y, unsigned short radius, unsigned short colour){
int x = radius-1;
    int y = 0;
    int dx = 1;
    int dy = 1;
    int err = dx - (radius << 1);

    while (x >= y)
    {
        ILI9341_DrawPixel(X + x, Y + y, colour);
        ILI9341_DrawPixel(X + y, Y + x, colour);
        ILI9341_DrawPixel(X - y, Y + x, colour);
        ILI9341_DrawPixel(X - x, Y + y, colour);
        ILI9341_DrawPixel(X - x, Y - y, colour);
        ILI9341_DrawPixel(X - y, Y - x, colour);
        ILI9341_DrawPixel(X + y, Y - x, colour);
        ILI9341_DrawPixel(X + x, Y - y, colour);

        if (err <= 0)
        {
            y++;
            err += dy;
            dy += 2;
        }
        if (err > 0)
        {
            x--;
            dx += 2;
            err += (-radius << 1) + dx;
        }
    }
}

/*Draw filled circle at X,Y location with specified radius and colour. X and Y represent circles center */
void GFX::TFT_DrawFilledCircle(unsigned short X, unsigned short Y, unsigned short radius, unsigned short colour){
		int x = radius;
    int y = 0;
    int xChange = 1 - (radius << 1);
    int yChange = 0;
    int radiusError = 0;

    while (x >= y)
    {
        for(int i = X - x; i <= X + x; i++)
        {
            ILI9341_DrawPixel(i, Y + y,colour);
            ILI9341_DrawPixel(i, Y - y,colour);
        }
        for(int i = X - y; i <= X + y; i++)
        {
            ILI9341_DrawPixel(i, Y + x,colour);
            ILI9341_DrawPixel(i, Y - x,colour);
        }

        y++;
        radiusError += yChange;
        yChange += 2;
        if(((radiusError << 1) + xChange) > 0)
        {
            x--;
            radiusError += xChange;
            xChange += 2;
        }
    }
		//Really slow implementation, will require future overhaul
		//TODO:	https://stackoverflow.com/questions/1201200/fast-algorithm-for-drawing-filled-circles	
}

/*Draw a hollow rectangle between positions X0,Y0 and X1,Y1 with specified colour*/
void GFX::TFT_DrawHollowRectangleCoord(unsigned short X0, unsigned short Y0, unsigned short X1, unsigned short Y1, unsigned short colour){

	uint16_t 	X_length = 0;
	uint16_t 	Y_length = 0;
	uint8_t		Negative_X = 0;
	uint8_t 	Negative_Y = 0;
	float 		Calc_Negative = 0;
	
		Calc_Negative = X1 - X0;
		if(Calc_Negative < 0)
			Negative_X = 1;
		Calc_Negative = 0;	
		Calc_Negative = Y1 - Y0;
		if(Calc_Negative < 0) 
			Negative_Y = 1;
		
		//DRAW HORIZONTAL!
		if(!Negative_X)
		{
			X_length = X1 - X0;		
		}
		else
		{
			X_length = X0 - X1;		
		}
		ILI9341_DrawHorizontalLine(X0, Y0, X_length, colour);
		ILI9341_DrawHorizontalLine(X0, Y1, X_length, colour);
		
		//DRAW VERTICAL!
		if(!Negative_Y)
		{
			Y_length = Y1 - Y0;		
		}
		else
		{
			Y_length = Y0 - Y1;		
		}
		ILI9341_DrawVerticalLine(X0, Y0, Y_length, colour);
		ILI9341_DrawVerticalLine(X1, Y0, Y_length, colour);
		
		if((X_length > 0)||(Y_length > 0)) 
		{
			ILI9341_DrawPixel(X1, Y1, colour);
		}
}
/*Draw a filled rectangle between positions X0,Y0 and X1,Y1 with specified colour*/
void GFX::TFT_DrawFilledRectangleCoord(unsigned short X0, unsigned short Y0, unsigned short X1, unsigned short Y1, unsigned short colour){

	uint16_t 	X_length = 0;
	uint16_t 	Y_length = 0;
	uint8_t		Negative_X = 0;
	uint8_t 	Negative_Y = 0;
	int32_t 	Calc_Negative = 0;
	
	uint16_t X0_true = 0;
	uint16_t Y0_true = 0;
	
		Calc_Negative = X1 - X0;
		if(Calc_Negative < 0) 
			Negative_X = 1;
		Calc_Negative = 0;
		
		Calc_Negative = Y1 - Y0;
		if(Calc_Negative < 0) 
			Negative_Y = 1;
				
		//DRAW HORIZONTAL!
		if(!Negative_X)
		{
			X_length = X1 - X0;
			X0_true = X0;
		}
		else
		{
			X_length = X0 - X1;
			X0_true = X1;
		}
		
		//DRAW VERTICAL!
		if(!Negative_Y)
		{
			Y_length = Y1 - Y0;
			Y0_true = Y0;		
		}
		else
		{
			Y_length = Y0 - Y1;
			Y0_true = Y1;	
		}
		
		TFT_DrawRectangle(X0_true, Y0_true, X_length, Y_length, colour);	
}
/*Draws a character (fonts imported from fonts.h) at X,Y location with specified font colour, size and Background colour*/
/*See fonts.h implementation of font on what is required for changing to a different font when switching fonts libraries*/
GFXTypeDef GFX::TFT_DrawChar(char character, unsigned char X, unsigned char Y, unsigned short colour, unsigned short size, unsigned short background_colour){

	uint8_t 	function_char;
    uint8_t 	i,j;
		
		function_char = character;
		
    if (function_char < ' ') {
        character = 0;
    } else {
        function_char -= 32;
		}
   	
		char temp[CHAR_WIDTH];
		for(uint8_t k = 0; k<CHAR_WIDTH; k++)
		{
		temp[k] = font[function_char][k];
		}
		
    // Draw pixels
		TFT_DrawRectangle(X, Y, CHAR_WIDTH*size, CHAR_HEIGHT*size,background_colour);
    for (j=0; j<CHAR_WIDTH; j++) {
        for (i=0; i<CHAR_HEIGHT; i++) {
            if (temp[j] & (1<<i)) {			
							if(size == 1)
							{
              ILI9341_DrawPixel(X+j, Y+i, colour);
							}
							else
							{
							TFT_DrawRectangle(X+(j*size), Y+(i*size), size, size, colour);
							}
            }						
        }
    }
	return GFX_OK;
}

/*Draws an array of characters (fonts imported from fonts.h) at X,Y location with specified font colour, size and Background colour*/
/*See fonts.h implementation of font on what is required for changing to a different font when switching fonts libraries*/
GFXTypeDef GFX::TFT_DrawText(const char* text, unsigned char X, unsigned char Y, unsigned short colour, unsigned short size, unsigned short background_colour){


    while (*text) {
        TFT_DrawChar(*text++, X, Y, colour, size, background_colour);
        X += CHAR_WIDTH*size;
    }
		return GFX_OK;
}
/*Draws a full screen picture from flash. Image converted from RGB .jpeg/other to C array using online converter*/
//USING CONVERTER: http://www.digole.com/tools/PicturetoC_Hex_converter.php
//65K colour (2Bytes / Pixel)
void GFX::TFT_Draw_Image(const char* image_array, unsigned char orientation){
	if(orientation == SCREEN_HORIZONTAL_1)
	{
		ILI9341_SetRotation(SCREEN_HORIZONTAL_1);
		ILI9341_SetAddress(0,0,ILI9341_SCREEN_WIDTH,ILI9341_SCREEN_HEIGHT);
			
		HAL_GPIO_WritePin(TFT_D_C_GPIO_Port, TFT_D_C_Pin, GPIO_PIN_SET);	
		HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);
		
		unsigned char Temp_small_buffer[BURST_MAX_SIZE];
		uint32_t counter = 0;
		for(uint32_t i = 0; i < ILI9341_SCREEN_WIDTH*ILI9341_SCREEN_HEIGHT*2/BURST_MAX_SIZE; i++)
		{			
				for(uint32_t k = 0; k< BURST_MAX_SIZE; k++)
				{
					Temp_small_buffer[k]	= image_array[counter+k];		
				}						
				HAL_SPI_Transmit(&hspi1, (unsigned char*)Temp_small_buffer, BURST_MAX_SIZE, 10);	
				counter += BURST_MAX_SIZE;			
		}
		HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);
	}
	else if(orientation == SCREEN_HORIZONTAL_2)
	{
		ILI9341_SetRotation(SCREEN_HORIZONTAL_2);
		ILI9341_SetAddress(0,0,ILI9341_SCREEN_WIDTH,ILI9341_SCREEN_HEIGHT);
			
		HAL_GPIO_WritePin(TFT_D_C_GPIO_Port, TFT_D_C_Pin, GPIO_PIN_SET);	
		HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);
		
		unsigned char Temp_small_buffer[BURST_MAX_SIZE];
		uint32_t counter = 0;
		for(uint32_t i = 0; i < ILI9341_SCREEN_WIDTH*ILI9341_SCREEN_HEIGHT*2/BURST_MAX_SIZE; i++)
		{			
				for(uint32_t k = 0; k< BURST_MAX_SIZE; k++)
				{
					Temp_small_buffer[k]	= image_array[counter+k];		
				}						
				HAL_SPI_Transmit(&hspi1, (unsigned char*)Temp_small_buffer, BURST_MAX_SIZE, 10);	
				counter += BURST_MAX_SIZE;			
		}
		HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);
	}
	else if(orientation == SCREEN_VERTICAL_2)
	{
		ILI9341_SetRotation(SCREEN_VERTICAL_2);
		ILI9341_SetAddress(0,0,ILI9341_SCREEN_HEIGHT,ILI9341_SCREEN_WIDTH);
			
		HAL_GPIO_WritePin(TFT_D_C_GPIO_Port, TFT_D_C_Pin, GPIO_PIN_SET);	
		HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);
		
		unsigned char Temp_small_buffer[BURST_MAX_SIZE];
		uint32_t counter = 0;
		for(uint32_t i = 0; i < ILI9341_SCREEN_WIDTH*ILI9341_SCREEN_HEIGHT*2/BURST_MAX_SIZE; i++)
		{			
				for(uint32_t k = 0; k< BURST_MAX_SIZE; k++)
				{
					Temp_small_buffer[k]	= image_array[counter+k];		
				}						
				HAL_SPI_Transmit(&hspi1, (unsigned char*)Temp_small_buffer, BURST_MAX_SIZE, 10);	
				counter += BURST_MAX_SIZE;			
		}
		HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);
	}
	else if(orientation == SCREEN_VERTICAL_1)
	{
		ILI9341_SetRotation(SCREEN_VERTICAL_1);
		ILI9341_SetAddress(0,0,ILI9341_SCREEN_HEIGHT,ILI9341_SCREEN_WIDTH);
			
		HAL_GPIO_WritePin(TFT_D_C_GPIO_Port, TFT_D_C_Pin, GPIO_PIN_SET);	
		HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);
		
		unsigned char Temp_small_buffer[BURST_MAX_SIZE];
		uint32_t counter = 0;
		for(uint32_t i = 0; i < ILI9341_SCREEN_WIDTH*ILI9341_SCREEN_HEIGHT*2/BURST_MAX_SIZE; i++)
		{			
				for(uint32_t k = 0; k< BURST_MAX_SIZE; k++)
				{
					Temp_small_buffer[k]	= image_array[counter+k];		
				}						
				HAL_SPI_Transmit(&hspi1, (unsigned char*)Temp_small_buffer, BURST_MAX_SIZE, 10);	
				counter += BURST_MAX_SIZE;			
		}
		HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);
	}


}


