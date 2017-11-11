/*
 * graphics.h
 *
 *  Created on: 26 Oct. 2017
 *      Author: tekker
 */

#ifndef MAIN_GRAPHICS_H_
#define MAIN_GRAPHICS_H_

// TODO: Refactor to seperate source file
// graphics code

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <byteswap.h>

#include <stddef.h>
#include "esp_log.h"
#include "esp_attr.h"

#include "framebuffer.h"

#include "DejaVuSans18.h"

// VGA color palette
#define VGA_BLACK       0x0000
#define VGA_WHITE       0xFFFF
#define VGA_RED         0xF800
#define VGA_GREEN       0x0400
#define VGA_BLUE        0x001F
#define VGA_SILVER      0xC618
#define VGA_GRAY        0x8410
#define VGA_MAROON      0x8000
#define VGA_YELLOW      0xFFE0
#define VGA_OLIVE       0x8400
#define VGA_LIME        0x07E0
#define VGA_AQUA        0x07FF
#define VGA_TEAL        0x0410
#define VGA_NAVY        0x0010
#define VGA_FUCHSIA     0xF81F
#define VGA_PURPLE      0x8010
#define VGA_TRANSPARENT 0xFFFFFFFF

#define BLACK           0x0000
#define WHITE           0xFFFF

uint16_t      textcolor, textbgcolor ;
bool _transparent = true;

static ip4_addr_t gfx_s_ip_addr;
static int gfx_debugDisplay = 0;

#define ILI_WIDTH 320
#define ILI_HEIGHT 240


// FRAMEBUFFER reference...
//static uint16_t                 *frameBuffer;

static const char* GFX_TAG = "espilicam_gfx";

typedef struct propFont
{
        uint8_t charCode;
        uint16_t adjYOffset;
        uint16_t width;
        uint16_t height;
        uint16_t xOffset;
        uint16_t xDelta;
        uint8_t* dataPtr;
} _propFont;


typedef struct current_font {
    uint8_t* font ;
    uint8_t  x_size ;
    uint8_t  y_size ;
    uint8_t  offset ;
    uint8_t  numchars ;
} _current_font ;

_current_font cfont ;

static void Init_UTFT_GFX_Fonts() {
    cfont.font     = NULL ;
    cfont.x_size   = 0 ;
    cfont.y_size   = 0 ;
    cfont.offset   = 0 ;
    cfont.numchars = 0 ;
    textcolor      = VGA_PURPLE;
    textbgcolor    = VGA_PURPLE;
}

static void setFont(uint8_t* font) {
    cfont.font     = font ;
    cfont.x_size   = cfont.font[0] ;
    cfont.y_size   = cfont.font[1] ;
    cfont.offset   = cfont.font[2] ;
    cfont.numchars = cfont.font[3];
}

static uint8_t* getFont()      { return cfont.font ; }
static uint8_t  getFontXsize() { return cfont.x_size ; }
static uint8_t  getFontYsize() { return cfont.y_size ; }

// from esp32-nes
#define U16x2toU32(m,l) ((((uint32_t)(l>>8|(l&0xFF)<<8))<<16)|(m>>8|(m&0xFF)<<8))


// 32-bit aligned byte array update with dual 16-bit elements
static inline uint32_t IRAM_ATTR update_pixels(uint32_t px2x_pixels, uint16_t pos0pixel, uint16_t pos1pixel, bool drawPos0, bool drawPos1) {
	uint32_t* newPixels = &px2x_pixels;
	uint16_t *pixels16 = (uint16_t*)&newPixels[0];

	if (drawPos0) {
		//pixels16[0] = pos0pixel;// pos0pixel;
		pixels16[0] = __bswap_16(pos0pixel);// pos0pixel;
	}
	if (drawPos1) {
		//pixels16[1] = pos1pixel;
		pixels16[1] = __bswap_16(pos1pixel);
	}
	return px2x_pixels;
}

// TODO: Refactor with framebuffer updates... Check 32-bit aligned access to FB etc.
static inline void IRAM_ATTR plot_pixel_in_fb(uint16_t x, uint16_t y, uint16_t clrid) {

	fb_context_t fbc_display;
	int width = camera_get_fb_width();
	int height =  camera_get_fb_height();

	int current_byte_pos = 0;
	int current_pixel_pos = 0;
	int current_line_pos = 0;
	int xmod = 0;

	current_pixel_pos = ( y % height ) * width;

	if (gfx_debugDisplay == 1)
		ESP_LOGI(GFX_TAG,"Call  framebuffer_pos %d for %d,%d",current_pixel_pos,x,y);

	uint32_t *fbl_i = NULL;


/*
	for (int z = 0; z <= current_pixel_pos; z = z + width)  {
		if (gfx_debugDisplay == 1)
		  ESP_LOGI(GFX_TAG,"Scanning  framebuffer_pos (%d - %d)",z,current_pixel_pos);


		fbl_i = ( uint32_t* )framebuffer_pos_32( &fbc_display, z );
	    if (fbl_i == NULL) ESP_LOGI(GFX_TAG,"NULL from fb_pos!!");
	    return;
	}
*/

	fbl_i = ( uint32_t* )framebuffer_pos_32( &fbc_display, current_pixel_pos );
	if (fbl_i == NULL) {
		ESP_LOGI(GFX_TAG,"NULL from fb_pos for %d",current_pixel_pos);
		return;
	}

	//if ((x % 2) || (x==0))
	if ((x % 2) == 0)
		xmod = 0;
	else
		xmod = 1; // ie, x = 1... we must wind back to 0

	//if (x & 1) xmod = 1;
	current_line_pos = ((x-xmod)) % width ;
	current_byte_pos = current_line_pos / 2;


	if (gfx_debugDisplay == 1)
	ESP_LOGI(GFX_TAG,"Access fbl %d at pos for %d for %d,%d",current_pixel_pos,current_byte_pos,x,y);

	uint32_t fbl = fbl_i[current_byte_pos];
	if (fbl != NULL) {
		//fbl_i[current_byte_pos] = update_pixels(fbl,clrid,clrid,true,true);
		if (xmod == 1)
			fbl_i[current_byte_pos] = update_pixels(fbl,0,clrid,false,true); // mod pixel 1
		else
			fbl_i[current_byte_pos] = update_pixels(fbl,clrid,0,true, false); // mod. pixel 0
	}
}


// code below will error out if called with anything but 0,0 index pos
// seems current segmented-framebuffer code can only be called consecutively from idx 0?
/*
static inline void IRAM_ATTR plot_pixel_in_fb(uint16_t x, uint16_t y, uint16_t clrid) {

	fb_context_t fbc_display;
	int width = camera_get_fb_width();
	int height =  camera_get_fb_height();

	int current_byte_pos = 0;
	int current_pixel_pos = 0;
	int current_line_pos = 0;
	int xmod = 0;

	current_pixel_pos = ( y % height ) * width;
	uint32_t *fbl_i = ( uint32_t* )framebuffer_pos( &fbc_display, current_pixel_pos );
	if (x % 1) xmod = 1; 	// ie, x = 1... we must wind back to 0
	current_line_pos = ((x-xmod)) % width ;
	current_byte_pos = current_line_pos / 2;

	//if (y > 200)
	if (gfx_debugDisplay == 1)
	ESP_LOGI(GFX_TAG,"Access fbl %d at pos for %d for %d,%d",current_pixel_pos,current_byte_pos,x,y);

	uint32_t fbl = fbl_i[current_byte_pos];
	if (fbl != NULL) {
		//fbl_i[current_byte_pos] = update_pixels(fbl,clrid,clrid,true,true);
		if (xmod == 1)
			fbl_i[current_byte_pos] = update_pixels(fbl,0,clrid,false,true); // mod pixel 1
		else
			fbl_i[current_byte_pos] = update_pixels(fbl,clrid,0,true, false); // mod. pixel 0
	}
}
*/

// basic pixel modification for non-segmented 32-bit framebuffer
/*
inline void plot_pixel_32bit(uint32_t* fb, int x, int y, uint16_t clrid)
{

    uint32_t *dst= &fb[(y*ILI_WIDTH+x)/2];  // Division by 2 due to u16/u32 pointer mismatch!
    if(x&1)
        *dst= (*dst& 0xFFFF) | (clrid<<16);    // odd pixel
    else
        *dst= (*dst&~0xFFFF) |  clrid;        // even pixel
}
*/



static inline void IRAM_ATTR drawPixelF( uint16_t xpos, uint16_t ypos, uint16_t color )
{

	plot_pixel_in_fb(xpos,ypos,color);

}

// low-level drawing of shapes as needed... draw lines etc...
// as needed..

#define ABS(x)	(x < 0 ? -x : x)
#define SIGN(x)	(x < 0 ? -1 : (x > 0 ? 1 : 0))

static void DrawLine(int x1,int y1,int x2,int y2, uint16_t color)
{
	int	dx,dy,sx,sy;
	int	accum;

	dx = x2 - x1;
	dy = y2 - y1;

	sx = SIGN(dx);
	sy = SIGN(dy);

	dx = ABS(dx);
	dy = ABS(dy);

	x2 += sx;
	y2 += sy;

	if(dx > dy)
	{
		accum = dx >> 1;
		do{
			drawPixelF(x1,y1,color);

			accum -= dy;
			if(accum < 0)
			{
				accum += dx;
				y1 += sy;
			}

			x1 += sx;
		}while(x1 != x2);
	}else{
		accum = dy >> 1;
		do{
			drawPixelF(x1,y1,color);

			accum -= dx;
			if(accum < 0)
			{
				accum += dy;
				x1 += sx;
			}

			y1 += sy;
		}while(y1 != y2);
	}
}



static void DrawEllipse(int left,int top,int right,int bottom, uint16_t color)
{
	int		a,b;
	int		x,y,a2,b2,S,T;
	int		two_a2,two_b2;
	int		four_a2,four_b2;
	int		temp;

	if(right < left)
	{
		temp = left;
		left = right;
		right = temp;
	}
	if(bottom < top)
	{
		temp = top;
		top = bottom;
		bottom = temp;
	}

	a = (right - left + 1) / 2;
	b = (bottom - top + 1) / 2;

	if(!a && !b)
	{
		drawPixelF(left,top,color); // Draw a single pixel
		return;
	}

	if(!b)
	{
		DrawLine(top,left,top,right,color); // Draw a horizontal line
		return;
	}

	if(!a)
	{
		DrawLine(left,top,left,bottom,color); // Draw a vertical line
		return;
	}

	a2 = a * a;
	b2 = b * b;
	two_a2 = a2 << 1;
	two_b2 = b2 << 1;
	four_a2 = a2 << 2;
	four_b2 = b2 << 2;
	x = 0;
	y = b;
	S = a2 * (1 - (b << 1)) + two_b2;
	T = b2 - two_a2 * ((b << 1) - 1);

	drawPixelF(right + x - a,bottom + y - b,color);
	drawPixelF(left - x + a,bottom + y - b,color);
	drawPixelF(left - x + a,top - y + b,color);
	drawPixelF(right + x - a,top - y + b,color);

	do{
		if(S < 0)
		{
			S += two_b2 * ((x << 1) + 3);
			T += four_b2 * (x + 1);
			x++;
		}else if(T < 0){
			S += two_b2 * ((x << 1) + 3) - four_a2 * (y - 1);
			T += four_b2 * (x + 1) - two_a2 * ((y << 1) - 3);
			x++;
			y--;
		}else{
			S -= four_a2 * (y - 1);
			T -= two_a2 * ((y << 1) - 3);
			y--;
		}
		drawPixelF(right + x - a,bottom + y - b,color);
		drawPixelF(left - x + a,bottom + y - b,color);
		drawPixelF(left - x + a,top - y + b,color);
		drawPixelF(right + x - a,top - y + b,color);
	}while(y > 0);
}



void centeredCircle ( int centerX , int centerY , int radius, uint16_t color )
{
  int cornerX = centerX - radius;
  int cornerY = centerY - radius;
  int diameter = radius * 2;
  DrawEllipse(cornerX , cornerY ,cornerX+diameter ,cornerY+diameter, color );
}


void bullsEye ( int xCenter , int yCenter , int radiusBullsEye )
{

  centeredCircle ( xCenter, yCenter, radiusBullsEye, VGA_RED );
  centeredCircle ( xCenter, yCenter, (int)((2.0/3)*radiusBullsEye),VGA_BLUE);
  centeredCircle ( xCenter, yCenter, (int)((1.0/3)*radiusBullsEye),VGA_GREEN);
}

void crossHair ( int xCenter, int yCenter , int radiusCrossHair )
{

  centeredCircle ( xCenter , yCenter , radiusCrossHair, VGA_BLACK );

  int topX = xCenter;
  int topY = yCenter - radiusCrossHair;
  int bottomX = xCenter;
  int bottomY = yCenter + radiusCrossHair;
  int rightX = xCenter + radiusCrossHair;
  int rightY = yCenter;
  int leftX = xCenter - radiusCrossHair;
  int leftY = yCenter;

  DrawLine(topX,topY , bottomX,bottomY, VGA_BLACK );
  DrawLine(leftX,leftY , rightX,rightY, VGA_BLACK );
  centeredCircle ( xCenter , yCenter , (int) ( radiusCrossHair * ( 1.0/4) ), VGA_BLACK);

  bullsEye(xCenter , yCenter, (int) ( radiusCrossHair * ( 1.0/8) ) );
}








static inline void IRAM_ATTR printFontChar(uint8_t c, int16_t x, int16_t y) {
    if (cfont.font == NULL) return ;

    uint16_t start = ((c - cfont.offset) * ((cfont.x_size / 8) * cfont.y_size)) + 4 ;
    bool     drawBG = (textbgcolor != textcolor) ;
    uint8_t  j, zz, ch, i ;

    for (j = 0 ; j < cfont.y_size ; j++) {
        for (zz = 0 ; zz < (cfont.x_size / 8) ; zz++) {
            ch = cfont.font[start + zz] ;
            for (i = 0 ; i < 8 ; i++) {
                if ((ch & (1 << (7 - i))) != 0) {
                    drawPixelF(x + i + (zz * 8), y + j, textcolor) ;

                } else if (drawBG) {
                    drawPixelF(x + i + (zz * 8), y + j, textbgcolor) ;
                }
            }
        }
        start += (cfont.x_size / 8) ;
    }
}

//  Glyph data for an individual character in the ttf font
static inline bool getCharPtr(uint8_t c, _propFont* fontChar)
{
    // propFont data is stored as cFont.font data with 0 width...
    uint8_t* tempPtr = cfont.font + 4; // point at data
    do
    {
        fontChar->charCode = *tempPtr++;
        fontChar->adjYOffset = *tempPtr++;
        fontChar->width = *tempPtr++;
        fontChar->height = *tempPtr++;
        fontChar->xOffset = *tempPtr++;
        fontChar->xOffset = fontChar->xOffset < 0x80 ? fontChar->xOffset : (0x100 - fontChar->xOffset);
        fontChar->xDelta = *tempPtr++;
        if (c != fontChar->charCode && fontChar->charCode != 0xFF)
        {
            if (fontChar->width != 0)
            {
                // packed bits
                tempPtr += (((fontChar->width * fontChar->height)-1) / 8) + 1;
            }
        }
    } while (c != fontChar->charCode && fontChar->charCode != 0xFF);

    fontChar->dataPtr = tempPtr;
    return (fontChar->charCode != 0xFF);
}


// returns the string width in pixels. Useful for positions strings on the screen.
static inline int getStringWidth(char* str)
{
    char* tempStrptr = str;

    // is it a fixed width font?
    if (cfont.x_size != 0)
    {
        return (strlen(str) * cfont.x_size);
    }
    else
    {
        // calculate the string width
        int strWidth = 0;
        while (*str != 0)
        {
            _propFont fontChar;
            bool found = getCharPtr(*str, &fontChar);
            if (found && *str == fontChar.charCode)
            {
                strWidth += fontChar.xDelta + 1;
            }
            str++;
        }

        return strWidth;
    }
}



/*
void plot_pixel_in_fb(uint16_t x, uint16_t y, uint16_t clrid) {

	fb_context_t fbc_display;
	int width = camera_get_fb_width();
	int height =  camera_get_fb_height();

	int current_byte_pos = 0;
	int current_pixel_pos = 0;
	int current_line_pos = 0;
	int xmod = 0;

	current_pixel_pos = ( y % height ) * width;
	//ESP_LOGI(GFX_TAG,"CALL framebuffer_pos  for %d",current_pixel_pos);
	uint32_t *fbl_i = ( uint32_t* )framebuffer_pos( &fbc_display, current_pixel_pos );
	// start of fb segment / address
	//ESP_LOGI(GFX_TAG,"Got fb at %p",fbl_i);

	if (x % 1) xmod = 1; 	// ie, x = 1... we must wind back to 0
			   	   	   	   // x = 5, we need to access pixel 4

	current_line_pos = ((x-xmod)) % width ;
	current_byte_pos = current_line_pos / 2;

	ESP_LOGI(GFX_TAG,"Access fbl %d at pos for %d for %d,%d",current_pixel_pos,current_byte_pos,x,y);

	uint32_t fbl = fbl_i[current_byte_pos];
	if (fbl != NULL) {
				if (xmod == 1)
			        // *fbl= (*fbl & 0xFFFF) | (clrid<<16);    // odd pixel
					fbl_i[current_byte_pos] = (fbl & 0xFFFF) | (clrid<<16);
			    else
			    	    fbl_i[current_byte_pos] = (fbl &~0xFFFF) |  clrid;
			        // *fbl= (*fbl &~0xFFFF) |  clrid;        // even pixel
	}

}
*/


/*
{
	uint32_t *fbl = ( uint32_t* )framebuffer_pos( &fbc_display, current_pixel_pos );
	if(x&1)
	        *fbl= (*fbl& 0xFFFF) | (clrid<<16);    // odd pixel
	    else
	        *fbl= (*fbl&~0xFFFF) |  clrid;        // even pixel

}
*/

static void fillRect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color) {
  uint16_t w = x1 - x0;
  uint16_t h = y1 - y0;
  uint16_t x = x0;
  uint16_t y = y0;
  if (x0 > x1) {
    x = x1;
    w = -w;
  }
  if (y0 > y1) {
    y = y1;
    h = -h;
  }

  //ESP_LOGI(GFX_TAG,"fillrect for %d,%d,%d,%d",x,y,w,h);

  for (uint16_t i=0; i<h; i++) {

    	  // TODO: DRAWPIXEL REDO THIS GOTTA BE SLOW AS AN ESP32 CAN BE
    	  for (uint16_t z=0; z<w; z++) {
    		plot_pixel_in_fb( x+z, y+i, color);
    	  }

  }

}

// print a ttf based character
static inline int printProportionalChar(uint8_t c, int x, int y)
{
    uint8_t i=0,j=0,ch=0;
    uint16_t temp;
    uint8_t *tempPtr;

    _propFont fontChar;
    if (!getCharPtr(c, &fontChar))
    {
        return 0;
    }

    // fill background
    //uint16_t fcolor = textcolor; //getColor();
    if (!_transparent)
    {
        int fontHeight = cfont.y_size; //getFontHeight();
        fillRect(x, y, x + fontChar.xDelta+1, y + fontHeight,textbgcolor);
    }

    tempPtr = fontChar.dataPtr;
    if (fontChar.width != 0)
    {
          uint8_t mask = 0x80;
          for (j=0; j < fontChar.height; j++)
          {
             //ch=pgm_read_uint8_t(tempPtr++);
             for (i=0; i < fontChar.width; i++)
             {
                if (((i + (j*fontChar.width)) % 8) == 0)
                {
                    mask = 0x80;
                    ch = *tempPtr++;
                }
                if ((ch & mask) !=0)
                {
                    drawPixelF(x+fontChar.xOffset+i,y+j+fontChar.adjYOffset,textcolor);
                }
                else
                {
                    //setPixel(bcolorr, bcolorg, bcolorb);
                }
                mask >>= 1;
             }
          }
    }
    return fontChar.xDelta;
}



static inline void printFont(char* st, int16_t x, int16_t y, int16_t deg) {
    int16_t stl = strlen(st) ;

    // handle wrapping over the left or bottom...

    for (int16_t i = 0 ; i < stl ; i++) {
        if (deg == 0) {
                    if (cfont.x_size == 0)
                    {
                        // each char has individual width
                        x += printProportionalChar(*st++, x, y)+1;
                    }
                    else
                    {
                        printFontChar(*st++, x + (i * cfont.x_size), y);
                    }
        } else {
            // unsupported at present
            //rotateChar(*st++, x, y, i, deg) ;
        }
    }
}

static inline void printFontCenterHorz(char* st, int16_t y, int16_t deg) {
	int textWidth = getStringWidth(st);
	int startX = (ILI_WIDTH/2)-(textWidth/2);

	// TODO: cut off text if too long?
	if (startX < 0) startX = 0;
	printFont(st,startX,y,deg);

}

static void drawOverlayGraphics(char* fpsString) {
	printFont(fpsString,4,4,0);

	int centerX = (ILI_WIDTH/2);
	int centerY = (ILI_HEIGHT/2);

	crossHair(centerX, centerY, 60);

	/*
	//printFontCenterHorz("ESP32 ESPILICAM",30,0);
	char msgStr[20] = "TESTITTETSSTSERTESTTETSTE";
	int i = 0;
	for (i = 0; i < strlen(fpsString); i++)
		msgStr[i] = fpsString[i];
	msgStr[i++] = "\0";
	//printFontCenterHorz(msgStr,10,4);
	printFont(msgStr,4,4,0);
	//printFont(fpsString,4,4,0);
	//printFontCenterHorz(fpsString,4,4);

	 */
}


static void displayHelpScreen( void )
{
   char msgStr[20];
   char msgStr2[20];
   //memset( frameBuffer, BLACK, 2 * WIDTH * HEIGHT );         /* Clear screen */


   //fillRect(0,0,320,240,VGA_BLACK);

   static char ip_str[13];
   sprintf(ip_str, IPSTR, IP2STR(&gfx_s_ip_addr));

   printFontCenterHorz("ESP32 ESPILICAM",10,0);
   printFontCenterHorz("OV7670-ILI9341",10+getFontYsize(),0);
   //printFont("ESP32 ESPILICAM",10,10,0);
   //printFont("OV7670-ILI9341",10,10+getFontYsize(),0);

   printFont("command console at:",5,100,0);
   sprintf(msgStr,"telnet %s",ip_str);
   printFont(msgStr,50, 100 + getFontYsize(),0);
   printFont("or stream at:",50, 100 + (2*getFontYsize()), 0);
   sprintf(msgStr2,"http://%s/stream",ip_str);
   printFont(msgStr2, 10, 100 + (3*getFontYsize()),0);

}

static void setupInitialGraphics() {
  //frameBuffer = (uint16_t*)currFbPtr;
  //memset( frameBuffer, BLACK, 2 * WIDTH * HEIGHT );   /* Clear screen */
  fillRect(0,0,320,240,VGA_WHITE);
  Init_UTFT_GFX_Fonts();
  setFont(&DejaVuSans18);
  int xsize = getFontXsize();
  ESP_LOGI(GFX_TAG, "GFX Initialized: Font xsize: %d", xsize);

}

#endif /* MAIN_GRAPHICS_H_ */
