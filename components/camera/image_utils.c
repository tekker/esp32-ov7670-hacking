#include <math.h>
#include <stdlib.h>
#include <byteswap.h>
#include <stdint.h>


#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

static inline uint8_t clamp(int n)
{
    n = n>255 ? 255 : n;
    return n<0 ? 0 : n;
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
static inline uint16_t ILI9341_color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

typedef struct {
  uint8_t r;       // percent
  uint8_t g;       // percent
  uint8_t b;       // percent
} rgb;

typedef struct {
  double h;       // angle in degrees
  double s;       // percent
  double v;       // percent
} hsv;
typedef struct {
  float h;       // angle in degrees
  float s;       // percent
  float v;       // percent
} hsvF;



static hsv rgb2hsv(rgb in);
static rgb rgb565to888(uint16_t  in);
static uint16_t rgb888to565(uint8_t r, uint8_t g, uint8_t b);
static hsv rgb888toHSB(uint8_t red, uint8_t green, uint8_t blue);
static rgb hsv2rgb888(hsv in);
static uint16_t hsv2rgb565_i(hsv in);

static hsv rgb888toHSB(uint8_t red, uint8_t green, uint8_t blue) {
  double r = red / 255.0f;
  double g = green / 255.0f;
  double b = blue / 255.0f;
  double max = max(max(r, g), b);
  double min = min(min(r, g), b);
  double delta = max - min;
  double hue = 0;
  double brightness = max;
  double saturation = max == 0 ? 0 : (max - min) / max;
  if (delta != 0) {
    if (r == max) {
      hue = (g - b) / delta;
    }
    else {
      if (g == max) {
        hue = 2 + (b - r) / delta;
      }
      else {
        hue = 4 + (r - g) / delta;
      }
    }
    hue *= 60;
    if (hue < 0) hue += 360;
  }
  hsv hsv1;
  hsv1.h = hue;
  hsv1.s = saturation;
  hsv1.v = brightness;

  return hsv1;


}
#define swapFloat(a, b) { float t = a; a = b; b = t; }

static void RGB2H(uint8_t red, uint8_t green, uint8_t blue,	float *h){

  float r = red / 255.0f;
  float g = green / 255.0f;
  float b = blue / 255.0f;
  float max = fmaxf(fmaxf(r, g), b);
  float min = fminf(fminf(r, g), b);
  float delta = max - min;
  if (delta != 0)
  {
    float hue;
    if (r == max)
    {
      hue = (g - b) / delta;
    }
    else
    {
      if (g == max)
      {
        hue = 2 + (b - r) / delta;
      }
      else
      {
        hue = 4 + (r - g) / delta;
      }
    }
    hue *= 60;
    if (hue < 0) hue += 360;
    *h = hue;
  }
  else
  {
    *h = 0;
  }
  //s = max == 0 ? 0 : (max - min) / max;
  //b = max;
}


static void RGB2HSV_old(float r, float g, float b,	float *h, float *s, float *v)
{
  // da http://lolengine.net/blog/2013/01/13/fast-rgb-to-hsv
  float K = 0.f;

  if (g < b)
  {
    swapFloat(g, b);
    K = -1.f;
  }

  if (r < g)
  {
    swapFloat(r, g);
    K = -2.f / 6.f - K;
  }

  float chroma = r - min(g, b);
  *h = fabs(K + (g - b) / (6.f * chroma + 1e-20f));
  *s = chroma / (r + 1e-20f);
  *v = r;
}

hsv rgb2hsv(rgb in)
{
  // da http://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
  hsv         out;
  double      min, max, delta;

  min = in.r < in.g ? in.r : in.g;
  min = min  < in.b ? min : in.b;

  max = in.r > in.g ? in.r : in.g;
  max = max  > in.b ? max : in.b;

  out.v = max;                                // v
  delta = max - min;
  if (delta < 0.00001)
  {
    out.s = 0;
    out.h = 0; // undefined, maybe nan?
    return out;
  }
  if (max > 0.0) { // NOTE: if Max is == 0, this divide would cause a crash
    out.s = (delta / max);                  // s
  }
  else {
    // if max is 0, then r = g = b = 0
    // s = 0, v is undefined
    out.s = 0.0;
    out.h = -1; //NAN;                            // its now undefined
    return out;
  }
  if (in.r >= max)                           // > is bogus, just keeps compilor happy
    out.h = (in.g - in.b) / delta;        // between yellow & magenta
  else
    if (in.g >= max)
      out.h = 2.0 + (in.b - in.r) / delta;  // between cyan & yellow
    else
      out.h = 4.0 + (in.r - in.g) / delta;  // between magenta & cyan

  out.h *= 60.0;                              // degrees

  if (out.h < 0.0)
    out.h += 360.0;

  return out;
}
uint16_t rgb888to565(uint8_t r, uint8_t g, uint8_t b) {
  //rrrrr ggg ggg bbbbb
  uint16_t rgb565=0;
  rgb565 =  ((r >> 3) << 11);
  rgb565 |= ((g >> 2) << 6);
  rgb565 |= (b >> 3);
  return rgb565;
}
static rgb rgb565to888(uint16_t  in) {
  /* da http://stackoverflow.com/questions/2442576/how-does-one-convert-16-bit-rgb565-to-24-bit-rgb888
  R8 = ( R5 * 527 + 23 ) >> 6;
  G8 = ( G6 * 259 + 33 ) >> 6;
  B8 = ( B5 * 527 + 23 ) >> 6;
  */
  rgb  out;
  uint16_t  ins = in;
  out.r = ((in) >> 3) * 255 / 31;
  out.g = (in >> 5 & 63) * 255 / 63;
  out.b = ((in) & 31) * 255 / 31;
  return out;
}

uint16_t hsv2rgb565(uint8_t y,uint8_t u, uint8_t v) {
  double      hh, p, q, t, ff;
  long        i;
  uint8_t         r,g,b;

  #define CLIP(X) ( (X) > 255 ? 255 : (X) < 0 ? 0 : X)
  #define C(Y) ( (Y) - 16 )
  #define D(U) ( (U) - 128 )
  #define E(V) ( (V) - 128 )
  #define YUV2R(Y, U, V) CLIP(( 298 * C(Y) + 409 * E(V) + 128) >> 8)
  #define YUV2G(Y, U, V) CLIP(( 298 * C(Y) - 100 * D(U) - 208 * E(V) + 128) >> 8)
  #define YUV2B(Y, U, V) CLIP(( 298 * C(Y) + 516 * D(U) + 128) >> 8)

  //r = y + 1.4075 * (v - 128);
  //g = y - 0.3455 * (u - 128) - (0.7169 * (v - 128));
  //b = y + 1.7790 * (u - 128);

  r = YUV2R(y, u, v);
  g = YUV2G(y, u, v);
  b = YUV2B(y, u, v);

  uint16_t rgb565 = 0;
  rgb565 = ((r >> 3) << 11);
  rgb565 |= ((g >> 2) << 6);
  rgb565 |= (b >> 3);
  return rgb565;
}

uint16_t hsv2rgb565_i(hsv in) {
  double      hh, p, q, t, ff;
  long        i;
  rgb         out;

  hh = in.h;
  if (hh >= 360.0) hh = 0.0;
  hh /= 60.0;
  i = (long)hh;
  ff = hh - i;
  p = in.v * (1.0 - in.s);
  q = in.v * (1.0 - (in.s * ff));
  t = in.v * (1.0 - (in.s * (1.0 - ff)));
  switch (i) {
  case 0:
    out.r = in.v;
    out.g = t;
    out.b = p;
    break;
  case 1:
    out.r = q;
    out.g = in.v;
    out.b = p;
    break;
  case 2:
    out.r = p;
    out.g = in.v;
    out.b = t;
    break;

  case 3:
    out.r = p;
    out.g = q;
    out.b = in.v;
    break;
  case 4:
    out.r = t;
    out.g = p;
    out.b = in.v;
    break;
  case 5:
  default:
    out.r = in.v;
    out.g = p;
    out.b = q;
    break;
  }
  uint16_t rgb565 = (((uint8_t)out.r >> 3) << 11) | (((uint8_t)out.g >> 2) << 5) | ((uint8_t)out.b & 0b00011111);
  return rgb565;

}
static rgb hsv2rgb888(hsv in)
{
  double      hh, p, q, t, ff;
  long        i;
  rgb         out;

  if (in.s <= 0.0) {       // < is bogus, just shuts up warnings
    out.r = in.v;
    out.g = in.v;
    out.b = in.v;
    return out;
  }
  hh = in.h;
  if (hh >= 360.0) hh = 0.0;
  hh /= 60.0;
  i = (long)hh;
  ff = hh - i;
  p = in.v * (1.0 - in.s);
  q = in.v * (1.0 - (in.s * ff));
  t = in.v * (1.0 - (in.s * (1.0 - ff)));

  switch (i) {
  case 0:
    out.r = in.v;
    out.g = t;
    out.b = p;
    break;
  case 1:
    out.r = q;
    out.g = in.v;
    out.b = p;
    break;
  case 2:
    out.r = p;
    out.g = in.v;
    out.b = t;
    break;

  case 3:
    out.r = p;
    out.g = q;
    out.b = in.v;
    break;
  case 4:
    out.r = t;
    out.g = p;
    out.b = in.v;
    break;
  case 5:
  default:
    out.r = in.v;
    out.g = p;
    out.b = q;
    break;
  }
  return out;
}


/* convert a YUV set to a rgb set - thanks to MartinS and
   http://www.efg2.com/lab/Graphics/Colors/YUV.htm */
static inline uint16_t yuvtorgb(int Y, int U, int V)
{
	int r, g, b;
	static short L1[256], L2[256], L3[256], L4[256], L5[256];
	static int initialised;

	if (!initialised) {
		int i;
		initialised=1;
		for (i=0;i<256;i++) {
			L1[i] = 1.164*(i-16);
			L2[i] = 1.596*(i-128);
			L3[i] = -0.813*(i-128);
			L4[i] = 2.018*(i-128);
			L5[i] = -0.391*(i-128);
		}
	}
#if 0
	r = 1.164*(Y-16) + 1.596*(V-128);
	g = 1.164*(Y-16) - 0.813*(U-128) - 0.391*(V-128);
	b = 1.164*(Y-16) + 2.018*(U-128);
#endif

	r = L1[Y] + L2[V];
	g = L1[Y] + L3[U] + L5[V];
	b = L1[Y] + L4[U];

	if (r < 0) r = 0;
	if (g < 0) g = 0;
	if (b < 0) b = 0;
	if (r > 255) r = 255;
	if (g > 255) g = 255;
	if (b > 255) b = 255;

  uint16_t ret = ILI9341_color565(r,g,b); //(((uint16_t)r>>3)<<11) | (((uint16_t)g>>2)<<5) | (((uint16_t)b>>3)<<0);
  return ret;


}

// actual logic from rawpixels.net
static inline uint8_t rawpix(int YY, int UU, int VV) {


        float R = YY + 1.402   *(VV-128);
        float G = YY - 0.34414 *(UU-128) - 0.71414*(VV-128);
        float B = YY + 1.772   *(UU-128);

        if (B<0) B=0; else if (B>255) B=255;
        if (G<0) G=0; else if (G>255) G=255;
        if (R<0) R=0; else if (R>255) R=255;

        uint16_t ret = ILI9341_color565(R,G,B);  //(((uint16_t)R>>3)<<11) | (((uint16_t)G>>2)<<5) | (((uint16_t)B>>3)<<0);
        return ret;
}





static void convertyuv422torgb565(unsigned char *inbuf,unsigned char *outbuf,int width,int height)
{
  int rows,cols,rowwidth;
  int y,u,v,r,g,b,rdif,invgdif,bdif;
  int size;
  unsigned char *YUVdata,*RGBdata;
  int YPOS,UPOS,VPOS;

  YUVdata = inbuf;
  RGBdata = outbuf;

  rowwidth = width>>1;
  size=width*height*2;
  YPOS=0;
  UPOS=YPOS + size/2;
  VPOS=UPOS + size/4;

  for(rows=0;rows<height;rows++)
  {
    for(cols=0;cols<width;cols++)
    {
		 u = YUVdata[UPOS] - 128;
		 v = YUVdata[VPOS] - 128;

		 rdif = v + ((v * 103) >> 8);
		 invgdif = ((u * 88) >> 8) +((v * 183) >> 8);
		 bdif = u +( (u*198) >> 8);

		 r = YUVdata[YPOS] + rdif;
		 g = YUVdata[YPOS] - invgdif;
		 b = YUVdata[YPOS] + bdif;
		 r=r>255?255:(r<0?0:r);
		 g=g>255?255:(g<0?0:g);
		 b=b>255?255:(b<0?0:b);

		 *(RGBdata++) =( ((g & 0x1C) << 3) | ( b >> 3) );
		 *(RGBdata++) =( (r & 0xF8) | ( g >> 5) );

		 YPOS++;

		 if(cols & 0x01)
		 {
		    UPOS++;
		    VPOS++;
		 }
    }
    if((rows & 0x01)== 0)
    {
		 UPOS -= rowwidth;
		 VPOS -= rowwidth;
    }
  }

}


static void Yuv2Rgb(int *yuv, int *rgb, int up, int low)
{
	rgb[0] = yuv[0] + ( (359 * yuv[2]) >> 8 ) - 179;		//r
	//rgb[0] = (359 * yuv[2]) >> 8;
	//rgb[0] += yuv[0] + 76;
	//if(rgb[0] > 0x1ff)
	//	rgb[0] -= 0x1ff;
	//rgb[0] -= 179;
   	rgb[1] = yuv[0] +   135 - ( (88 * yuv[1] + 183 * yuv[2]) >> 8 );	//g
   	rgb[2] = yuv[0] + ( (454 * yuv[1]) >> 8 ) - 227;	//b

	rgb[0] = (rgb[0] > up) ? up : ((rgb[0] < low) ? low : rgb[0]);
	rgb[1] = (rgb[1] > up) ? up : ((rgb[1] < low) ? low : rgb[1]);
	rgb[2] = (rgb[2] > up) ? up : ((rgb[2] < low) ? low : rgb[2]);
}



uint8_t reverseBits8(uint8_t b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}
/*
uint8_t reverseBits8(uint8_t in) {
  uint8_t b = ((in * 0x80200802ULL) & 0x0884422110ULL) * 0x0101010101ULL >> 32;
  return b;
}
*/


uint16_t reverseBits16(uint16_t v) {
// CHAR_BIT is the number of bits per byte (normally 8).
  uint8_t CHAR_BIT = 8; // this is usually defined and so unneeded
  uint16_t r = v; // r will be reversed bits of v; first get LSB of v
  int s = sizeof(v) * CHAR_BIT - 1; // extra shift needed at end
  for (v >>= 1; v; v >>= 1)
  {
    r <<= 1;
    r |= v & 1;
    s--;
  }
  r <<= s; // shift when v's highest bits are zero
  return r;

}
