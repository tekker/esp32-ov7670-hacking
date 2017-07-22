#ifndef _BITMAP_H_
#define _BITMAP_H_
#include <stdint.h>

// http://www.dragonwins.com/domains/getteched/bmp/bmpfileformat.htm

/*
 * Constants for the compression field...
 */


#define BI_RGB       0             /* No compression - straight BGR data */
#define BI_RLE8      1             /* 8-bit run-length compression */
#define BI_RLE4      2             /* 4-bit run-length compression */
#define BI_BITFIELDS 3             /* RGB bitmap with RGB masks */

typedef struct __attribute__((packed, aligned(1))) {
    uint8_t signature[2];
    uint32_t filesize;
    uint32_t reserved;
    uint32_t fileoffset_to_pixelarray;
} fileheader;
typedef struct __attribute__((packed, aligned(1))) {
    uint32_t dibheadersize;
    uint32_t width;
    uint32_t height;
    uint16_t planes;
    uint16_t bitsperpixel;
    uint32_t compression;
    uint32_t imagesize;
    uint32_t ypixelpermeter;
    uint32_t xpixelpermeter;
    uint32_t numcolorspallette;
    uint32_t mostimpcolor;
} bitmapinfoheader;

// 40 byte header for standard RGB888 bitmaps
typedef struct {
    fileheader fileheader;
    bitmapinfoheader bitmapinfoheader;
} bitmap;

// 16-bit pixel bitmaps append a color table to the standard info header
typedef struct __attribute__((packed, aligned(1))) {
    uint32_t dibheadersize;
    uint32_t width;
    uint32_t height;
    uint16_t planes;
    uint16_t bitsperpixel;
    uint32_t compression;
    uint32_t imagesize;
    uint32_t ypixelpermeter;
    uint32_t xpixelpermeter;
    uint32_t numcolorspallette;
    uint32_t mostimpcolor;
    uint32_t BF1;  // bitfield rgb mask 1 (g)
    uint32_t BF2;  // bitfield rgb mask 2 (r)
    uint32_t BF3;  // bitfield rgb mask 3 (b)
} bitmapinfoheader565;

// 52 byte header for RGB565 encoded bitmaps
typedef struct {
    fileheader fileheader;
    bitmapinfoheader565 bitmapinfoheader;
} bitmap565;

char *bmp_create_header(int w, int h);
char *bmp_create_header565(int w, int h);

#endif
