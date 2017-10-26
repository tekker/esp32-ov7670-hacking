#pragma once


// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

struct fb_segment_t;
typedef struct fb_segment fb_segment_t;

struct fb_segment
{
    uint32_t size;
    uint32_t lines;
    fb_segment_t *next;
    uint8_t buf[4];
};

typedef struct
{
    int fb_offset;
    fb_segment_t *fb_segment;
} fb_context_t;

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------


uint8_t* framebuffer_create( uint32_t height, uint32_t width, uint32_t bytes_per_pixel );
void framebuffer_free( void *buf );
uint8_t* framebuffer_pos( fb_context_t* fbc, uint32_t pos );
int framebuffer_size( void );

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

