// --------------------------------------------------------------------------
//
// Project       IoT - Internet of Things - Wifi Camera
//
// File          framebuffer.c
//
// Author        Axel Werner
//
// --------------------------------------------------------------------------
// Changelog
//
// 2017-10-11  AWe   reuse the same framebuffer also for smaller imamges
// 2017-10-08  AWe   put all framebuffer stuff in this file
//
// --------------------------------------------------------------------------

#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <esp_types.h>

#include "framebuffer.h"

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

static const char* TAG = "framebuffer";

uint8_t *fb_root = NULL;
int fb_size = 0;

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

uint8_t* framebuffer_create( size_t height, size_t width, size_t bytes_per_pixel )
{
    int mem32required = height * width * bytes_per_pixel;

    if( fb_root != NULL)
    {
        if( mem32required > fb_size)
        {
            // maybe it is better to keep the framebuffer for smaller images
            ESP_LOGD( TAG, "Try to free framebuffer" );
            framebuffer_free( fb_root );
            fb_root = NULL;
            fb_size = 0;
        }
        else
        {
            return fb_root;
        }
    }

    size_t free32mem = heap_caps_get_minimum_free_size( MALLOC_CAP_32BIT );
    if( free32mem < mem32required )
    {
        // no space to allocate memory for framebuffer
        ESP_LOGE( TAG, "no space to allocate %d byte memory for framebuffer. Have only %d bytes",
                  mem32required, free32mem );
        return NULL;
    }

    ESP_LOGD( TAG, "Need %d bytes for framebuffer", mem32required );

    fb_segment_t *previous_segment = NULL;

    while( mem32required > 0 )
    {
        size_t mem32free = heap_caps_get_largest_free_block( MALLOC_CAP_8BIT );
        ESP_LOGD( TAG, "Can get %d bytes for framebuffer", mem32free );

        if( mem32free > mem32required )
            mem32free = mem32required ;

        uint32_t bytes_per_lines = width * bytes_per_pixel;
        uint32_t lines = mem32free / bytes_per_lines;
        size_t segment_size = lines * bytes_per_lines ;
        int mem32alloc = segment_size + sizeof( fb_segment_t ) - sizeof( uint32_t );

        fb_segment_t *mem_ptr =  heap_caps_malloc( mem32alloc, MALLOC_CAP_32BIT );
        ESP_LOGD( TAG, "Take %d bytes for framebuffer", segment_size );

        if( mem_ptr == NULL )
        {
            ESP_LOGE( TAG, "Failed to allocate frame buffer" );
            framebuffer_free( fb_root );
            return NULL;
        }

        mem_ptr->next = NULL;
        mem_ptr->lines = lines;
        mem_ptr->size =  segment_size;

        if( fb_root == NULL )
        {
            fb_root = ( uint8_t * )mem_ptr;
            fb_size = 0;
        }
        else
            previous_segment->next = mem_ptr;

        previous_segment = mem_ptr;

        mem32required -= segment_size;
        fb_size += segment_size;
        ESP_LOGD( TAG, "Need %d bytes for framebuffer", mem32required );
    }

    return fb_root;
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

void framebuffer_free( void *buf )
{
    ESP_LOGD( TAG, "Free framebuffer" );
    ESP_LOGI( TAG, "Free heap: %u", xPortGetFreeHeapSize() );
    heap_caps_print_heap_info( MALLOC_CAP_32BIT );
    fb_segment_t *cur_segment = ( fb_segment_t * ) buf;
    while( cur_segment != NULL )
    {
        fb_segment_t *next_segment = cur_segment->next;
        heap_caps_free( cur_segment );
        ESP_LOGI( TAG, "Free heap: %u", xPortGetFreeHeapSize() );  // heap_caps_get_free_size( MALLOC_CAP_DEFAULT );
        cur_segment = next_segment;
    }
    heap_caps_print_heap_info( MALLOC_CAP_32BIT );
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

// pos: postion of the pixel pos = y * width + x
uint8_t* framebuffer_pos( fb_context_t* fbc, uint32_t pos )
{
    int fb_pos = pos * 2;   // 2 = bytes_per_pixel

    if( fb_pos == 0 )
    {
        fbc->fb_offset = 0;
        fbc->fb_segment = ( fb_segment_t * )fb_root;
    }

    if( ( fb_pos - fbc->fb_offset ) == fbc->fb_segment->size )
    {
        fbc->fb_offset +=  fbc->fb_segment->size;
        fbc->fb_segment = fbc->fb_segment->next;

        //ESP_LOGD( TAG, "Next segment  %d - %d = %d -> %d", fb_pos, fbc->fb_offset,
        //          fb_pos - fbc->fb_offset, fbc->fb_segment->size );
    }
    else if( ( fb_pos - fbc->fb_offset ) > ( int )fbc->fb_segment->size )
    {
        ESP_LOGE( TAG, "Out of sync  %d - %d = %d > %d", fb_pos, fbc->fb_offset,
                  fb_pos - fbc->fb_offset, fbc->fb_segment->size );
    }

    return ( uint8_t * )( fbc->fb_segment->buf + ( fb_pos - fbc->fb_offset ) );
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

int framebuffer_size( void )
{
    return fb_size;
}
// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

