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

//static uint8_t *fb_root = NULL;
static uint32_t *fb_root = NULL;
static int fb_size = 0;

static fb_segment_t *m_ptrFirstSegment  = NULL ;
static fb_segment_t *m_ptrLastSegment   = NULL ;
static fb_segment_t *m_ptrCursorSegment = NULL ;

// we haz some strange bugs when storing this memory as a linked-list only
// we can store the framebuffer segments in an array, maybe...
// TODO: At present, this is a waste of a couple of KB RAM
// EXPECT 2 - 3 SEGMENTS.
static uint32_t* segmentArray[240];
static uint32_t segmentOffsets[240];

static int totalSegments = 0;

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

 uint32_t* framebuffer_create( size_t height, size_t width, size_t bytes_per_pixel )
{
    int mem32required = height * width * bytes_per_pixel;
    size_t min_segment_size = 1 * width * bytes_per_pixel;

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
    size_t free8mem = heap_caps_get_minimum_free_size( MALLOC_CAP_8BIT );
    ESP_LOGD( TAG, "Minimum free size (8-bit capable) - %d", free8mem );
    size_t free32mem = heap_caps_get_minimum_free_size( MALLOC_CAP_32BIT );
    ESP_LOGD( TAG, "Minimum free size (32-bit capable) - %d", free32mem );

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
    		// CHECK FREE BLOCKS IN BOTH 32 BIT AND 8 BIT:
    		size_t mem8free = heap_caps_get_largest_free_block( MALLOC_CAP_8BIT );
    		ESP_LOGD( TAG, "Can get %d contiguous bytes from 8-bit capable memory.", mem8free );
        size_t mem32free = heap_caps_get_largest_free_block( MALLOC_CAP_32BIT );
        ESP_LOGD( TAG, "Can get %d contiguous bytes from 32-bit capable memory (ALLOCATING).", mem32free );

        // ataweg is using the 8-bit low-water mark for available ram to allocate.
        //mem32free = mem8free;
        //ESP_LOGD( TAG, "Using 8-bit capable low-water mark to indicate available RAM for allocating as 32-bit!!!.");

        if( mem32free > mem32required )
            mem32free = mem32required ;

        uint32_t bytes_per_lines = width * bytes_per_pixel;
        uint32_t lines = mem32free / bytes_per_lines;
        size_t segment_size = lines * bytes_per_lines ;
        size_t alloc_size = sizeof(fb_segment_t) + sizeof(uint32_t)*(segment_size/4);
        size_t mem32alloc = segment_size + sizeof( fb_segment_t ); // - sizeof( uint32_t[0] );
        // TODO: ENSURE DYNAMIC ALLOCATION SIZE IS CORRECT
        fb_segment_t *mem_ptr =  heap_caps_malloc( mem32alloc, MALLOC_CAP_32BIT );
        ESP_LOGD( TAG, "Take %d bytes for framebuffer (actual %d)", segment_size, alloc_size  );

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
            fb_root = ( uint32_t * )mem_ptr;
            fb_size = 0;

            // Memory Pool additions
            m_ptrFirstSegment = mem_ptr;
            m_ptrLastSegment = m_ptrFirstSegment;


        }
        else {
            previous_segment->next = mem_ptr;
            m_ptrLastSegment = mem_ptr;
        }

        // TODO: Check alloc size!
        // store refs to each pointer
        segmentArray[totalSegments] = (uint32_t*)mem_ptr->buf;
        segmentOffsets[totalSegments] = fb_size;
        //segmentOffsets[totalSegments] = segment_size;

        previous_segment = mem_ptr;
        totalSegments++;

        mem32required -= segment_size;
        fb_size += segment_size;
        ESP_LOGD( TAG, "Need %d bytes for framebuffer", mem32required );
    }

    ESP_LOGD( TAG, "Framebuffer allocation complete. %d bytes in %d segments", fb_size, totalSegments);
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
        // TODO: ENSURE DYNAMIC ALLOCATION IS FREED
        heap_caps_free( cur_segment );

        ESP_LOGI( TAG, "Free heap: %u", xPortGetFreeHeapSize() );  // heap_caps_get_free_size( MALLOC_CAP_DEFAULT );
        cur_segment = next_segment;
    }
    heap_caps_print_heap_info( MALLOC_CAP_32BIT );
}

 inline bool is_pos_in_segment(uint32_t pos, uint32_t seg_start_pos, uint32_t seg_size) {
	 return (
			 (!(pos < seg_start_pos))  // not before start pos
			 &&
			 (!(pos > (seg_start_pos + (seg_size-1) ))) // not after end pos
	  );
 }

// using linked lists to access segments...
 uint32_t* framebuffer_pos_32_ll( fb_context_t* fbc, uint32_t pos ) {
	 uint32_t fb_pos = pos * 2;
	 bool found = false;
	 uint32_t last_offset = 0;
	 uint32_t segment_end = 0;
	 uint32_t seg_size = 0;

	 if (totalSegments == 0) return 0;
	 m_ptrCursorSegment = m_ptrFirstSegment;

	 for (int i = 0; i < totalSegments; i++) {
		 seg_size = m_ptrCursorSegment->size;
		 if ( (i+1) > totalSegments)
			 segment_end = fb_size;
		  else {
			  segment_end = last_offset + m_ptrCursorSegment->size;
			  //segment_end = segmentOffsets[i+1];
		     }
		 //seg_size = m_ptrCursorSegment->size;
		 //seg_size = segment_end - last_offset;

		 found = is_pos_in_segment(fb_pos, last_offset, seg_size);
		 if (found) {
			 //ESP_LOGD( TAG, "Found: %lu - %lu = %lu > %lu", fb_pos, last_offset,
			 //		                   fb_pos - last_offset, seg_size );
			 return (uint32_t*)(m_ptrCursorSegment->buf+(( fb_pos - last_offset )/4)); //32-bit 1 buf el
			 //return (uint32_t*)(m_ptrCursorSegment->buf+( fb_pos - last_offset ));
			 //return (uint32_t*)(segmentArray[i]+(( fb_pos - last_offset )/4));
		 }

		 last_offset = segment_end;
		 m_ptrCursorSegment = m_ptrCursorSegment->next;
		 //last_offset = segmentOffsets[i];
	 }
	 return NULL;

	 //return ( uint32_t * )framebuffer_pos(fbc,pos);
 }


 // replace 32-bit framebuffer access
 uint32_t* framebuffer_pos_32( fb_context_t* fbc, uint32_t pos ) {
	 uint32_t fb_pos = pos * 2;
	 bool found = false;
	 uint32_t last_offset = 0;
	 uint32_t segment_end = 0;
	 uint32_t seg_size = 0;

	 if (totalSegments == 0) return 0;
	 for (int i = 0; i < totalSegments; i++) {
		 last_offset = segmentOffsets[i];

		 if ( (i+1) > totalSegments)
			 segment_end = fb_size;
		  else
			 segment_end = segmentOffsets[i+1];

		 seg_size = segment_end - last_offset;

		 found = is_pos_in_segment(fb_pos, last_offset,seg_size);

		 if (found) {
			 //ESP_LOGD( TAG, "Found: %lu - %lu = %lu > %lu", fb_pos, last_offset,
			 //		                   fb_pos - last_offset, seg_size );
			 return (uint32_t*)(segmentArray[i]+(( fb_pos - last_offset )/4));
		 }
	 }
	 return NULL;

	 //return ( uint32_t * )framebuffer_pos(fbc,pos);
 }


// original code for in-sync access to framebuffer'z
// pos: postion of the pixel pos = y * width + x
 uint8_t* framebuffer_pos_8( fb_context_t* fbc, uint32_t pos )
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

