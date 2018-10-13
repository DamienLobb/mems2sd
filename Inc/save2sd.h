/*
 * save2sd.h
 *
 *  Created on: 13 Oct 2018
 *      Author: John
 *
 *  Description:
 *  Manages a buffer of 512 bytes, when this buffer is filled
 *  it then writes this to the SD card using SDIO.
 */

#ifndef SAVE2SD_H_
#define SAVE2SD_H_
#include "fatfs.h"

#define SD_CARD_BUFFER_SZ 10240

/* Fatfs Files and interface */
FATFS fs;
FRESULT fr;
FIL fil;

uint16_t bytes_in_buffer;
uint8_t *sd_buffer_ptr;

/* Total number of bytes written to the SD card
 * since sdStart was called
 */
uint32_t sd_total_bytes_written;

/* Create a buffer to hold the */
uint8_t sd_buffer[SD_CARD_BUFFER_SZ];

uint8_t save2sdWrite(void *buffer, uint16_t num_of_bytes);

uint8_t sdStart();

uint8_t sdStop();

void resetBuffer();

#endif /* SAVE2SD_H_ */