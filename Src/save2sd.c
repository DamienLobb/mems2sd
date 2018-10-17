/*
 * save2sd.c
 *
 *  Created on: 13 Oct 2018
 *      Author: John
 */

#include "save2sd.h"
#include <string.h>
#include "stdbool.h"
#include "audio_application.h"
/**
 * @brief This will copy data from buffer in to the
 * the sd_card buffer. When this buffer reaches
 * 512 bytes it will then be written to the SD card
 * @param buffer - Pointer to the buffer to copy from
 * @param num_of_bytes - Number of bytes to copy from buffer
 * @return 0 on success
 */
bool audio_ready = false;
#define AUDIO_SD_BUFFER_LENGTH 8192
#define  AUDIO_PCM_BUFFER_LENGTH 128
uint32_t SD_buffer_pos = 0;
uint32_t SD_buffer_num = 0;
uint32_t SD_buffers[2][AUDIO_SD_BUFFER_LENGTH];
uint32_t total_samples = 0;


uint8_t save2sdWrite(uint16_t *buffer, uint16_t num_of_elements)
{
	//uint8_t ret = 0;
	unsigned int bytes_written;
/*	//uint32_t start = HAL_GetTick();

	uint16_t bytes_from_elements =  num_of_elements * 2;
*/
	/* Check if we will fill the buffer on this call */
/*	uint16_t byte_count = bytes_from_elements + bytes_in_buffer;
	if(byte_count < SD_CARD_BUFFER_SZ)
	{
*/		/* Copy bytes to the SD Buffer */
/*		memcpy(sd_buffer_ptr, buffer, bytes_from_elements);
		sd_buffer_ptr += bytes_from_elements;
		bytes_in_buffer = byte_count;
	}
	else if (byte_count == SD_CARD_BUFFER_SZ)
	{
*/		/* We have filled the buffer write to the card
		memcpy(sd_buffer_ptr, buffer, bytes_from_elements);

		if(FR_OK == f_write(&fil, sd_buffer, SD_CARD_BUFFER_SZ, &bytes_written))
		{
			sd_total_bytes_written += bytes_written;

		}
		else
			ret = -1;

		bytes_in_buffer = 0;
		sd_buffer_ptr = sd_buffer;
	}
	else
	{
		 Not needed as always hits the buffer size*/

		/* We have more bytes than the buffer size*/

		/* Write bytes to SD card*/

		/* Reset SD waiting Buffer*/

		/* Add bytes to SD waiting buffer*/
//	}

	/* Not filled yet so just copy in to buffer*/

	/* Buffer will be full so copy bytes to 512bytes */

//	return ret;
	uint32_t current_samples = 0;
	if(audio_ready){
		current_samples = SD_buffer_pos;
		SD_buffer_pos = 0;
		total_samples += current_samples;
		sd_total_bytes_written = total_samples;
		// Switch to the other buffer while writing out this buffer
		SD_buffer_num = (SD_buffer_num + 1)%2;
		audio_ready = false;

		f_write(&fil, SD_buffers[SD_buffer_num], current_samples*2, &bytes_written);
	}
	return 0;
}

void save2sd2(){
	if(audio_ready) return;

		if(SD_buffer_pos + AUDIO_PCM_BUFFER_LENGTH < AUDIO_SD_BUFFER_LENGTH)
		{
			memcpy(SD_buffers[SD_buffer_num] + SD_buffer_pos, PCM_Buffer, AUDIO_PCM_BUFFER_LENGTH);
			SD_buffer_pos += AUDIO_PCM_BUFFER_LENGTH;
		}

		if(SD_buffer_pos + AUDIO_PCM_BUFFER_LENGTH >= AUDIO_SD_BUFFER_LENGTH) audio_ready = true;
	}

/**
 * @brief Convenience function to open the SD card file for writing
 * @return 1 on success -1 on failure
 */
uint8_t sdStart()
{
	uint8_t ret = -1;
	sd_total_bytes_written = 0;

	resetBuffer();

	if(FR_OK == f_mount(&fs, "0:", 1))
		if(FR_OK==f_open(&fil, "0:SAV2.raw", FA_CREATE_ALWAYS | FA_WRITE | FA_READ))
			ret = 0;
	FRESULT res = f_lseek(&fil, 400000);
		if(res != FR_OK) return ret;

		res = f_lseek(&fil, 0);
		if(res != FR_OK) return ret;

	return ret;
}

/**
 * @brief Closes the file and dismounts the sd card drive
 * @return 1 on success -1 on failure
 */
uint8_t sdStop()
{
	uint8_t ret = -1;

	/* Write any remaining bytes to the card*/

	if( FR_OK == f_sync (&fil))
		if( FR_OK == f_close (&fil))
			ret = 0;

	return ret;
}

void resetBuffer()
{
	/* Set the number of byes to zero*/
	bytes_in_buffer = 0;
	sd_buffer_ptr = sd_buffer;
}
