/*
 * save2sd.c
 *
 *  Created on: 13 Oct 2018
 *      Author: John
 */

#include "save2sd.h"
#include <string.h>

/**
 * @brief This will copy data from buffer in to the
 * the sd_card buffer. When this buffer reaches
 * 512 bytes it will then be written to the SD card
 * @param buffer - Pointer to the buffer to copy from
 * @param num_of_bytes - Number of bytes to copy from buffer
 * @return 0 on success
 */
uint8_t save2sdWrite(uint16_t *buffer, uint16_t num_of_elements)
{
	uint8_t ret = 0;
	unsigned int bytes_written;
	//uint32_t start = HAL_GetTick();

	uint16_t bytes_from_elements =  num_of_elements * 2;

	/* Check if we will fill the buffer on this call */
	uint16_t byte_count = bytes_from_elements + bytes_in_buffer;
	if(byte_count < SD_CARD_BUFFER_SZ)
	{
		/* Copy bytes to the SD Buffer */
		memcpy(sd_buffer_ptr, buffer, bytes_from_elements);
		sd_buffer_ptr += bytes_from_elements;
		bytes_in_buffer = byte_count;
	}
	else if (byte_count == SD_CARD_BUFFER_SZ)
	{
		/* We have filled the buffer write to the card*/
		memcpy(sd_buffer_ptr, buffer, bytes_from_elements);


		if(FR_OK == f_write(&fil, sd_buffer[sd_buffer_num], SD_CARD_BUFFER_SZ, &bytes_written))
		{
			sd_total_bytes_written += bytes_written;
			sd_buffer_num = (sd_buffer_num+1)%2; //Assigns buffer_num to the remainder of (current value+1)/2
												//when 0 -> 1, when 1 -> 0
		}
		else
			ret = -1;

		bytes_in_buffer = 0;
		sd_buffer_ptr = sd_buffer[sd_buffer_num];
	}
	else
	{
		/* Not needed as always hits the buffer size*/

		/* We have more bytes than the buffer size*/

		/* Write bytes to SD card*/

		/* Reset SD waiting Buffer*/

		/* Add bytes to SD waiting buffer*/
	}

	/* Not filled yet so just copy in to buffer*/

	/* Buffer will be full so copy bytes to 512bytes */

	return ret;
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
	FRESULT res = f_lseek(&fil, 400000);	//Expands file in advance to make writes faster
		if(res != FR_OK) return ret;

		res = f_lseek(&fil, 0); 			//Returns to start of file
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
	sd_buffer_ptr = sd_buffer[sd_buffer_num];
}
