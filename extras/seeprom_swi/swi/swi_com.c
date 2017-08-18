/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief temperature sensor source file.
 *
 * - Compiler:           GCC and IAR for AVR
 * - Supported devices:  All AVR XMega devices with TWI module can be used.
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *
 *****************************************************************************/

/* Copyright (c) 2010 Atmel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an Atmel
 * AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "string.h"
#include "swi_com.h"
#include "hardware.h"
#include "spi.h"
#include "swi_phy.h"


#define GP  0x107   /* x^8 + x^2 + x + 1 */
#define DI  0x07
#define PAGE_SIZE 0x08

static unsigned char crc8_table[256];     /* 8-bit table */
static int made_table=0;

void crc8(unsigned char *crc, unsigned char m);
void init_crc8(void);


/*!
 * \brief 
 *
 * \param 
 *						
 * \param *buf	
 * \return TWI_SUCCESS if all bytes were written, error code otherwise
 */
uint8_t read_mfg_id(uint8_t dev_addr, uint8_t *buf)
{
	swi_package_t packet;

	// Op Code
	packet.opcode = MFGIDEAD;
	
	// SWI chip address to communicate with
	packet.dev_addr = dev_addr;
	
	// Internal memory address / pointer register
	packet.mem_addr = 0x00;
	packet.mem_addr_length = 0x00;

	// How many bytes to write
	packet.wlen = 0;
	
    // How many bytes to read
	packet.rlen = 3;
	
	// Where to put the read data
	packet.buffer = buf;

	packet.chk_ack_only_flag = 0;
	
	// Perform a read access & check result
	if (swi_read(&packet) != SWI_SUCCESS)
		return SWI_FUNCTION_RETCODE_DATA_NAK_FAIL;

	return SWI_SUCCESS;
}

uint8_t  read_security_register(uint8_t dev_addr, uint8_t mem_addr, uint8_t rlen, uint8_t *buf)
{
	swi_package_t packet;
	
	// Op Code
	packet.opcode = SECREGACCESS;
	
	// SWI chip address to communicate with
	packet.dev_addr = dev_addr;
	
	// Internal memory address / pointer register
	packet.mem_addr = mem_addr;
	packet.mem_addr_length = 0x01;

	// How many bytes to write
	packet.wlen = 0;
	
	// How many bytes to read
	packet.rlen = rlen;
	
	// Where to put the read data
	packet.buffer = buf;

	packet.chk_ack_only_flag = 0;
	
	// Perform a read access & check result
	if (swi_read(&packet) != SWI_SUCCESS)
	return SWI_FUNCTION_RETCODE_DATA_NAK_FAIL;

	return SWI_SUCCESS;	
	
}


uint8_t  write_security_register(uint8_t dev_addr, uint8_t mem_addr, uint8_t wlen,uint8_t *buf)
{
	swi_package_t packet;
	
	// Op Code
	packet.opcode = SECREGACCESS;
	
	// SWI chip address to communicate with
	packet.dev_addr = dev_addr;
	
	// Internal memory address / pointer register
	packet.mem_addr = mem_addr;
	packet.mem_addr_length = 0x01;

	// How many bytes to write
	packet.wlen = wlen;
	
	// How many bytes to read
	packet.rlen = 0;
	
	// Where to put the read data
	packet.buffer = buf;

	packet.chk_ack_only_flag = 0;
	
	// Perform a read access & check result
	if (swi_write(&packet) != SWI_SUCCESS)
	{
		/* device not in high speed mode */
		return SWI_FUNCTION_RETCODE_DATA_NAK_FAIL;
	}
	
	/* device is in high speed mode */
	return SWI_SUCCESS;
	
}


uint8_t  lock_security_register(uint8_t dev_addr, uint8_t *buf)
{
	swi_package_t packet;

	// Op Code
	packet.opcode = LOCKSECREG;
	
	// SWI chip address to communicate with
	packet.dev_addr = dev_addr;
	
	// Internal memory address / pointer register
	packet.mem_addr = 0x60;
	packet.mem_addr_length = 0x01;

	// How many bytes to write
	packet.wlen = 1;
	
	// How many bytes to read
	packet.rlen = 0;
	
	// Where to put the read data
	packet.buffer = buf;

	packet.chk_ack_only_flag = 0;
	
	// Perform a read access & check result
	if (swi_write(&packet) != SWI_SUCCESS)
	return SWI_FUNCTION_RETCODE_ADDR_NAK_FAIL;

	return SWI_SUCCESS;
	
}


uint8_t  check_lock_command(uint8_t dev_addr, uint8_t *buf)
{
	swi_package_t packet;
	uint8_t retCode = 0xFF;
	
	// Op Code
	packet.opcode = LOCKSECREG;
	
	// SWI chip address to communicate with
	packet.dev_addr = dev_addr;
	
	// Internal memory address / pointer register
	packet.mem_addr = 0x60;
	packet.mem_addr_length = 0x01;

	// How many bytes to write
	packet.wlen = 0;
	
	// How many bytes to read
	packet.rlen = 0;
	
	// Where to put the read data
	packet.buffer = buf;

	packet.chk_ack_only_flag = 0;
	
	// Perform a read access & check result
	retCode = swi_write(&packet);

	return retCode;
	
}


uint8_t  chk_std_speed_mode(uint8_t dev_addr, uint8_t *buf)
{
	swi_package_t packet;
	
	// Op Code
	packet.opcode = STDSPEEDMODE;
	
	// SWI chip address to communicate with
	packet.dev_addr = dev_addr;
	
	// Internal memory address / pointer register
	packet.mem_addr = 0x00;
	packet.mem_addr_length = 0x00;

	// How many bytes to write
	packet.wlen = 0;
	
	// How many bytes to read
	packet.rlen = 1;
	
	// Where to put the read data
	packet.buffer = buf;

	packet.chk_ack_only_flag = 1;
	
	// Perform a read access & check result
	if (swi_read(&packet) != SWI_SUCCESS)
	{
		/* device not in low speed mode */
		return SWI_FUNCTION_RETCODE_ADDR_NAK_FAIL;
	}
	
	/* device is in low speed mode */
	return SWI_SUCCESS;
	
}

uint8_t  chk_high_speed_mode(uint8_t dev_addr, uint8_t *buf)
{
	swi_package_t packet;
	
	// Op Code
	packet.opcode = HIGHSPEEDMODE;
	
	// SWI chip address to communicate with
	packet.dev_addr = dev_addr;
	
	// Internal memory address / pointer register
	packet.mem_addr = 0x00;
	packet.mem_addr_length = 0x00;

	// How many bytes to write
	packet.wlen = 0;
	
	// How many bytes to read
	packet.rlen = 0;
	
	// Where to put the read data
	packet.buffer = buf;

	packet.chk_ack_only_flag = 1;
		
	// Perform a read access & check result
	if (swi_read(&packet) != SWI_SUCCESS)
	{
		/* device not in high speed mode */
		return SWI_FUNCTION_RETCODE_ADDR_NAK_FAIL;
	}
	
	/* device is in high speed mode */
	return SWI_SUCCESS;
	
}


uint8_t  set_std_speed_mode(uint8_t dev_addr, uint8_t *buf)
{
	swi_package_t packet;
	
	// Op Code
	packet.opcode = STDSPEEDMODE;
	
	// SWI chip address to communicate with
	packet.dev_addr = dev_addr;
	
	// Internal memory address / pointer register
	packet.mem_addr = 0x00;
	packet.mem_addr_length = 0x00;

	// How many bytes to write
	packet.wlen = 0;
	
	// How many bytes to read
	packet.rlen = 1;
	
	// Where to put the read data
	packet.buffer = buf;

	packet.chk_ack_only_flag = 1;
	
	// Perform a write operation & check result
	if (swi_write_stdspeed_cmd(&packet) != SWI_SUCCESS)
	{
		/* device not in high speed mode */
		return SWI_FUNCTION_RETCODE_ADDR_NAK_FAIL;
	}
	
	/* device is in high speed mode */
	return SWI_SUCCESS;
	
}


uint8_t  set_high_speed_mode(uint8_t dev_addr, uint8_t *buf)
{
	swi_package_t packet;
	
	// Op Code
	packet.opcode = HIGHSPEEDMODE;
	
	// SWI chip address to communicate with
	packet.dev_addr = dev_addr;
	
	// Internal memory address / pointer register
	packet.mem_addr = 0x00;
	packet.mem_addr_length = 0x00;

	// How many bytes to write
	packet.wlen = 0;
	
	// How many bytes to read
	packet.rlen = 1;
	
	// Where to put the read data
	packet.buffer = buf;

	packet.chk_ack_only_flag = 1;
	
	// Perform a write operation & check result
	if (swi_write(&packet) != SWI_SUCCESS)
	{
		/* device not in high speed mode */
		return SWI_FUNCTION_RETCODE_ADDR_NAK_FAIL;
	}
	
	/* device is in high speed mode */
	return SWI_SUCCESS;
	
}


uint8_t  write_memory(uint8_t dev_addr, uint8_t mem_addr, uint8_t wlen,uint8_t *buf)
{
	swi_package_t packet;
	
	// Op Code
	packet.opcode = EEPROM_ADDRESS;
	
	// SWI chip address to communicate with
	packet.dev_addr = dev_addr;
	
	// Internal memory address / pointer register
	packet.mem_addr = mem_addr;
	packet.mem_addr_length = 0x01;

	// How many bytes to write
	packet.wlen = wlen;
	
	// How many bytes to read
	packet.rlen = 0;
	
	// Where to put the read data
	packet.buffer = buf;

	packet.chk_ack_only_flag = 0;
	
	// Perform a read access & check result
	if (swi_write(&packet) != SWI_SUCCESS)
	{
		/* device not in high speed mode */
		return SWI_FUNCTION_RETCODE_DATA_NAK_FAIL;
	}
	
	/* device is in high speed mode */
	return SWI_SUCCESS;
	
}


uint8_t read_memory(uint8_t dev_addr, uint8_t mem_addr, uint8_t rlen,uint8_t *buf)
{
	swi_package_t packet;
	
	// Op Code
	packet.opcode = EEPROM_ADDRESS;
	
	// SWI chip address to communicate with
	packet.dev_addr = dev_addr;
	
	// Internal memory address / pointer register
	packet.mem_addr = mem_addr;
	packet.mem_addr_length = 0x01;

	// How many bytes to write
	packet.wlen = 0;
	
	// How many bytes to read
	packet.rlen = rlen;
	
	// Where to put the read data
	packet.buffer = buf;

	packet.chk_ack_only_flag = 0;
	
	// Perform a read access & check result
	if (swi_read(&packet) != SWI_SUCCESS)
	{
		/* device not in high speed mode */
		return SWI_FUNCTION_RETCODE_ADDR_NAK_FAIL;
	}
	
	/* device is in high speed mode */
	return SWI_SUCCESS;
	
}




uint8_t  reading_rom_zone_register(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf)
{
	swi_package_t packet;
	
	// Op Code
	packet.opcode = ROMZONEREGACCESS;
	
	// SWI chip address to communicate with
	packet.dev_addr = dev_addr;
	
	// Internal memory address / pointer register
	packet.mem_addr = reg_addr;
	packet.mem_addr_length = 0x01;

	// How many bytes to write
	packet.wlen = 0;
	
	// How many bytes to read
	packet.rlen = 1;
	
	// Where to put the read data
	packet.buffer = buf;

	packet.chk_ack_only_flag = 0;
	
	// Perform a read access & check result
	if (swi_read(&packet) != SWI_SUCCESS)
	{
		/* device not in high speed mode */
		return SWI_FUNCTION_RETCODE_ADDR_NAK_FAIL;
	}
	
	/* device is in high speed mode */
	return SWI_SUCCESS;
	
}

uint8_t  writing_rom_zone_register(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf)
{
	swi_package_t packet;
	
	// Op Code
	packet.opcode = ROMZONEREGACCESS;
	
	// SWI chip address to communicate with
	packet.dev_addr = dev_addr;
	
	// Internal memory address / pointer register
	packet.mem_addr = reg_addr;
	packet.mem_addr_length = 0x01;

	// How many bytes to write
	packet.wlen = 1;
	
	// How many bytes to read
	packet.rlen = 0;
	
	// Where to put the read data
	packet.buffer = buf;

	packet.chk_ack_only_flag = 0;
	
	// Perform a read access & check result
	if (swi_write(&packet) != SWI_SUCCESS)
	{
		/* device not in high speed mode */
		return SWI_FUNCTION_RETCODE_ADDR_NAK_FAIL;
	}
	
	/* device is in high speed mode */
	return SWI_SUCCESS;
	
}

uint8_t freeze_rom_zone_state(uint8_t dev_addr, uint8_t *buf)
{
	swi_package_t packet;
	
	// Op Code
	packet.opcode = FREEZEROMZONESTATE;
	
	// SWI chip address to communicate with
	packet.dev_addr = dev_addr;
	
	// Internal memory address / pointer register
	packet.mem_addr = 0x55;
	packet.mem_addr_length = 0x01;

	// How many bytes to write
	packet.wlen = 1;
	
	// How many bytes to read
	packet.rlen = 0;
	
	// Where to put the read data
	packet.buffer = buf;

	packet.chk_ack_only_flag = 0;
	
	// Perform a read access & check result
	if (swi_write(&packet) != SWI_SUCCESS)
	{
		/* device not in high speed mode */
		return SWI_FUNCTION_RETCODE_ADDR_NAK_FAIL;
	}
	
	/* device is in high speed mode */
	return SWI_SUCCESS;
	
}


/*
* crc8.c
*
* Computes a 8-bit CRC
*
*/

  
 void init_crc8()
{
	int i,j;
	unsigned char crc;

	if (!made_table) 
	{
		for (i=0; i<256; i++) 
		{
			crc = i;
			for (j=0; j<8; j++)
			{
				crc = (crc << 1) ^ ((crc & 0x80) ? DI : 0);				
			}
			crc8_table[i] = crc & 0xFF;
		}
		made_table=1;
	}
}

/*
* For a byte array whose accumulated crc value is stored in *crc, computes
* resultant crc obtained by appending m to the byte array
*/
 void crc8(unsigned char *crc, unsigned char m)
 {
	if (!made_table)
		init_crc8();

	*crc = crc8_table[(*crc) ^ m];
	*crc &= 0xFF;
 }
 
 
 uint8_t set_vcc_level(uint16_t vcc)
 {
	 // 2^n = 4096
	 // vout = ((Vref*Dn)/2^N)*G -> = (4.68*Dn)/4096)*G
	 // 1.14mV resolution
	 
	 // Example:
	 // Dn = 875.213*Vout
	 // Desired vout = 2V,
	 // Dn = 1750

	 static uint8_t vcc_level[2];

	 vcc_level[1] = vcc & 0xFF;
	 vcc_level[0] = (vcc>>8) & 0xFF;
	 
	 gpio_set_pin_low(CSDAC_PORT,CSDAC);
	 Spi_send_and_receive ( vcc_level[0]);
	 Spi_send_and_receive ( vcc_level[1]);
	 gpio_set_pin_high(CSDAC_PORT,CSDAC);

	 /* latch adc data */
	 gpio_set_pin_low(LDAC_PORT,LDAC);
	 _delay_us(4);
	 gpio_set_pin_high(LDAC_PORT,LDAC);

	 return 0x00;
 }


 uint16_t get_current_meas()
 {
	 /* using a 16 bits ADC */
	 
	 uint16_t measurement = 0;
	 static uint8_t msb;
	 static uint8_t lsb;
	 
	 msb = 0;
	 lsb = 0;
	 
	 gpio_set_pin_low(ADCCS_PORT,ADCCS);
	 msb = Spi_send_and_receive (0xFF);
	 lsb = Spi_send_and_receive (0xFF);
	 gpio_set_pin_high(ADCCS_PORT,ADCCS);

	 measurement = ((msb<<8)|(lsb));

	 return measurement;
 }
 
 /*
 * Scan to determine slave device address
 */
 uint8_t scan_swi_device_addr (void)
 {
	 uint8_t discoveredAddr = NO_DEVICE_DETECTED;
	 uint8_t errCode;
	 uint8_t addr;
	 
	 for(uint8_t i =0; i < 8; i++)
	 {
		 if(swi_device_discovery() == 0)
		 {

			swi_start_stop_cond();
			
			// placing device address on the bus
			addr = i;
			// shift address to proper bit position
			addr <<= 1;
			// add EEPROM op code and set R/W = 1
			addr |= 0xA1;
			
			errCode = swi_send_bytes(0x01, &addr);
		
			if(errCode == 0x00)//ACK
			{	
				discoveredAddr = i;
				/*shift bit into correct position */
				discoveredAddr <<= 1;
				swi_start_stop_cond();			
				break;
			}

		 }
	 }

	return discoveredAddr;
 }
  
 
 

uint8_t  write_memory_buffer(uint8_t dev_addr, uint8_t mem_addr, uint8_t data_size, uint8_t *buf)
{
	static uint8_t write_size 			= 0;
	static uint8_t n 					= 0;
	static uint8_t status              = 0;
	static uint16_t start_page_addr 	= 0;
	static uint16_t end_page_addr 		= 0;
	   
	do
	{
		n = (uint16_t) (mem_addr / PAGE_SIZE);
		start_page_addr = n * PAGE_SIZE;
		end_page_addr = ((n + 1) * PAGE_SIZE);
		   
		// response_data doesn't fit, do a partial write
		if ((mem_addr + data_size) > end_page_addr)
			write_size = end_page_addr - mem_addr;
		else
			write_size = data_size;

		status = write_memory(dev_addr, mem_addr, write_size,buf);
		   
		if (status != SWI_SUCCESS)
		return status;
		   
		data_size -= write_size;
		mem_addr += write_size;
		buf += write_size;
		   
	}
	while (data_size);
}

 
 