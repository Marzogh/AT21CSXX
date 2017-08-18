/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief temperature sensor driver for AVR.
 *
 * This file defines a useful set of functions for the AT30TSE002B and AT30TS75x
 * devices.
 *
 * - Compiler:           IAR and GNU GCC for AVR
 * - Supported devices:  All AVR devices with an TWI module can be used.
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *
 ******************************************************************************/

/* Copyright (c) 2009 Atmel Corporation. All rights reserved.
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


#ifndef AT21CS01_H_
#define AT21CS01_H_

#include <stdint.h>


#define NO_DEVICE_DETECTED					 0xFF
// error codes
//!
#define AT21CS01_SWI_SUCCESS				  ( 0)
#define AT21CS01_SWI_GENERAL_ERROR			  (-1)
#define AT21CS01_SWI_WRITE_NACK				  (-2)
#define AT21CS01_SWI_READ_NACK				  (-3)
#define AT21CS01_SWI_REG_LOCKED               (-4)
#define AT21CS01_SWI_INVALID_EEPROM_ADDRESS   (-5)
#define AT21CS01_SWI_INVALID_SIZE             (-6)
#define AT21CS01_SWI_OUT_OF_BOUNDS            (-7)

#define SWI_SUCCESS                           ( 0)

/*! \name AT21CS01 OP Code registers
 */
//! @{
#define EEPROM_ADDRESS				0x0A
#define SECREGACCESS				0x0B
#define LOCKSECREG					0x02
#define ROMZONEREGACCESS			0x07
#define FREEZEROMZONESTATE			0x01
#define MFGIDEAD					0x0C
#define STDSPEEDMODE				0x0D
#define HIGHSPEEDMODE				0x0E
//! @}

#define ROMZONEREG_MEMZONE0			((uint8_t)0x01)
#define ROMZONEREG_MEMZONE1			((uint8_t)0x02)
#define ROMZONEREG_MEMZONE2			((uint8_t)0x04)
#define ROMZONEREG_MEMZONE3			((uint8_t)0x08)

uint8_t  read_mfg_id(uint8_t dev_addr, uint8_t *buf);
uint8_t  read_security_register(uint8_t dev_addr,uint8_t parm1, uint8_t parm2, uint8_t *buf);
uint8_t  chk_high_speed_mode(uint8_t dev_addr, uint8_t *buf);
uint8_t  chk_std_speed_mode(uint8_t dev_addr, uint8_t *buf);

uint8_t  set_std_speed_mode(uint8_t dev_addr, uint8_t *buf);
uint8_t  set_high_speed_mode(uint8_t dev_addr, uint8_t *buf);
uint8_t  freeze_rom_zone_register(uint8_t dev_addr, uint8_t *buf);
uint8_t  reading_rom_zone_register(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf);
uint8_t  writing_rom_zone_register(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf);
uint8_t  freeze_rom_zone_state(uint8_t dev_addr, uint8_t *buf);
uint8_t  lock_security_register(uint8_t dev_addr, uint8_t *buf);
uint8_t  write_security_register(uint8_t dev_addr, uint8_t arg1,uint8_t arg2, uint8_t *buf);
uint8_t  check_lock_command(uint8_t dev_addr, uint8_t *buf);
uint8_t  read_memory(uint8_t dev_addr, uint8_t mem_addr, uint8_t rlen,uint8_t *buf);
uint8_t  write_memory(uint8_t dev_addr, uint8_t mem_addr, uint8_t wlen,uint8_t *buf);
uint8_t  write_memory_buffer(uint8_t dev_addr, uint8_t mem_addr, uint8_t wlen,uint8_t *buf);
uint8_t  set_vcc_level(uint16_t vcc);
uint16_t get_current_meas(void);
uint8_t  scan_swi_device_addr (void);
#endif	
