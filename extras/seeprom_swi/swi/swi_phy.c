// ----------------------------------------------------------------------------
//         ATMEL Microcontroller Software Support  -  Colorado Springs, CO -
// ----------------------------------------------------------------------------
// DISCLAIMER:  THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
// DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
// OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// ----------------------------------------------------------------------------
/** \file
 *  \brief 
 *       
 *  \author
 *  \date 
 */

#include <stdint.h>          // data type definitions
#include "string.h"
#include "swi_com.h"
#include "hardware.h"
#include "swi_phy.h"

#define low			0x00
#define high		0x01
#define ACK			0x00
#define device_pin  0x02 //PD2

// delay represents tRD
#define initDeviceWriteHi(pin)	{drive_si_low(pin); _delay_us(1); release_si(pin);}
#define initDeviceWriteStd(pin) {drive_si_low(pin); _delay_us(6); release_si(pin);}
	
uint8_t cmdWriteBuffer[64];


uint16_t gtLOW1_DLY;
uint16_t gtRCV1_DLY;

uint16_t gtLOW0_DLY;
uint16_t gtRCV0_DLY;
uint16_t gtDr_DLY;
uint16_t tlwo0;


void selectSpeed(uint8_t speed)
{
	switch(speed)
	{
		case 0:
			tlwo0 = 6;
		break;
	
	
		case 1:
			tlwo0 = 1;
		break;			
		
	}

}


uint8_t swi_write(const swi_package_t *packet )
{

	uint8_t errCode;
	uint8_t index;
	uint8_t writeSize = 1;
	
	errCode = 0;
	index = 0;
	
	/* load device addr w/ opcode
	 * and clear the R/W bit
	 */
	cmdWriteBuffer[index++] = (((packet->opcode<<4)|packet->dev_addr) & ~0x01);	
	
	/* get memory address */
	if(packet->mem_addr_length)
	{
		/* memory address */
		cmdWriteBuffer[index++] = packet->mem_addr;
		writeSize++;
	}
	
	/* load data to be written */
	memcpy(&cmdWriteBuffer[index],packet->buffer,packet->wlen);
	
	writeSize += packet->wlen;
	
	/* send start condition */
	swi_start_stop_cond();

	errCode = swi_send_bytes(writeSize, &cmdWriteBuffer[0]);
	
	swi_start_stop_cond();

	/* wait maximum write time */
	if(packet->mem_addr_length)
	{	   
		_delay_ms(5);
	}

	return errCode;
	
}


uint8_t swi_read(const swi_package_t *packet)
{
	
	uint8_t index;
	uint8_t errCode;
	uint8_t writeSize = 1;
	errCode = 0;
	index = 0;
	
	/* load device addr w/ opcode*/
	cmdWriteBuffer[index++] = ((packet->opcode<<4)|packet->dev_addr);
	
	/* get memory address */
	if(packet->mem_addr_length)
	{
		/* memory address */
		cmdWriteBuffer[index++] = packet->mem_addr;
		writeSize++;		
	}
	
	/* send start condition */
	swi_start_stop_cond();
	
	/* put device address and
	 * memory address if applicable
	 */
	
	/* random read */
	if(packet->mem_addr_length)
	{   // performing dummy write
		cmdWriteBuffer[0] &= 0xFE;
		errCode = swi_send_bytes(writeSize, &cmdWriteBuffer[0]);
		swi_start_stop_cond();
	}

   if(errCode == ACK)
   {
		// placing device address on the bus
		cmdWriteBuffer[0] |= 0x01;	
		errCode = swi_send_bytes((writeSize - packet->mem_addr_length), &cmdWriteBuffer[0]);
		 	
		if((errCode == ACK)&&(!packet->chk_ack_only_flag))
		{
			//perform Read Operation
			swi_receive_bytes(packet->rlen, packet->buffer);	
		}

		swi_start_stop_cond();
   }

	return errCode;
}


/** \brief This GPIO function sends one byte to an SWI device.
 * \param[in] value byte to send
 * \return status of the operation
 */
uint8_t swi_send_byte(uint8_t value)
{
	return swi_send_bytes(1, &value);
}

/** \brief This GPIO function sends bytes to an SWI device.
 * \param[in] count number of bytes to send
 * \param[in] buffer pointer to tx buffer
 * \return status of the operation
 */
uint8_t swi_send_bytes(uint8_t count, uint8_t *buffer)
{

	uint8_t i, bit_mask;
    uint8_t retCode;
	// Disable interrupts while sending.
	swi_disable_interrupts();

	// Set signal pin as output.
	PORT_OUT |= device_pin;
	PORT_DDR |= device_pin;

	for (i = 0; i < count; i++) {
		for (bit_mask = 0x80; bit_mask >= 1; bit_mask >>= 1) {	
			if (bit_mask & buffer[i]) {
				/* drives pin low */
				drive_si_low(device_pin);

				/* delay */
				tLOW1_DLY;
				
				/* release line, convert
				 * gpio to input
				 */
				release_si(device_pin);
				tRCV1_DLY;
			}
			else {
				// Send a zero bit.
				/* drives pin low */
				drive_si_low(device_pin);
				/* delay */
				tLOW0_DLY;
								
				/* release line, convert
				 * gpio to input
				 */
				release_si(device_pin);
				tRCV0_DLY;

			}
		}
		/* for ack/nack byte */
		/* drives pin low */
		drive_si_low(device_pin);
		tDr_DLY;
		
		release_si(device_pin);
		tLOW1_DLY;
		
		// check for ACK/NACK
		if(PORT_IN & device_pin)
		{
			//NAK detected
			release_si(device_pin);
			swi_enable_interrupts();
			
			if(i==0)
			 retCode = SWI_FUNCTION_RETCODE_ADDR_NAK_FAIL;
			else
			 retCode = SWI_FUNCTION_RETCODE_DATA_NAK_FAIL;
			
			return retCode;
		}	
		tRCV0_DLY;
	}
	

	/* always release the line */
	release_si(device_pin);
	
	swi_enable_interrupts();
	return SWI_FUNCTION_RETCODE_SUCCESS;
}


uint8_t swi_write_stdspeed_cmd(const swi_package_t *packet )
{

	uint8_t index;
	uint8_t writeSize = 1;
	uint8_t i, bit_mask;
	uint8_t retCode;
			
	index = 0;
	
	/* load device addr w/ opcode
	 * and clear the R/W bit
	 */
	cmdWriteBuffer[index++] = (((packet->opcode<<4)|packet->dev_addr) & ~0x01);	
	
	/* get memory address */
	if(packet->mem_addr_length)
	{
		/* memory address */
		cmdWriteBuffer[index++] = packet->mem_addr;
		writeSize++;
	}
	
	/* load data to be written */
	memcpy(&cmdWriteBuffer[index],packet->buffer,packet->wlen);
	
	writeSize += packet->wlen;
	
	/* send start condition */
	swi_start_stop_cond();
	
	/* Disable interrupts while sending. */
	swi_disable_interrupts();

	/* Set signal pin as output. */
	PORT_OUT |= device_pin;
	PORT_DDR |= device_pin;

	for (i = 0; i < writeSize; i++) {
		for (bit_mask = 0x80; bit_mask >= 1; bit_mask >>= 1) {	
			if (bit_mask & cmdWriteBuffer[i]) {
				/* drives pin low */
				drive_si_low(device_pin);

				/* delay */
				tLOW1_HDLY;
				
				/* release line, convert
				 * gpio to input
				 */
				release_si(device_pin);
				tRCV1_HDLY;
			}
			else {
				// Send a zero bit.
				/* drives pin low */
				drive_si_low(device_pin);
				/* delay */
				tLOW0_HDLY;
								
				/* release line, convert
				 * gpio to input
				 */
				release_si(device_pin);
				tRCV0_HDLY;

			}
		}
		/* for ack/nack byte */
		/* drives pin low */
		drive_si_low(device_pin);
		tDr_HDLY;
		
		release_si(device_pin);
		tLOW1_HDLY;
		
		// check for ACK/NACK
		if(PORT_IN & device_pin)
		{
			//NAK detected
			release_si(device_pin);
			swi_enable_interrupts();
			
			if(i==0)
			 retCode = SWI_FUNCTION_RETCODE_ADDR_NAK_FAIL;
			else
			 retCode = SWI_FUNCTION_RETCODE_DATA_NAK_FAIL;
			
			return retCode;
		}	
		tRCV0_HDLY;
	}
	

	/* always release the line */
	release_si(device_pin);
	swi_enable_interrupts();
	
	swi_start_stop_cond();

	return SWI_FUNCTION_RETCODE_SUCCESS;
	
}

/** \brief This GPIO function sends bytes to an SWI device.
 * \param[in] count number of bytes to send
 * \param[in] buffer pointer to tx buffer
 * \return status of the operation
 */
uint8_t send_std_speed_cmd(uint8_t count, uint8_t *buffer)
{

	uint8_t i, bit_mask;
    uint8_t retCode;
	// Disable interrupts while sending.
	swi_disable_interrupts();

	// Set signal pin as output.
	PORT_OUT |= device_pin;
	PORT_DDR |= device_pin;

	for (i = 0; i < count; i++) {
		for (bit_mask = 0x80; bit_mask >= 1; bit_mask >>= 1) {	
			if (bit_mask & buffer[i]) {
				/* drives pin low */
				drive_si_low(device_pin);

				/* delay */
				_delay_ns(tLW1_TYPICAL);
				
				/* release line, convert
				 * gpio to input
				 */
				release_si(device_pin);
				_delay_ns(tBIT_TYPICAL - tLW1_TYPICAL);
			}
			else {
				// Send a zero bit.
				/* drives pin low */
				drive_si_low(device_pin);
				/* delay */
				_delay_ns(8000);
								
				/* release line, convert
				 * gpio to input
				 */
				release_si(device_pin);
				_delay_ns(tBIT_TYPICAL - tLWO_TYPICAL);
			}
		}
		/* for ack/nack byte */
		/* drives pin low */
		drive_si_low(device_pin);
		_delay_ns(1000);
		
		release_si(device_pin);
		_delay_ns(tLW1_TYPICAL);
		
		// check for ACK/NACK
		if(PORT_IN & device_pin)
		{
			//NAK detected
			release_si(device_pin);
			swi_enable_interrupts();
			
			if(i==0)
			 retCode = SWI_FUNCTION_RETCODE_ADDR_NAK_FAIL;
			else
			 retCode = SWI_FUNCTION_RETCODE_DATA_NAK_FAIL;
			
			return retCode;
		}	
		_delay_ns(tBIT_TYPICAL - tLWO_TYPICAL);
	}
	

	/* always release the line */
	release_si(device_pin);
	
	swi_enable_interrupts();
	return SWI_FUNCTION_RETCODE_SUCCESS;
}


/** \brief This GPIO function receives bytes from an SWI device.
 *  \param[in] count number of bytes to receive
 *  \param[out] buffer pointer to rx buffer
 * \return status of the operation
 */
uint8_t swi_receive_bytes(uint8_t count, uint8_t *buffer) {
	uint8_t status = SWI_FUNCTION_RETCODE_SUCCESS;
	uint8_t i;
	uint8_t bit_mask;

    
	// Disable interrupts while receiving.
	swi_disable_interrupts();

	memset(&buffer[0],0,count);
	
	// Configure signal pin as input.
	PORT_DDR &= ~device_pin;


	// Data Phase,... Receive bits and store in buffer.
	for (i = 0; i < count; i++) {
		for (bit_mask = 0x80; bit_mask >= 1; bit_mask >>= 1) {

			/* Initiate device write*/
			#ifdef STANDARD_SPEED
				initDeviceWriteStd(device_pin);
			#else
				initDeviceWriteHi(device_pin);
			#endif

			//wait before sampling
			tSWIN_DLY; 
			
			// sample now
			if(PORT_IN & device_pin)
			{
				// received "one" bit
				buffer[i] |= bit_mask;				
			}
			
			//wait tBIT before reading the next byte
			tBIT_DLY;	
		}
		
		/* send ACK byte */
		if(i < (count-1))
		{sendAck();}
	
	}
	
	swi_enable_interrupts();
	return status;
}

void sendAck()
{
   drive_si_low(device_pin);
   tLOW0_DLY;
   release_si(device_pin);
   tBIT_DLY;   
}

uint8_t swi_device_discovery()
{

	/* drives pin low */
	drive_si_low(device_pin);

	/* discharge delay */
	tDSCHG_DLY;
		
	/* release line */
	release_si(device_pin);
	
	tRRT_DLY;

	drive_si_low(device_pin);
	
	tDRR_DLY;

	/* release line */
	release_si(device_pin);	
	
	_delay_us(8);

return (PORT_IN & device_pin);
}


void swi_start_stop_cond()
{
	/* release line */
	release_si(device_pin);
	
	tHTSS_DLY;
}

