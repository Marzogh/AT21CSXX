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
 *  \author 
 *  \date 	
 * 
 *    
 */
#ifndef SWI_PHYS_H
#   define SWI_PHYS_H

#include <stdint.h>                                       // data type definitions
#include "delay_x.h"               // software delay functions for the AVR CPU

#define swi_enable_interrupts  sei //!< enable interrupts
#define swi_disable_interrupts cli //!< disable interrupts

#define PORT_DDR         (DDRD)     //!< direction register for device id 0
#define PORT_OUT         (PORTD)    //!< output port register for device id 0
#define PORT_IN          (PIND)     //!< input port register for device id 0

#define INPUT	0
#define OUTPUT	1


// error codes for physical hardware dependent module
#define SWI_FUNCTION_RETCODE_SUCCESS			((uint8_t) 0x00) //!< Communication with device succeeded.
#define SWI_FUNCTION_RETCODE_TIMEOUT			((uint8_t) 0xF1) //!< Communication timed out.
#define SWI_FUNCTION_RETCODE_RX_FAIL			((uint8_t) 0xF9) //!< Communication failed after at least one byte was received.
#define SWI_FUNCTION_RETCODE_ADDR_NAK_FAIL		((uint8_t) 0xF2) //!< NAK during address.
#define SWI_FUNCTION_RETCODE_DATA_NAK_FAIL		((uint8_t) 0xF3) //!< Communication failed after at least one byte was received.

//#define STANDARD_SPEED

#if defined STANDARD_SPEED

/* Standard Speed
 * Note: These times have
 *       been converted to (us)
 */
#define tLOW0_MIN			24000
#define tLOW0_MAX			64000
#define tLOW1_MIN			4000
#define tLOW1_MAX			8000
#define tBIT_MIN			40000			
#define tBIT_MAX			100000			
#define tRCV_MIN			8000
#define tDSCHG				200000 // min per spec (150us)
#define tRESET              500000 // min per spec (480us)
#define tRRT				10000  // min per spec (n/a)
#define tDRR                1000   // min per spec (n/a)
#define tMSDR               5000   // min per spec (n/a)
#define tHTSS               700000

#define tLWO_TYPICAL       (tLOW0_MIN +((tLOW0_MAX-tLOW0_MIN)/2))
#define tLW1_TYPICAL       (tLOW1_MIN +((tLOW1_MAX-tLOW1_MIN)/2))
#define tBIT_TYPICAL	   (tBIT_MIN +((tBIT_MAX-tBIT_MIN)/2))

#define tLOW0_DLY			_delay_ns(tLWO_TYPICAL)
#define tLOW1_DLY			_delay_ns(tLW1_TYPICAL)

#define tBIT_DLY			_delay_ns(tBIT_TYPICAL)
#define tRCV0_DLY			_delay_ns(tBIT_TYPICAL - tLWO_TYPICAL)
#define tRCV1_DLY			_delay_ns(tBIT_TYPICAL - tLW1_TYPICAL)

#define tDr_DLY				_delay_ns(6000)
#define tSWIN_DLY           _delay_ns(1500)
#define tRRT_DLY			_delay_ns(tRRT)
#define tDRR_DLY			_delay_ns(tDRR)
#define tMSDR_DLY			_delay_ns(tMSDR)
#define tDSCHG_DLY			_delay_ns(tDSCHG)
#define tDRESET_DLY			_delay_ns(tRESET)
#define tHTSS_DLY           _delay_ns(tHTSS)


#else 

/* High Speed
 * Note: These times have
 *       been converted to (ns)
 */
#define tLOW0_MIN			6000
#define tLOW0_MAX			16000
#define tLOW1_MIN			1000
#define tLOW1_MAX			2000
#define tBIT_MIN			6000
#define tBIT_MAX			25000
#define tRCV_MIN			2000
#define tDSCHG_				200000 // min per spec (150us)
#define tRESET_				50000  // min per spec (480us)
#define tRRT_				10000  // min per spec (n/a)
#define tDRR_				1000   // min per spec (n/a)
#define tMSDR_				2000   // min per spec (n/a)
#define tHTSS_				200000

#define tLWO_TYPICAL			(tLOW0_MIN +((tLOW0_MAX-tLOW0_MIN)/2))
#define tLW1_TYPICAL	        (tLOW1_MIN +((tLOW1_MAX-tLOW1_MIN)/2))
#define tBIT_TYPICAL		    (tBIT_MIN +((tBIT_MAX-tBIT_MIN)/2))

#define tLOW0_DLY				_delay_ns(tLWO_TYPICAL)
#define tLOW1_DLY				_delay_ns(tLW1_TYPICAL)

#define tBIT_DLY				_delay_ns(tBIT_TYPICAL)
#define tRCV0_DLY				_delay_ns(tBIT_TYPICAL - tLWO_TYPICAL)
#define tRCV1_DLY				_delay_ns(tBIT_TYPICAL - tLW1_TYPICAL)

// DELAYS
#define tDr_DLY					_delay_ns(1000)
#define tSWIN_DLY               _delay_ns(1500)
#define tRRT_DLY				_delay_ns(tRRT_)
#define tDRR_DLY				_delay_ns(tDRR_)
#define tMSDR_DLY				_delay_ns(tMSDR_)
#define tDSCHG_DLY				_delay_ns(tDSCHG_)
#define tDRESET_DLY				_delay_ns(tRESET_)
#define tHTSS_DLY               _delay_ns(tHTSS_)

#endif

#define tLOW0_HDLY				_delay_ns(8000)
#define tDr_HDLY				_delay_ns(1000)
#define tLOW1_HDLY				_delay_ns(1500)
#define tRCV0_HDLY				_delay_ns(11000)
#define tRCV1_HDLY				_delay_ns(14000)



#define drive_si_low(si_pin)    {PORT_DDR |= si_pin; PORT_OUT &= ~si_pin;}
#define release_si(si_pin)      {PORT_DDR &= ~si_pin;}

/**
 * \brief Information concerning the data transmission
 */
typedef struct
{
	//! SWI chip address to communicate with.
	uint8_t dev_addr;
	//! op code
	uint8_t opcode;
	//! SWI address/commands to issue to the other chip (node).
	uint8_t mem_addr;
	//! .
	uint8_t mem_addr_length;
	//! Where to find the data.
	void *buffer;		
	//! How many bytes do we want to write.
	uint16_t wlen;
	//! How many bytes do we want to read.
	uint16_t rlen;	
	//! This flag tells the low level drive
	//  to check for ack only
	uint8_t chk_ack_only_flag;
	
}
__attribute__ ((packed)) swi_package_t;

// Function Prototypes
void    swi_enable(void);
void    swi_set_device_id(uint8_t id);
void    swi_set_signal_pin(uint8_t end);
void    sendAck(void);
void    swi_start_stop_cond(void);

uint8_t swi_send_bytes(uint8_t count, uint8_t *buffer);
uint8_t swi_send_byte(uint8_t value);
uint8_t swi_receive_bytes(uint8_t count, uint8_t *buffer);
uint8_t swi_device_discovery(void);
uint8_t swi_read(const swi_package_t *pkg );
uint8_t swi_write(const swi_package_t *pkg );
void selectSpeed(uint8_t speed);
uint8_t swi_write_stdspeed_cmd(const swi_package_t *packet );

#endif
