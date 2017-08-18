#include <avr/wdt.h>
#include <util/delay.h>

#include "config.h"
#include "spi.h"
#include "hardware.h"
#include "swi_com.h"
#include "string.h"
#include "swi_phy.h"

#define Clear_prescaler()   (CLKPR = (1<<CLKPCE),CLKPR = 0)

/* Device represents the low nibble
 * device address
 */
//#define DEVICE_ADDR 0x00
uint8_t DEVICE_ADDR;

enum test{EEPROM, SECURITY, SERIALNUMBER, LOCKSECURITYREG, CHCKLOCKSECREG,
	      LOCKROMREG, READROMREGACCESS, FREEZEROMZNSTATE, READMFGID,
		  CHCKSTDSPEEDMODE, SETSTDSPEED, CHCKHIGHSPEEDMODE, SETHIGHSPEED,
		  DISCOVERY,NO_DEVICE};
		  
const uint8_t eepromTestWrite[] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
								   0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F};

	
const uint8_t secTestWrite[]    = {0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10};
			
int main(void)
{
	static uint8_t rbuffer[32];
	static uint8_t wbuffer[32];
	static uint8_t FailedTest = 0xFF;
	static uint8_t status = AT21CS01_SWI_GENERAL_ERROR;

	DEVICE_ADDR = 0x00;
	
	/* Turn off watchdog */
	wdt_disable();

	/* For USB */
	Clear_prescaler();

	/* Initialize GPIO for three 
	 *  LEDs on MemBase board.
	 */
	LEDS_INIT;
	LED1_OFF;

	
	LED1_ON;
	LED2_OFF;
	LED3_OFF;

	/* Initialize hardware components*/
	Spi_initialize();

	/* Note: Dut_VCC is controlled by a DAC */
	
	/* Configure DAC control signals as output */
	gpio_set_pin_dir_output(ADCCS_DDR,ADCCS);
	gpio_set_pin_dir_output(LDAC_DDR,LDAC);
	gpio_set_pin_dir_output(CSDAC_DDR,CSDAC);

	/* Initialize the DAC control signals */		
	gpio_set_pin_high(LDAC_PORT,LDAC);
	gpio_set_pin_high(CSDAC_PORT,CSDAC);
	gpio_set_pin_high(ADCCS_PORT,ADCCS);
	
	/* DUT VCC is controlled by a 
	 * DAC circuit
	 * set to 3.32V
	 */
	set_vcc_level(0x7B00);
		
	Enable_interrupt();


	/* 
		//execute discovery
		if(swi_device_discovery() != SWI_SUCCESS)
		{
			// failed
			{FailedTest = DISCOVERY; goto _FAILED_;}	
		}
	*/
		
	/* performs discovery and scan for address*/	
	if((DEVICE_ADDR = scan_swi_device_addr()) == NO_DEVICE_DETECTED)
	{
		// failed
		{FailedTest = NO_DEVICE; goto _FAILED_;}	
	}

	#ifdef STANDARD_SPEED
	
	/**
	*
	* @Description: Set STD SPEED MODE (0x0D)  
	*               
	*
	* @param rbuffer - NOT USED
	*
	* @retval SWI_SUCCESS if successful, otherwise error(was not set)               
	* 	 
	*****************************************************************************/		
	status = set_std_speed_mode(DEVICE_ADDR, &rbuffer[0]);
		
	/* check operation status */
	if(status != SWI_SUCCESS)
		{FailedTest = SETSTDSPEED; goto _FAILED_;}	
	
	#endif	


	/****************************************************************
   	 ******************* EEPROM ACCESS (0x0A)     *******************
	 *******************      Page Wtite          *******************
   	 ***************************************************************/	
	 memcpy(wbuffer,eepromTestWrite,0x08);

	/* write page (8bytes) to EEPROM starting at address 0x00 */
	if(write_memory(DEVICE_ADDR,0x00, 0x08, &wbuffer[0])== SWI_SUCCESS)
	{
		
			
		/* clear before reading */
		memset(rbuffer,0,sizeof(rbuffer));
		status = read_memory(DEVICE_ADDR, 0x00, 0x08, &rbuffer[0]);	
			
		/* verifying write was successful */
		for(uint8_t i = 0; i < 0x08; i++)
		{
			if(rbuffer[i] != wbuffer[i])
			{
				/* break on error */
				status = AT21CS01_SWI_GENERAL_ERROR;
				break;
			}
		}	
	}


	/* check operation status */
	if(status != SWI_SUCCESS)
		{FailedTest = EEPROM; goto _FAILED_;}		
	
	
	/****************************************************************	                
					    EEPROM ACCESS (0x0A) 
						   
	 This function allows user to send a buffer of data, w/o regards
	 to the device page boundaries
   	 ***************************************************************/
	 memcpy(wbuffer,eepromTestWrite,sizeof(eepromTestWrite));

	if(write_memory_buffer(DEVICE_ADDR, 0x00, sizeof(eepromTestWrite), &wbuffer[0]) == SWI_SUCCESS)
	{
		
		/* clear before reading */
		memset(rbuffer,0,sizeof(rbuffer));
		
		status = read_memory(DEVICE_ADDR, 0x00, sizeof(eepromTestWrite), &rbuffer[0]);
		
		/* verifying write was successful */
		for(uint8_t i = 0; i < sizeof(eepromTestWrite); i++)
		{
			if(rbuffer[i] != wbuffer[i])
			{
				/* break on error */
				status = AT21CS01_SWI_GENERAL_ERROR;
				break;
			}
		}
	}

	/* check operation status */
	if(status != SWI_SUCCESS)
		{FailedTest = EEPROM; goto _FAILED_;}
			
	/****************************************************************
	SECURITY REGISTER ACCESS (0x0B)
	****************************************************************/
	memcpy(wbuffer,secTestWrite,sizeof(secTestWrite));
	
	status = write_security_register(DEVICE_ADDR, 0x10, 0x08, &wbuffer[0]);
	
	status = read_security_register(DEVICE_ADDR, 0x10, 0x08, &rbuffer[0]);	
	
	/* verifying data */
	for(uint8_t i = 0; i < 8; i++)
	{
		if(rbuffer[i] != wbuffer[i])
		{
			/* break on error */
			status = AT21CS01_SWI_GENERAL_ERROR;
			break;
		}
	}	

	/* check operation status */
	if(status != SWI_SUCCESS)
		{FailedTest = SECURITY; goto _FAILED_;}
			
			
	/****************************************************************
	READ SERIAL NUMBER, SECURITY REGISTER ACCESS (0x0B)
	****************************************************************/
	/* clear before reading */
	memset(rbuffer,0,sizeof(rbuffer));	
	
	status = read_security_register(DEVICE_ADDR, 0x00, 0x08, &rbuffer[0]);	

	/* check status of operation */
	if(status != SWI_SUCCESS)
		{FailedTest = SERIALNUMBER; goto _FAILED_;}		
			
			   
	/****************************************************************
	LOCK SECURITY REGISTER (0x02)
	****************************************************************/
	/**
	 *
	 * @Description: LOCK SECURITY REGISTER (0x02)
	 *               
     *
	 *
	 * @param wbuffer - Dummy byte is required to be sent.
	 *
	 * @retval status. 00 : zone has not been set as ROM
	 *                 FF : zone has been set to TOM and cannot be altered		
	 *****************************************************************************/		
	 if(Is_hwb())
	 {
		 /* Debugging use only,
		    Execute this fcn only if 
			the HWB button is held down (pressed)
	     */
		 status = lock_security_register(DEVICE_ADDR, &wbuffer[0]);
		 
		/* check operation status */
		if(status != SWI_SUCCESS)
			{FailedTest = LOCKSECURITYREG; goto _FAILED_;}		 		 
	 }



	/**
	 *
	 * @Description: Check LOCK SECURITY REGISTER (0x02)
	 *               Reading the status of a ROM Zone Register
     *
	 *
	 * @param rbuffer - NOT USED.
	 *
	 * @retval status. ACK  : 0x00, indicates the lock has NOT been set
	 *                 NACK : 0xF2, indicates the Lock has been set
	 *****************************************************************************/		
     status = check_lock_command(DEVICE_ADDR, &rbuffer[0]);			
 		
	/**
	 *
	 * @Description: Lock ROM ZONE REGISTER ACCESS (0x07) 
	 *               This command locks a particular Memory Zone	
     *
	 *
	 * @param wbuffer - Holds DataIn Byte "FF", which is required for this operation
	 *
	 * @retval SWI_SUCCESS if successful, otherwise error (Lock operation will not be performed) 
	 *
	 *****************************************************************************/	
	
	 if(Is_hwb())
	 {
		 /* Debugging use only,
		    Execute this fcn only if 
			the HWB button is held down (pressed)
	     */	
		 
		wbuffer[0]= 0xFF;
		status = SWI_SUCCESS;
		
		/* Memory Zone 1 */	   
		status |= writing_rom_zone_register(DEVICE_ADDR, ROMZONEREG_MEMZONE0, &wbuffer[0]);
		/* Memory Zone 1 */
		status |= writing_rom_zone_register(DEVICE_ADDR, ROMZONEREG_MEMZONE1, &wbuffer[0]);
		/* Memory Zone 1 */
		status |= writing_rom_zone_register(DEVICE_ADDR, ROMZONEREG_MEMZONE2, &wbuffer[0]);
		/* Memory Zone 1 */
		status |= writing_rom_zone_register(DEVICE_ADDR, ROMZONEREG_MEMZONE3, &wbuffer[0]);
		  
		/* check operation status */
		if(status != SWI_SUCCESS)
			{FailedTest = LOCKROMREG; goto _FAILED_;}		  
	 }
   
	/**
	 *
	 * @Description: Read ROM ZONE REGISTER ACCESS (0x07)
	 *               Reading the status of a ROM Zone Register
     *
	 *
	 * @param rbuffer[0] - Holds ROM zone lock status
	 *                     00 : zone has not been set as ROM
	 *                     FF : zone has been set to ROM and cannot be altered	 
	 *
	 * @retval SWI_SUCCESS if successful, otherwise error
	 *****************************************************************************/	
	status = SWI_SUCCESS;		
	/* Memory Zone 1 */
	status |= reading_rom_zone_register(DEVICE_ADDR, ROMZONEREG_MEMZONE0, &rbuffer[0]);
	/* Memory Zone 2 */
	status |= reading_rom_zone_register(DEVICE_ADDR, ROMZONEREG_MEMZONE1, &rbuffer[0]);		
	/* Memory Zone 3 */
	status |= reading_rom_zone_register(DEVICE_ADDR, ROMZONEREG_MEMZONE2, &rbuffer[0]);
	/* Memory Zone 4 */
	status |= reading_rom_zone_register(DEVICE_ADDR, ROMZONEREG_MEMZONE3, &rbuffer[0]);
		  
		
	/**
	 *
	 * @Description: FREEZE ROM ZONE STATE (0x01) 
	 *               	
     *
	 *
	 * @param wbuffer - Holds DataIn Byte "AA", which is required for this operation
	 *
	 * @retval SWI_SUCCESS if successful, otherwise error (Freeze operation will not be performed)  
	 *     
	 *****************************************************************************/	
	 if(Is_hwb())
	 {
		 /* Debugging use only,
		    Execute this fcn only if 
			the HWB button is held down (pressed)
	     */			
		wbuffer[0]= 0xAA;
		status = freeze_rom_zone_state(DEVICE_ADDR, &wbuffer[0]);
		
		/* check operation status */
		if(status != SWI_SUCCESS)
		 {FailedTest = FREEZEROMZNSTATE; goto _FAILED_;}		
		
	 }
   
	/**
	 *
	 * @Description: READ MFG ID (0x0C) 
	 *               
     *
	 * @param rbuffer - mfg id stored in buffer
	 *
	 * @retval SWI_SUCCESS if successful, otherwise error            
     * 	 
	 *****************************************************************************/		
	status = read_mfg_id(DEVICE_ADDR, &rbuffer[0]);	 
	
	/* check operation status */
	if(status != SWI_SUCCESS)
		{FailedTest = READMFGID; goto _FAILED_;}   		

	/**
	 *
	 * @Description: Check STD SPEED MODE (0x0D)   
	 *               
     *
	 * @param rbuffer - NOT USED
	 *
	 * @retval SWI_SUCCESS: device is in STD mode
	 * @retval SWI_FUNCTION_RETCODE_ADDR_NAK_FAIL : device not in STD SPEED mode               
     * 	 
	 *****************************************************************************/			
	status = chk_std_speed_mode(DEVICE_ADDR, &rbuffer[0]);
	 	

	/**
	 *
	 * @Description: Check HIGH SPEED MODE (0x0E)
	 *               
     *
	 * @param rbuffer - NOT USED
	 *
	 * @retval SWI_SUCCESS: device is in HIGH SPEED mode
	 * @retval SWI_FUNCTION_RETCODE_ADDR_NAK_FAIL : device NOT in HIGH SPEED mode            
     * 	 
	 *****************************************************************************/		
	status = chk_high_speed_mode(DEVICE_ADDR, &rbuffer[0]);	
	
	/**
	 *
	 * @Description: Set HIGH SPEED MODE (0x0E)
	 *               
     *
	 * @param wbuffer - NOT USED
	 *
	 * @retval SWI_SUCCESS if successful, otherwise error(was not set)           
     * 	 	
	 *****************************************************************************/	
	 if(Is_hwb())
	 {
		 /* Debugging use only,
		    Execute this fcn only if 
			the HWB button is held down (pressed)
	     */			 
		status = set_high_speed_mode(DEVICE_ADDR, &wbuffer[0]);
		
		/* check operation status */
		if(status != SWI_SUCCESS)
			{FailedTest = CHCKHIGHSPEEDMODE; goto _FAILED_;}		
	 }



_FAILED_:

if(status == SWI_SUCCESS)
{
	/* 2 lights, indicates all tests PASSED */
	LED1_ON;
	LED2_ON;
	LED3_OFF;
}
else
{
	/* 3 lights ON indicates a test has failed */
	LED1_ON;
	LED2_ON;
	LED3_ON;
}

 asm("NOP");
	
    return 0;
}
