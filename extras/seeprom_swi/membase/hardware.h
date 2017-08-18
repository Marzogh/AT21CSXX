/** \file
 *  \brief Header file with port and pin definitions of MemBase board.
 */

#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#include "config.h"
#include "compiler.h"

//_____ M A C R O S ________________________________________________________

//! @defgroup CONTROLLER_BRD Hardware Definitions for
//! This module contains low level hardware abstraction layer for CONTROLLER_BRD board
//! @{


#define ADCCS 		         PC6  //OUTPUT
#define ADCCS_DDR 		     DDRC  
#define ADCCS_PORT			 PORTC

#define GPIO1                PF1  //INPUT	
#define GPIO1_DDR 		     DDRF
#define GPIO1_PORT			 PORTF
	
#define LDAC 		         PE6  //OUTPUT
#define LDAC_DDR 		     DDRE
#define LDAC_PORT			 PORTE

#define CSDAC 		         PB6  //OUTPUT
#define CSDAC_DDR 		     DDRB
#define CSDAC_PORT			 PORTB

#define SWI_GPIO 		     PD1  //OUTPUT
#define GPIO0                PD2  //(?, leave as input for now)
#define ISENSE               PF0  //INPUT
		

// SET CONTROL SIGNALS DIRECTION		  
			  
//! @defgroup LED LEDs Management
//! Macros to manage Leds on CONTROLLER_BRD
//! @{	
#define  LED_PORT             PORTD
#define  LED_DDR              DDRD
#define  LED_PIN              PIND

#define  LED1_BIT             PD6
#define  LED2_BIT             PD5
#define  LED3_BIT             PD4 
#define  BUZZER_BIT           PD7

#define  LEDS_INIT          (LED_DDR  |=  (1<<LED1_BIT) | (1<<LED2_BIT) |(1<<LED3_BIT))
#define  LED_ALL_ON         (LED_PORT |=  (1<<LED1_BIT) | (1<<LED2_BIT)| (1<<LED3_BIT))
#define  LED_ALL_OFF        (LED_PORT &= ~((1<<LED1_BIT) | (1<<LED2_BIT)| (1<<LED3_BIT)))

#define  LED1_ON            (LED_PORT |=  (1<<LED1_BIT))
#define  LED2_ON            (LED_PORT |=  (1<<LED2_BIT))
#define  LED3_ON            (LED_PORT |=  (1<<LED3_BIT))

#define  LED1_OFF           (LED_PORT &= ~(1<<LED1_BIT))
#define  LED2_OFF           (LED_PORT &= ~(1<<LED2_BIT))
#define  LED3_OFF           (LED_PORT &= ~(1<<LED3_BIT))

#define  LED1_toggle()      (LED_PIN  |=  (1<<LED1_BIT))
#define  LED2_toggle()      (LED_PIN  |=  (1<<LED2_BIT))
#define  LED3_toggle()      (LED_PIN  |=  (1<<LED3_BIT))

#define  Is_LED1_ON()       (LED_PIN  &   (1<<LED1_BIT) ? TRUE : FALSE)
#define  Is_LED2_ON()       (LED_PIN  &   (1<<LED2_BIT) ? TRUE : FALSE)
#define  Is_LED3_ON()       (LED_PIN  &   (1<<LED3_BIT) ? TRUE : FALSE)
	    //! @}


//! @defgroup 
//! Macros to manage 
//! @{
#define INIT_SPI()   (DDRB   |=  (1<<RESET_BOWLINE) | (1<<ISEL ))
#define INIT_TWI()   (DDRB   |=  (1<<RESET_BOWLINE) | (1<<ISEL ) |(1<<ADDR))


//! @}

//! @defgroup USB_key_HWB HWB button management
//! HWB button is connected to PE2 and can also
//! be used as generic push button
//! @{
#define  Hwb_button_init()    (DDRE  &= ~(1<<PINE2), PORTE |= (1<<PINE2))
#define  Is_hwb()             ((PINE &   (1<<PINE2)) ?  FALSE : TRUE)
#define  Is_not_hwb()         ((PINE &   (1<<PINE2)) ?  TRUE : FALSE)
//! @}
 
#define gpio_set_pin_high(port,pin) (port |= (1<<pin))
#define gpio_set_pin_low(port,pin)  (port &= ~(1<<pin))
#define gpio_set_pin_dir_output(port,pin) (port |= (1<<pin))		
  
//! @}


#endif
