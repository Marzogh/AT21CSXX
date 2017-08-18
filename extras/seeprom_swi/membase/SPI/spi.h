/** \file
 *  \brief Header file for spi.c.
 */

#ifndef _SPI_H_
#define _SPI_H_

#include "config.h"


/** \defgroup SPI_Macro SPI Macro for AT90USB646/7 and AT90USB1286/7
 *  \{
 */
#if defined (__AVR_AT90USB646__) || defined (__AVR_AT90USB647__) || \
      defined (__AVR_AT90USB1286__) || defined (__AVR_AT90USB1287__)
   #define Spi_sck_high  PORTB |= (1 << PB1)
   #define Spi_sck_low   PORTB &= ~(1 << PB1)
   #define Spi_ss_high   PORTB |= (1 << PB0)
   #define Spi_ss_low    PORTB &= ~(1 << PB0)
   //#define Spi_timeout   ((F_CPU / 1000 / 1024) * 45)
   #define Spi_timeout   ((F_CPU / 1000 / 1024) * 60)
#endif

void  Spi_initialize();
uchar Spi_send_and_receive(uchar ucData);

#endif // _SPI_H_
