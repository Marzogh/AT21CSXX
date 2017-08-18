/** \file
 *  \brief Implementation of SPI functions.
 */



#include "spi.h"
#include "hardware.h"


/** \brief This function initializes SPI peripheral.
 */
void Spi_initialize()
{
    //volatile uchar ucIOReg;

    // Set MOSI, SS, SCK, as output, all others input
    DDRB = (1 << PB2) | (1 << PB1) | (1 << PB0);

    // Set SS high and SCK low
    Spi_ss_high;
    Spi_sck_low;

    //  Enable SPI, Master, mode 0, set clock rate
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1);    // fck/64

    // clear SPIF bit in SPSR
    //ucIOReg = SPSR;
    //ucIOReg = SPDR;
}




/** \brief Send and receive data on SPI.
 *
 * \param ucData is data which will be sent using SPI.
 * \return SPDR is data received from SPI slave.
 */
uchar Spi_send_and_receive ( uchar ucData )
{
    /* Start transmission */
    SPDR  = ucData ;     // Send Character

    /* Wait for transmission complete */

    while (!(SPSR & (1 << SPIF)))
    {
        // Wait until Char is sent
    }

    /* Return the received byte*/

    return SPDR;
}


