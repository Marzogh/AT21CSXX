/** \file
 *  \brief Header file for utility.c.
 */

#ifndef _UTILITY_H_
#define _UTILITY_H_

#include "config.h"

#define ERR_CMD_BUF_FULL    1

void Send_Error_Message( uint8_t ErrorCodes );
uint8_t Ascii_to_nible ( uint8_t ucData );
uint8_t Nible_to_ascii ( uint8_t ucData );
void uart_usb_puthex_array( uint8_t * pBuf, uint8_t ucLength );
void uart_usb_puthex_16 ( uint16_t ucData16 );


uint8_t ConvertNibbleToAscii (uint8_t nibble);
uint8_t ConvertAsciiToNibble (uint8_t ascii);
uint8_t ConvertAsciiToBinary (uint8_t length, uint8_t * buffer);
uint8_t ExtractDataLoad (char *command, uint8_t * dataLength,uint8_t ** data);
uint8_t ExtractOpCode(char *command, uint8_t *dataLength, uint8_t **dataBuf);
#endif // UTILITY_H
