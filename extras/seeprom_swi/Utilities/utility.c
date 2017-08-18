/** \file
 *  \brief Data converter functions
 */
#include <string.h>
#include "utility.h"


/** \brief Converting Nible to ASCII
 *
 * \param ucData is nible data to be converted
 *
 * \return ucData is the ASCII value
**/
uint8_t Nible_to_ascii ( uint8_t ucData )
{
    ucData &= 0x0F;
    if (ucData <= 0x09 )
    {
        ucData += '0';
    }
    else
    {
        ucData = ucData - 10 + 'A';
    }
    return ucData;
}

/** \brief Converting ASCII to Nible
 *
 * \param ucData is the ASCII value to be converted
 *
 * \return ucData is the nible value
**/
uint8_t Ascii_to_nible ( uint8_t ucData )
{
    if ((ucData <= '9' ) && (ucData >= '0')) 
    {
        ucData -= '0';
    }
    else if ((ucData <= 'F' ) && (ucData >= 'A')) 
    {
        ucData = ucData -'A' + 10;
    }
    else if ((ucData <= 'f' ) && (ucData >= 'a')) 
    {
        ucData = ucData -'a' + 10;
    }
    else 
    {
        ucData = 0;
    }
    return ucData;
}

/** \brief This function converts a nibble to Hex-ASCII.
 * \param[in] nibble nibble value to be converted
 * \return ASCII value
**/
uint8_t
ConvertNibbleToAscii (uint8_t nibble)
{
  nibble &= 0x0F;
  if (nibble <= 0x09)
    nibble += '0';
  else
    nibble += ('A' - 10);
  return nibble;
}


/** \brief This function converts an ASCII character to a nibble.
 * \param[in] ascii ASCII value to be converted
 * \return nibble value
**/
uint8_t ConvertAsciiToNibble(uint8_t ascii)
{
    if ((ascii <= '9') && (ascii >= '0'))
        ascii -= '0';
    else if ((ascii <= 'F' ) && (ascii >= 'A'))
        ascii -= ('A' - 10);
    else if ((ascii <= 'f') && (ascii >= 'a'))
        ascii -= ('a' - 10);
    else
        ascii = 0;
    return ascii;
}


/** \brief This function converts binary data to ASCII.
 * \param[in] length number of bytes in buffer
 * \param[in, out] buffer pointer to buffer
 * \return number of bytes in buffer
 */
uint8_t ConvertAsciiToBinary(uint8_t length, uint8_t *buffer)
{
	if (length < 2)
		return 0;

	uint8_t i, binIndex;
	uint8_t asciiBuffer[length];

	memcpy(asciiBuffer, buffer, length);

	for (i = 0, binIndex = 0; i < length; i += 2)
	{
		buffer[binIndex] = ConvertAsciiToNibble(asciiBuffer[i]) << 4;
		buffer[binIndex++] |= ConvertAsciiToNibble(asciiBuffer[i + 1]);
	}

	return --binIndex;
}


