/** \file config.h
 *  \brief This file contains the system configuration definition
 *
 *  Copyright (c) 2006 Atmel.
 *
 *  \version 1.1 at90usb128-demo-hidgen-1_0_2 $Id: config.h,v 1.1 2006/03/17 13:06:36 
 *   rletendu Exp $
 */

#ifndef _CONFIG_H_
#define _CONFIG_H_

//_____ I N C L U D E S ____________________________________________________
#include <avr/interrupt.h>

/** \brief Endian type
 */
#define LITTLE_ENDIAN
#define FOSC 16000

//#define TRUE		(1)	//!< definition for boolean type true
//#define FALSE		(0)	//!< definition for boolean type false
#define ENABLE		(1)	//!< definition for enable
#define DISABLE		(0)	//!< definition for disable

//! USB send and receive data buffer size
#define USB_BUFFER_SIZE 255

/** \brief Basic Datatypes */
typedef unsigned char   uchar;
typedef unsigned char*  puchar;
typedef signed char     schar;
typedef signed char*    pschar;
typedef unsigned short  ushort;
typedef unsigned short* pushort;
typedef signed short    sushort;
typedef signed short*   pshort;
typedef unsigned int    uint;
typedef unsigned int*   puint;
typedef signed int      sint;
typedef signed int*     psint;

/******************************************************************************/
/* GCC COMPILER                                                               */
/******************************************************************************/
#ifdef __GNUC__
    #define _ConstType_ __flash
    #define _MemType_
    #define _GenericType_ __generic
    #define code PROGMEM
    #define xdata
    #define idata
    #define data
    #define At(x) @ x
    #define pdata
    #define bdata
//    #define bit uchar
    #define Bool uchar
    #define Enable_interrupt() sei()
    #define Disable_interrupt() cli()
	#define Get_interrupt_state() (SREG&0x80)
#endif

// U16/U32 endian handlers
#ifdef LITTLE_ENDIAN
// LITTLE_ENDIAN => 16bit: (LSB,MSB), 32bit: (LSW,MSW)
// or (LSB0,LSB1,LSB2,LSB3) or (MSB3,MSB2,MSB1,MSB0)
//     #define MSB(u16)        (((puchar ) &u16)[1])
//     #define LSB(u16)        (((puchar ) &u16)[0])
//     #define MSW(u32)        (((pushort) &u32)[1])
//     #define LSW(u32)        (((pushort) &u32)[0])
//     #define MSB0(u32)       (((puchar ) &u32)[3])
//     #define MSB1(u32)       (((puchar ) &u32)[2])
//     #define MSB2(u32)       (((puchar ) &u32)[1])
//     #define MSB3(u32)       (((puchar ) &u32)[0])
    #define LSB0(u32)       MSB3(u32)
    #define LSB1(u32)       MSB2(u32)
    #define LSB2(u32)       MSB1(u32)
    #define LSB3(u32)       MSB0(u32)
#else
// BIG_ENDIAN => 16bit: (MSB,LSB), 32bit: (MSW,LSW) 
// or (LSB3,LSB2,LSB1,LSB0) or (MSB0,MSB1,MSB2,MSB3)
    #define MSB(u16)        (((puchar ) &u16)[0])
    #define LSB(u16)        (((puchar ) &u16)[1])
    #define MSW(u32)        (((pushort) &u32)[0])
    #define LSW(u32)        (((pushort) &u32)[1])
    #define MSB0(u32)       (((puchar ) &u32)[0])
    #define MSB1(u32)       (((puchar ) &u32)[1])
    #define MSB2(u32)       (((puchar ) &u32)[2])
    #define MSB3(u32)       (((puchar ) &u32)[3])
    #define LSB0(u32)       MSB3(u32)
    #define LSB1(u32)       MSB2(u32)
    #define LSB2(u32)       MSB1(u32)
    #define LSB3(u32)       MSB0(u32)
#endif

/** \brief ENABLE Constant definition.
 */
//#define ENABLE   1

/**  \brief DISABLE Constant definition.
 */
//#define DISABLE  0

/** \brief Fail to send a command error code definition.
 */
#ifndef FAIL_COMMAND_SEND
#define FAIL_COMMAND_SEND (0x03)
#endif

/** \brief General fail error code definition.
 */
#ifndef FAILED
#define FAILED 0x01
#endif

/** \brief Unknown command error code definition.
 */
#ifndef UNKNOWN_COMMAND
#define UNKNOWN_COMMAND (0x21)
#endif

#endif // _CONFIG_H_

