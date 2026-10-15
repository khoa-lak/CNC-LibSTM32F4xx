/*
  dual_stream.h - Composite stream handler for concurrent USB CDC + UART
  Part of grblHAL for STM32F4xx
*/

#ifndef _DUAL_STREAM_H_
#define _DUAL_STREAM_H_

#include "grbl/stream.h"

const io_stream_t *dual_stream_init (const io_stream_t *usb, const io_stream_t *uart);

#endif // _DUAL_STREAM_H_
