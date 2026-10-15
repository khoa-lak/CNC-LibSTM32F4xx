/*
  dual_stream.c - Composite stream handler for concurrent USB CDC + UART
  Part of grblHAL for STM32F4xx
*/

#include "dual_stream.h"
#include "grbl/grbl.h"
#include <string.h>

static const io_stream_t *s_usb = NULL;
static const io_stream_t *s_uart = NULL;
static uint8_t s_active_ch = 0; // 0: None, 1: USB, 2: UART

static bool dualIsConnected (void)
{
    bool conn = false;
    if(s_usb && s_usb->is_connected)
        conn |= s_usb->is_connected();
    if(s_uart && s_uart->is_connected)
        conn |= s_uart->is_connected();
    return conn || (s_uart != NULL);
}

static uint16_t dualGetRxBufferFree (void)
{
    uint16_t u = (s_usb && s_usb->get_rx_buffer_free) ? s_usb->get_rx_buffer_free() : 255;
    uint16_t s = (s_uart && s_uart->get_rx_buffer_free) ? s_uart->get_rx_buffer_free() : 255;
    return (u < s) ? u : s;
}

static void dualWriteS (const char *s)
{
    if(s_usb && s_usb->is_connected && s_usb->is_connected() && s_usb->write)
        s_usb->write(s);
    if(s_uart && s_uart->write)
        s_uart->write(s);
}

static void dualWriteAll (const char *s)
{
    dualWriteS(s);
}

static bool dualPutC (const uint8_t c)
{
    if(s_usb && s_usb->is_connected && s_usb->is_connected() && s_usb->write_char)
        s_usb->write_char(c);
    if(s_uart && s_uart->write_char)
        return s_uart->write_char(c);
    return true;
}

static void dualWriteN (const uint8_t *s, uint16_t length)
{
    if(s_usb && s_usb->is_connected && s_usb->is_connected() && s_usb->write_n)
        s_usb->write_n(s, length);
    if(s_uart && s_uart->write_n)
        s_uart->write_n(s, length);
}

static int32_t dualGetC (void)
{
    // If locked to a channel mid-line, read only from that channel until newline
    if(s_active_ch == 1) {
        if(s_usb && s_usb->read) {
            int32_t c = s_usb->read();
            if(c != -1) {
                if(c == '\n' || c == '\r')
                    s_active_ch = 0;
                return c;
            }
        }
        return -1;
    }

    if(s_active_ch == 2) {
        if(s_uart && s_uart->read) {
            int32_t c = s_uart->read();
            if(c != -1) {
                if(c == '\n' || c == '\r')
                    s_active_ch = 0;
                return c;
            }
        }
        return -1;
    }

    // Both idle: prioritize USB if available, otherwise UART
    if(s_usb && s_usb->read) {
        int32_t c = s_usb->read();
        if(c != -1) {
            if(c != '\n' && c != '\r')
                s_active_ch = 1;
            return c;
        }
    }

    if(s_uart && s_uart->read) {
        int32_t c = s_uart->read();
        if(c != -1) {
            if(c != '\n' && c != '\r')
                s_active_ch = 2;
            return c;
        }
    }

    return -1;
}

static void dualResetReadBuffer (void)
{
    s_active_ch = 0;
    if(s_usb && s_usb->reset_read_buffer)
        s_usb->reset_read_buffer();
    if(s_uart && s_uart->reset_read_buffer)
        s_uart->reset_read_buffer();
}

static void dualCancelReadBuffer (void)
{
    s_active_ch = 0;
    if(s_usb && s_usb->cancel_read_buffer)
        s_usb->cancel_read_buffer();
    if(s_uart && s_uart->cancel_read_buffer)
        s_uart->cancel_read_buffer();
}

static enqueue_realtime_command_ptr dualSetRtHandler (enqueue_realtime_command_ptr handler)
{
    if(s_usb && s_usb->set_enqueue_rt_handler)
        s_usb->set_enqueue_rt_handler(handler);
    if(s_uart && s_uart->set_enqueue_rt_handler)
        s_uart->set_enqueue_rt_handler(handler);
    return handler;
}

static bool dualSuspendRead (bool suspend)
{
    bool ok = true;
    if(s_usb && s_usb->suspend_read)
        ok &= s_usb->suspend_read(suspend);
    if(s_uart && s_uart->suspend_read)
        ok &= s_uart->suspend_read(suspend);
    return ok;
}

static bool dualEnqueueRtCommand (uint8_t c)
{
    if(s_usb && s_usb->enqueue_rt_command)
        s_usb->enqueue_rt_command(c);
    if(s_uart && s_uart->enqueue_rt_command)
        s_uart->enqueue_rt_command(c);
    return true;
}

const io_stream_t *dual_stream_init (const io_stream_t *usb, const io_stream_t *uart)
{
    static io_stream_t stream;
    memset(&stream, 0, sizeof(io_stream_t));

    s_usb = usb;
    s_uart = uart;

    stream.type = StreamType_Serial;
    stream.state.is_usb = 1;
    stream.state.linestate_event = 1;
    stream.is_connected = dualIsConnected;
    stream.read = dualGetC;
    stream.write = dualWriteS;
    stream.write_all = dualWriteAll;
    stream.write_char = dualPutC;
    stream.write_n = dualWriteN;
    stream.get_rx_buffer_free = dualGetRxBufferFree;
    stream.reset_read_buffer = dualResetReadBuffer;
    stream.cancel_read_buffer = dualCancelReadBuffer;
    stream.set_enqueue_rt_handler = dualSetRtHandler;
    stream.suspend_read = dualSuspendRead;
    stream.enqueue_rt_command = dualEnqueueRtCommand;
    if(s_usb)
        stream.on_linestate_changed = s_usb->on_linestate_changed;

    return &stream;
}
