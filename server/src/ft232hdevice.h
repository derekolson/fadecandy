/*
 * FT232H device interface
 * 
 * Copyright (c) 2018 Derek Olson
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once
#include "usbdevice.h"
#include "opc.h"
#include <set>

 /*
* Identifiers of USB adapter.
*/
#define FT232H_VID              0x0403
#define FT232H_PID              0x6014

/*
* USB endpoints.
*/
#define IN_EP                   0x02
#define OUT_EP                  0x81

/* Shifting commands IN MPSSE Mode*/
 #define MPSSE_WRITE_NEG 0x01   /* Write TDI/DO on negative TCK/SK edge*/
 #define MPSSE_BITMODE   0x02   /* Write bits, not bytes */
 #define MPSSE_READ_NEG  0x04   /* Sample TDO/DI on negative TCK/SK edge */
 #define MPSSE_LSB       0x08   /* LSB first */
 #define MPSSE_DO_WRITE  0x10   /* Write TDI/DO */
 #define MPSSE_DO_READ   0x20   /* Read TDO/DI */
 #define MPSSE_WRITE_TMS 0x40   /* Write TMS/CS */
 
 /* FTDI MPSSE commands */
 #define SET_BITS_LOW   0x80
 #define SET_BITS_HIGH  0x82
 #define GET_BITS_LOW   0x81
 #define GET_BITS_HIGH  0x83
 #define LOOPBACK_START 0x84
 #define LOOPBACK_END   0x85
 #define TCK_DIVISOR    0x86
 /* H Type specific commands */
 #define DIS_DIV_5       0x8a
 #define EN_DIV_5        0x8b
 #define EN_3_PHASE      0x8c
 #define DIS_3_PHASE     0x8d
 #define CLK_BITS        0x8e
 #define CLK_BYTES       0x8f
 #define CLK_WAIT_HIGH   0x94
 #define CLK_WAIT_LOW    0x95
 #define EN_ADAPTIVE     0x96
 #define DIS_ADAPTIVE    0x97
 #define CLK_BYTES_OR_HIGH 0x9c
 #define CLK_BYTES_OR_LOW  0x9d
 /*FT232H specific commands */
 #define DRIVE_OPEN_COLLECTOR 0x9e
 /* Value Low */
 /* Value HIGH */ /*rate is 12000000/((1+value)*2) */
 #define DIV_VALUE(rate) (rate > 6000000)?0:((6000000/rate -1) > 0xffff)? 0xffff: (6000000/rate -1)
 
 /* Commands in MPSSE and Host Emulation Mode */
 #define SEND_IMMEDIATE 0x87
 #define WAIT_ON_HIGH   0x88
 #define WAIT_ON_LOW    0x89
 
 /* Commands in Host Emulation Mode */
 #define READ_SHORT     0x90
 #define READ_EXTENDED  0x91
 #define WRITE_SHORT    0x92
 #define WRITE_EXTENDED 0x93
 
 /* Definitions for flow control */
 #define SIO_RESET          0 /* Reset the port */
 #define SIO_MODEM_CTRL     1 /* Set the modem control register */
 #define SIO_SET_FLOW_CTRL  2 /* Set flow control register */
 #define SIO_SET_BAUD_RATE  3 /* Set baud rate */
 #define SIO_SET_DATA       4 /* Set the data characteristics of the port */
 
 #define FTDI_DEVICE_OUT_REQTYPE (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_OUT)
 #define FTDI_DEVICE_IN_REQTYPE (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_IN)
 
 /* Requests */
 #define SIO_RESET_REQUEST             SIO_RESET
 #define SIO_SET_BAUDRATE_REQUEST      SIO_SET_BAUD_RATE
 #define SIO_SET_DATA_REQUEST          SIO_SET_DATA
 #define SIO_SET_FLOW_CTRL_REQUEST     SIO_SET_FLOW_CTRL
 #define SIO_SET_MODEM_CTRL_REQUEST    SIO_MODEM_CTRL
 #define SIO_POLL_MODEM_STATUS_REQUEST 0x05
 #define SIO_SET_EVENT_CHAR_REQUEST    0x06
 #define SIO_SET_ERROR_CHAR_REQUEST    0x07
 #define SIO_SET_LATENCY_TIMER_REQUEST 0x09
 #define SIO_GET_LATENCY_TIMER_REQUEST 0x0A
 #define SIO_SET_BITMODE_REQUEST       0x0B
 #define SIO_READ_PINS_REQUEST         0x0C
 #define SIO_READ_EEPROM_REQUEST       0x90
 #define SIO_WRITE_EEPROM_REQUEST      0x91
 #define SIO_ERASE_EEPROM_REQUEST      0x92
 
 
 #define SIO_RESET_SIO 0
 #define SIO_RESET_PURGE_RX 1
 #define SIO_RESET_PURGE_TX 2
 
 #define SIO_DISABLE_FLOW_CTRL 0x0
 #define SIO_RTS_CTS_HS (0x1 << 8)
 #define SIO_DTR_DSR_HS (0x2 << 8)
 #define SIO_XON_XOFF_HS (0x4 << 8)
 
 #define SIO_SET_DTR_MASK 0x1
 #define SIO_SET_DTR_HIGH ( 1 | ( SIO_SET_DTR_MASK  << 8))
 #define SIO_SET_DTR_LOW  ( 0 | ( SIO_SET_DTR_MASK  << 8))
 #define SIO_SET_RTS_MASK 0x2
 #define SIO_SET_RTS_HIGH ( 2 | ( SIO_SET_RTS_MASK << 8 ))
 #define SIO_SET_RTS_LOW ( 0 | ( SIO_SET_RTS_MASK << 8 ))
 
 #define SIO_RTS_CTS_HS (0x1 << 8)


class FT232HDevice : public USBDevice
{
public:
    FT232HDevice(libusb_device *device, bool verbose);
    virtual ~FT232HDevice();

    static bool probe(libusb_device *device);

    virtual int open();
    virtual void loadConfiguration(const Value &config);
    virtual void writeMessage(const OPC::Message &msg);
    virtual void writeMessage(Document &msg);
    virtual void writeColorCorrection(const Value &color);
    virtual std::string getName();
    virtual void flush();
    virtual void describe(rapidjson::Value &object, Allocator &alloc);

    static const unsigned NUM_PIXELS = 600;

    // Send current buffer contents
    void writeFramebuffer();

    // Framebuffer accessor
    uint8_t *fbPixel(unsigned num) {
        return &mFramebuffer.data[4 * (num % PIXELS_PER_PACKET) + 4];
    }

private:
    static const unsigned PIXELS_PER_PACKET = 600;
    static const unsigned LUT_ENTRIES_PER_PACKET = 31;
    static const unsigned LUT_PACKETS = 25;
    static const unsigned LUT_ENTRIES = 257;
    static const unsigned INTERFACE = 1;
    static const unsigned MAX_FRAMES_PENDING = 2;

    static const uint8_t TYPE_FRAMEBUFFER = 0x00;
    static const uint8_t TYPE_LUT = 0x40;
    static const uint8_t TYPE_CONFIG = 0x80;
    static const uint8_t FINAL = 0x20;

    static const uint8_t CFLAG_NO_DITHERING     = (1 << 0);
    static const uint8_t CFLAG_NO_INTERPOLATION = (1 << 1);
    static const uint8_t CFLAG_NO_ACTIVITY_LED  = (1 << 2);
    static const uint8_t CFLAG_LED_CONTROL      = (1 << 3);

    struct Packet {
        uint8_t data[(4*PIXELS_PER_PACKET)+4+300];
    };

    enum PacketType {
        OTHER = 0,
        FRAME,
    };

    struct Transfer {
        Transfer(FT232HDevice *device, void *buffer, int length, PacketType type = OTHER);
        ~Transfer();
        libusb_transfer *transfer;
        #if NEED_COPY_USB_TRANSFER_BUFFER
          void *bufferCopy;
        #endif
        PacketType type;
        bool finished;
    };

    const Value *mConfigMap;
    std::set<Transfer*> mPending;
    int mNumFramesPending;
    bool mFrameWaitingForSubmit;

    char mSerialBuffer[256];
    char mVersionString[10];

    libusb_device_descriptor mDD;
    Packet mFramebuffer;
    Packet mColorLUT[LUT_PACKETS];

    bool submitTransfer(Transfer *fct);
    void writeDevicePixels(Document &msg);
    static LIBUSB_CALL void completeTransfer(libusb_transfer *transfer);

    void opcSetPixelColors(const OPC::Message &msg);
    void opcSysEx(const OPC::Message &msg);
    void opcSetGlobalColorCorrection(const OPC::Message &msg);
    void opcMapPixelColors(const OPC::Message &msg, const Value &inst);
};
