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
#include <unordered_map>

/*
* Identifiers of USB adapter.
*/
#define FT232H_VID              0x0403
#define FT232H_PID              0x6014

// FTDI / MPSSE Commands
#define MPSSE_DEBUG 0
#define FTDI_DEVICE_OUT_REQTYPE (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_OUT)
#define SIO_RESET_REQUEST               0
#define SIO_SET_LATENCY_TIMER_REQUEST   0x09
#define SIO_SET_BITMODE_REQUEST         0x0B

#define SIO_RESET_SIO                   0
#define BITMODE_RESET                   0
#define BITMODE_MPSSE                   0x02     
#define LATENCY_MS                      2            

#define FREQ_TO_DIV(freq) (((60000000 / freq) / 2) - 1)

class FT232HDevice : public USBDevice
{
public:
    FT232HDevice(libusb_device *device, bool verbose);
    virtual ~FT232HDevice() noexcept;

    static bool probe(libusb_device *device);

    virtual int open();
    virtual bool matchConfiguration(const Value &config);
    virtual void loadConfiguration(const Value &config);
    virtual void writeMessage(const OPC::Message &msg);
    virtual void writeMessage(Document &msg);
    virtual void writeColorCorrection(const Value &color);
    virtual std::string getName();
    virtual void flush();
    virtual void describe(rapidjson::Value &object, Allocator &alloc);

private:
    static const unsigned char OUT_ENDPOINT = 0x02;

    static const uint32_t START_FRAME = 0x00000000;
    static const uint32_t END_FRAME = 0xFFFFFFFF;
    static const uint8_t BRIGHTNESS_MASK = 0xE0;

    union PixelFrame {
        struct
        {
            uint8_t l;
            uint8_t b;
            uint8_t g;
            uint8_t r;
        };

        uint32_t value;
    };

    typedef std::unordered_map<uint8_t, std::unordered_map<uint8_t, uint8_t>> ColorLUT;

    const Value *mConfigMap;

    char mSerialBuffer[256];

    libusb_device_descriptor mDD;
    PixelFrame* mFrameBuffer;
    PixelFrame* mFlushBuffer;
    ColorLUT mColorLUT;
    uint32_t mNumLights;
    uint8_t mBrightness;

    // buffer accessor
    PixelFrame *fbPixel(unsigned num) {
        return &mFrameBuffer[num + 1];
    }

    int mpsseWrite(unsigned char* buf, int size);
    void writeDevicePixels(Document &msg);
    void writeFramebuffer();

    void opcSetPixelColors(const OPC::Message &msg);
    void opcSysEx(const OPC::Message &msg);
    void opcSetGlobalColorCorrection(const OPC::Message &msg);
    void opcMapPixelColors(const OPC::Message &msg, const Value &inst);
};
