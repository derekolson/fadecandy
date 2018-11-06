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

#include "ft232hdevice.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "opc.h"
#include <math.h>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <stdio.h>


FT232HDevice::FT232HDevice(libusb_device *device, bool verbose)
    : USBDevice(device, "ft232h", verbose),
      mConfigMap(0),
      mNumLights(0)
{
    mSerialBuffer[0] = '\0';
    mSerialString = mSerialBuffer;
}

FT232HDevice::~FT232HDevice()
{
    free(mFrameBuffer);
    free(mFlushBuffer);
}

bool FT232HDevice::probe(libusb_device *device)
{
    libusb_device_descriptor dd;

    if (libusb_get_device_descriptor(device, &dd) < 0) {
        // Can't access descriptor?
        return false;
    }

    return dd.idVendor == FT232H_VID && dd.idProduct == FT232H_PID;
}

int FT232HDevice::open()
{
    int r = libusb_get_device_descriptor(mDevice, &mDD);
    if (r < 0) {
        return r;
    }

    mMpsse = mpsse_open_dev(mDevice, TEN_MHZ);
    mHandle = mMpsse->ftdi.usb_dev;

    return libusb_get_string_descriptor_ascii(mHandle, mDD.iSerialNumber, 
        (uint8_t*)mSerialBuffer, sizeof mSerialBuffer);
}

bool FT232HDevice::matchConfiguration(const Value &config) {
    const Value &vNumLights = config["numLights"];

    if (vNumLights.IsNull() || !vNumLights.IsUint()) {
        std::clog << "USB device " << getName() << " configuration missing: \"numLights\"\n";
        return false;
    }

    return USBDevice::matchConfiguration(config);
}

void FT232HDevice::loadConfiguration(const Value &config)
{
    mConfigMap = findConfigMap(config);

    const Value &vNumLights = config["numLights"];
    mNumLights = vNumLights.GetUint();

    uint32_t bufferSize = sizeof(PixelFrame) * (mNumLights + 2); // Number of lights plus start and end frames
    mFrameBuffer = (PixelFrame*)malloc(bufferSize);

    uint32_t flushSize = (mNumLights / 2) + (mNumLights % 2);
    mFlushBuffer = (PixelFrame*)malloc(flushSize);

    // Initialize all buffers to zero
    memset(mFlushBuffer, 0, flushSize);
    memset(mFrameBuffer, 0, bufferSize);

    // Initialize start and end frames
    mFrameBuffer[0].value = START_FRAME;
    mFrameBuffer[mNumLights + 1].value = END_FRAME;
}

void FT232HDevice::flush()
{
}

void FT232HDevice::writeColorCorrection(const Value &color)
{
    // Default color LUT parameters
    double gamma = 1.0;                         // Power for nonlinear portion of curve
    double whitepoint[3] = {1.0, 1.0, 1.0};     // White-point RGB value (also, global brightness)

    /*
     * Parse the JSON object
     */

    if (color.IsObject()) {
        const Value &vGamma = color["gamma"];
        const Value &vWhitepoint = color["whitepoint"];

        if (vGamma.IsNumber()) {
            gamma = vGamma.GetDouble();
        } else if (!vGamma.IsNull() && mVerbose) {
            std::clog << "Gamma value must be a number.\n";
        }

        if (vWhitepoint.IsArray() &&
            vWhitepoint.Size() == 3 &&
            vWhitepoint[0u].IsNumber() &&
            vWhitepoint[1].IsNumber() &&
            vWhitepoint[2].IsNumber()) {
            whitepoint[0] = vWhitepoint[0u].GetDouble();
            whitepoint[1] = vWhitepoint[1].GetDouble();
            whitepoint[2] = vWhitepoint[2].GetDouble();
        } else if (!vWhitepoint.IsNull() && mVerbose) {
            std::clog << "Whitepoint value must be a list of 3 numbers.\n";
        }

    } else if (!color.IsNull() && mVerbose) {
        std::clog << "Color correction value must be a JSON dictionary object.\n";
    }

    /*
     * Calculate the color LUT
     */

    for (unsigned channel = 0; channel < 3; channel++) {
        for (unsigned entry = 0; entry < 256; entry++) {
            double output;

            /*
             * Normalized input value corresponding to this LUT entry.
             * Ranges from 0 to slightly higher than 1. (The last LUT entry
             * can't quite be reached.)
             */
            double input = (entry << 8) / 65535.0;

            // Scale by whitepoint before anything else
            input *= whitepoint[channel];

            output = pow(input, gamma);

            // Round to the nearest integer, and clamp. Overflow-safe.
            uint8_t intValue = (output * 0xFF) + 0.5;
            mColorLUT[channel][entry] = std::max<uint8_t>(0, std::min<uint8_t>(0xFF, intValue));
        }
    }
}

void FT232HDevice::writeFramebuffer()
{
    mpsse_write(mMpsse, (char *) mFrameBuffer, sizeof(PixelFrame) * (mNumLights + 2));
}

void FT232HDevice::writeMessage(Document &msg)
{
    /*
     * Dispatch a device-specific JSON command.
     *
     * This can be used to send frames or settings directly to one device,
     * bypassing the mapping we use for Open Pixel Control clients. This isn't
     * intended to be the fast path for regular applications, but it can be used
     * by configuration tools that need to operate regardless of the mapping setup.
     */

    const char *type = msg["type"].GetString();

    if (!strcmp(type, "device_pixels")) {
        // Write raw pixels, without any mapping
        writeDevicePixels(msg);
        return;
    }

    // Chain to default handler
    USBDevice::writeMessage(msg);
}

void FT232HDevice::writeDevicePixels(Document &msg)
{
    /*
     * Write pixels without mapping, from a JSON integer
     * array in msg["pixels"]. The pixel array is removed from
     * the reply to save network bandwidth.
     *
     * Pixel values are clamped to [0, 255], for convenience.
     */

    const Value &pixels = msg["pixels"];
    if (!pixels.IsArray()) {
        msg.AddMember("error", "Pixel array is missing", msg.GetAllocator());
    } else {

        // Truncate to the framebuffer size, and only deal in whole pixels.
        uint32_t numPixels = pixels.Size() / 3;
        if (numPixels > mNumLights)
            numPixels = mNumLights;

        for (uint32_t i = 0; i < numPixels; i++) {
            PixelFrame *out = fbPixel(i);

            const Value &r = pixels[i * 3 + 0];
            const Value &g = pixels[i * 3 + 1];
            const Value &b = pixels[i * 3 + 2];

            out->r = std::max(0, std::min(255, r.IsInt() ? r.GetInt() : 0));
            out->g = std::max(0, std::min(255, g.IsInt() ? g.GetInt() : 0));
            out->b = std::max(0, std::min(255, b.IsInt() ? b.GetInt() : 0));
            out->l = 0xFF; // todo: fix so we actually pass brightness
        }

        writeFramebuffer();
    }
}

void FT232HDevice::writeMessage(const OPC::Message &msg)
{
    /*
     * Dispatch an incoming OPC command
     */

    switch (msg.command) {

        case OPC::SetPixelColors:
            opcSetPixelColors(msg);
            writeFramebuffer();
            return;

        case OPC::SystemExclusive:
            opcSysEx(msg);
            return;
    }

    if (mVerbose) {
        std::clog << "Unsupported OPC command: " << unsigned(msg.command) << "\n";
    }
}

void FT232HDevice::opcSysEx(const OPC::Message &msg)
{
    if (msg.length() < 4) {
        if (mVerbose) {
            std::clog << "SysEx message too short!\n";
        }
        return;
    }

    unsigned id = (unsigned(msg.data[0]) << 24) |
                  (unsigned(msg.data[1]) << 16) |
                  (unsigned(msg.data[2]) << 8)  |
                   unsigned(msg.data[3])        ;

    switch (id) {

        case OPC::FCSetGlobalColorCorrection:
            return opcSetGlobalColorCorrection(msg);

    }

    // Quietly ignore unhandled SysEx messages.
}

void FT232HDevice::opcSetPixelColors(const OPC::Message &msg)
{
    /*
     * Parse through our device's mapping, and store any relevant portions of 'msg'
     * in the framebuffer.
     */

    if (!mConfigMap) {
        // No mapping defined yet. This device is inactive.
        return;
    }

    const Value &map = *mConfigMap;
    for (unsigned i = 0, e = map.Size(); i != e; i++) {
        opcMapPixelColors(msg, map[i]);
    }
}

void FT232HDevice::opcMapPixelColors(const OPC::Message &msg, const Value &inst)
{
    /*
     * Parse one JSON mapping instruction, and copy any relevant parts of 'msg'
     * into our framebuffer. This looks for any mapping instructions that we
     * recognize:
     *
     *   [ OPC Channel, First OPC Pixel, First output pixel, Pixel count ]
     */

    unsigned msgPixelCount = msg.length() / 3;

    if (inst.IsArray() && inst.Size() == 4) {
        // Map a range from an OPC channel to our framebuffer

        const Value &vChannel = inst[0u];
        const Value &vFirstOPC = inst[1];
        const Value &vFirstOut = inst[2];
        const Value &vCount = inst[3];

        if (vChannel.IsUint() && vFirstOPC.IsUint() && vFirstOut.IsUint() && vCount.IsInt()) {
            unsigned channel = vChannel.GetUint();
            unsigned firstOPC = vFirstOPC.GetUint();
            unsigned firstOut = vFirstOut.GetUint();
            unsigned count;
            int direction;
            if (vCount.GetInt() >= 0) {
                count = vCount.GetInt();
                direction = 1;
            } else {
                count = -vCount.GetInt();
                direction = -1;
            }

            if (channel != msg.channel) {
                return;
            }

            // Clamping, overflow-safe
            firstOPC = std::min<unsigned>(firstOPC, msgPixelCount);
            firstOut = std::min<unsigned>(firstOut, mNumLights);
            count = std::min<unsigned>(count, msgPixelCount - firstOPC);
            count = std::min<unsigned>(count,
                direction > 0 ? mNumLights - firstOut : firstOut + 1);

            // Copy pixels
            const uint8_t *inPtr = msg.data + (firstOPC * 3);
            unsigned outIndex = firstOut;
            while (count--) {
                PixelFrame *outPtr = fbPixel(outIndex);
                outIndex += direction;
                outPtr->r = mColorLUT[0][inPtr[0]];
                outPtr->g = mColorLUT[1][inPtr[1]];
                outPtr->b = mColorLUT[2][inPtr[2]];
                outPtr->l = 0xFF; // todo: fix so we actually pass brightness
                inPtr += 3;
            }

            return;
        }
    }

    // Still haven't found a match?
    if (mVerbose) {
        rapidjson::GenericStringBuffer<rapidjson::UTF8<> > buffer;
        rapidjson::Writer<rapidjson::GenericStringBuffer<rapidjson::UTF8<> > > writer(buffer);
        inst.Accept(writer);
        std::clog << "Unsupported JSON mapping instruction: " << buffer.GetString() << "\n";
    }
}

void FT232HDevice::opcSetGlobalColorCorrection(const OPC::Message &msg)
{
    /*
     * Parse the message as JSON text, and if successful, write new
     * color correction data to the device.
     */

    // Mutable NUL-terminated copy of the message string
    std::string text((char*)msg.data + 4, msg.length() - 4);

    // Parse it in-place
    rapidjson::Document doc;
    doc.ParseInsitu<0>(&text[0]);

    if (doc.HasParseError()) {
        if (mVerbose) {
            std::clog << "Parse error in color correction JSON at character "
                << doc.GetErrorOffset() << ": " << doc.GetParseError() << "\n";
        }
        return;
    }

    /*
     * Successfully parsed the JSON. From here, it's handled identically to
     * objects that come through the config file.
     */
    writeColorCorrection(doc);
}

std::string FT232HDevice::getName()
{
    std::ostringstream s;
    s << "FT232H";
    if (mSerialString[0]) {
        s << " (Serial# " << mSerialString << ")";
    }
    return s.str();
}

void FT232HDevice::describe(rapidjson::Value &object, Allocator &alloc)
{
    USBDevice::describe(object, alloc);
    object.AddMember("numLights", mNumLights, alloc);
}
