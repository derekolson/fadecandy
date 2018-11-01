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

static int bulk_write(libusb_device_handle *handle, unsigned char *output, int nbytes)
{
    int bytes_written;

    // if (1) {
    //     int i;
    //     fprintf(stderr, "usb bulk write %d bytes:", nbytes);
    //     for (i=0; i<nbytes; i++)
    //         fprintf(stderr, "%c%02x", i ? '-' : ' ', output[i]);
    //     fprintf(stderr, "\n");
    // }

    int ret = libusb_bulk_transfer(handle, IN_EP, (unsigned char*) output,
        nbytes, &bytes_written, 1000);

    if (bytes_written != nbytes)
        fprintf(stderr, "usb bulk written %d bytes of %d",
            bytes_written, nbytes);
    
    return ret;
}

static void mpsse_speed(libusb_device_handle *handle, int khz)
{
    unsigned char output [3];
    int divisor = (30 * 2000 / khz + 1) / 2 - 1;

    if (divisor < 0)
        divisor = 0;

    if (30 > 6) {
        /* Use 60MHz master clock (disable divide by 5). */
        output[0] = DIS_DIV_5;

        /* Turn off adaptive clocking. */
        output[1] = DIS_ADAPTIVE;

        /* Disable three-phase clocking. */
        output[2] = DIS_3_PHASE;
        bulk_write(handle, output, 3);
    }

    /* Command "set TCK divisor". */
    output [0] = TCK_DIVISOR;
    output [1] = (divisor & 0xFF);
    output [2] = ((divisor >> 8) & 0xFF);
    bulk_write(handle, output, 3);

    khz = (30 * 2000 / (divisor + 1) + 1) / 2;
    fprintf(stderr, "%s: clock rate %.1f MHz\n", "ft232h", khz / 1000.0);
}

FT232HDevice::Transfer::Transfer(FT232HDevice *device, void *buffer, int length, PacketType type)
    : transfer(libusb_alloc_transfer(0)),
      type(type), finished(false)
{
    #if NEED_COPY_USB_TRANSFER_BUFFER
        bufferCopy = malloc(length);
        memcpy(bufferCopy, buffer, length);
        uint8_t *data = (uint8_t*) bufferCopy;
    #else
        uint8_t *data = (uint8_t*) buffer;
    #endif

    libusb_fill_bulk_transfer(transfer, device->mHandle,
        IN_EP, data, length, FT232HDevice::completeTransfer, this, 2000);
}

FT232HDevice::Transfer::~Transfer()
{
    libusb_free_transfer(transfer);
    #if NEED_COPY_USB_TRANSFER_BUFFER
        free(bufferCopy);
    #endif
}

FT232HDevice::FT232HDevice(libusb_device *device, bool verbose)
    : USBDevice(device, "ft232h", verbose),
      mConfigMap(0), mNumFramesPending(0), mFrameWaitingForSubmit(false)
{
    mSerialBuffer[0] = '\0';
    mSerialString = mSerialBuffer;

    // Framebuffer headers
    memset(&mFramebuffer, 0, sizeof mFramebuffer);
    for (unsigned i = 0; i < 2704; i++) {
        mFramebuffer.data[i] = 0x00;
    }
}

FT232HDevice::~FT232HDevice()
{
    /*
     * If we have pending transfers, cancel them.
     * The Transfer objects themselves will be freed
     * once libusb completes them.
     */

    for (std::set<Transfer*>::iterator i = mPending.begin(), e = mPending.end(); i != e; ++i) {
        Transfer *fct = *i;
        libusb_cancel_transfer(fct->transfer);
    }
}

bool FT232HDevice::probe(libusb_device *device)
{
    libusb_device_descriptor dd;

    if (libusb_get_device_descriptor(device, &dd) < 0) {
        // Can't access descriptor?
        return false;
    }

    return dd.idVendor == 0x0403 && dd.idProduct == 0x6014;
}

int FT232HDevice::open()
{
    int r = libusb_get_device_descriptor(mDevice, &mDD);
    if (r < 0) {
        return r;
    }

    r = libusb_open(mDevice, &mHandle);
    if (r < 0) {
        return r;
    }

    r = libusb_claim_interface(mHandle, 0);
    if (r < 0) {
        return r;
    }

    /* Reset the device. */
    r = libusb_control_transfer(mHandle,
        FTDI_DEVICE_OUT_REQTYPE,
        SIO_RESET, 0, 1, 0, 0, 1000);
    if ( r < 0) {
        return r;
    }

    /* MPSSE mode. */
    r = libusb_control_transfer(mHandle,
        FTDI_DEVICE_OUT_REQTYPE,
        SIO_SET_BITMODE_REQUEST, MPSSE_BITMODE, 1, 0, 0, 1000);
    if ( r < 0) {
        return r;
    }

    /* Optimal latency timer is 1 for slow mode and 0 for fast mode. */
    unsigned latency_timer = 0;
    r = libusb_control_transfer(mHandle,
        FTDI_DEVICE_OUT_REQTYPE,
        SIO_SET_LATENCY_TIMER_REQUEST, latency_timer, 1, 0, 0, 1000);
    if ( r != 0) {
        return r;
    }

    r = libusb_control_transfer(mHandle,
        FTDI_DEVICE_IN_REQTYPE,
        SIO_GET_LATENCY_TIMER_REQUEST, 0, 1, (unsigned char*) &latency_timer, 1, 1000);
    if ( r != 1) {
        return r;
    }

    unsigned char enable_loopback[] = "\x85";
    bulk_write(mHandle, enable_loopback, 1);

    mpsse_speed(mHandle, 10000);

    unsigned major = mDD.bcdDevice >> 8;
    unsigned minor = mDD.bcdDevice & 0xFF;
    snprintf(mVersionString, sizeof mVersionString, "%x.%02x", major, minor);

    return libusb_get_string_descriptor_ascii(mHandle, mDD.iSerialNumber, 
        (uint8_t*)mSerialBuffer, sizeof mSerialBuffer);
}

void FT232HDevice::loadConfiguration(const Value &config)
{
    mConfigMap = findConfigMap(config);
}

bool FT232HDevice::submitTransfer(Transfer *fct)
{
    /*
     * Submit a new USB transfer. The Transfer object is guaranteed to be freed eventually.
     * On error, it's freed right away.
     */

    int r = libusb_submit_transfer(fct->transfer);

    if (r < 0) {
        if (mVerbose && r != LIBUSB_ERROR_PIPE) {
            std::clog << "Error submitting USB transfer: " << libusb_strerror(libusb_error(r)) << "\n";
        }
        delete fct;
        return false;

    } else {
        mPending.insert(fct);
        return true;
    }
}

void FT232HDevice::completeTransfer(libusb_transfer *transfer)
{
    FT232HDevice::Transfer *fct = static_cast<FT232HDevice::Transfer*>(transfer->user_data);
    fct->finished = true;
}

void FT232HDevice::flush()
{
    // Erase any finished transfers
    std::set<Transfer*>::iterator current = mPending.begin();
    while (current != mPending.end()) {
        std::set<Transfer*>::iterator next = current;
        next++;

        Transfer *fct = *current;
        if (fct->finished) {
            switch (fct->type) {

                case FRAME:
                    mNumFramesPending--;
                    break;

                default:
                    break;
            }

            mPending.erase(current);
            delete fct;
        }

        current = next;
    }

    // Submit new frames, if we had a queued frame waiting
    if (mFrameWaitingForSubmit && mNumFramesPending < MAX_FRAMES_PENDING) {
        writeFramebuffer();
    }
}

void FT232HDevice::writeColorCorrection(const Value &color)
{
    /*
     * Populate the color correction table based on a JSON configuration object,
     * and send the new color LUT out over USB.
     *
     * 'color' may be 'null' to load an identity-mapped LUT, or it may be
     * a dictionary of options including 'gamma' and 'whitepoint'.
     *
     * This calculates a compound curve with a linear section and a nonlinear
     * section. The linear section, near zero, avoids creating very low output
     * values that will cause distracting flicker when dithered. This isn't a problem
     * when the LEDs are viewed indirectly such that the flicker is below the threshold
     * of perception, but in cases where the flicker is a problem this linear section can
     * eliminate it entierly at the cost of some dynamic range.
     *
     * By default, the linear section is disabled (linearCutoff is zero). To enable the
     * linear section, set linearCutoff to some nonzero value. A good starting point is
     * 1/256.0, correspnding to the lowest 8-bit PWM level.
     */

    // Default color LUT parameters
    double gamma = 1.0;                         // Power for nonlinear portion of curve
    double whitepoint[3] = {1.0, 1.0, 1.0};     // White-point RGB value (also, global brightness)
    double linearSlope = 1.0;                   // Slope (output / input) of linear section of the curve, near zero
    double linearCutoff = 0.0;                  // Y (output) coordinate of intersection of linear and nonlinear curves

    /*
     * Parse the JSON object
     */

    if (color.IsObject()) {
        const Value &vGamma = color["gamma"];
        const Value &vWhitepoint = color["whitepoint"];
        const Value &vLinearSlope = color["linearSlope"];
        const Value &vLinearCutoff = color["linearCutoff"];

        if (vGamma.IsNumber()) {
            gamma = vGamma.GetDouble();
        } else if (!vGamma.IsNull() && mVerbose) {
            std::clog << "Gamma value must be a number.\n";
        }

        if (vLinearSlope.IsNumber()) {
            linearSlope = vLinearSlope.GetDouble();
        } else if (!vLinearSlope.IsNull() && mVerbose) {
            std::clog << "Linear slope value must be a number.\n";
        }

        if (vLinearCutoff.IsNumber()) {
            linearCutoff = vLinearCutoff.GetDouble();
        } else if (!vLinearCutoff.IsNull() && mVerbose) {
            std::clog << "Linear slope value must be a number.\n";
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
     * Calculate the color LUT, stowing the result in an array of USB packets.
     */

    Packet *packet = mColorLUT;
    const unsigned firstByteOffset = 1;  // Skip padding byte
    unsigned byteOffset = firstByteOffset;

    for (unsigned channel = 0; channel < 3; channel++) {
        for (unsigned entry = 0; entry < LUT_ENTRIES; entry++) {
            double output;

            /*
             * Normalized input value corresponding to this LUT entry.
             * Ranges from 0 to slightly higher than 1. (The last LUT entry
             * can't quite be reached.)
             */
            double input = (entry << 8) / 65535.0;

            // Scale by whitepoint before anything else
            input *= whitepoint[channel];

            // Is this entry part of the linear section still?
            if (input * linearSlope <= linearCutoff) {

                // Output value is below linearCutoff. We're still in the linear portion of the curve
                output = input * linearSlope;

            } else {

                // Nonlinear portion of the curve. This starts right where the linear portion leaves
                // off. We need to avoid any discontinuity.

                double nonlinearInput = input - (linearSlope * linearCutoff);
                double scale = 1.0 - linearCutoff;
                output = linearCutoff + pow(nonlinearInput / scale, gamma) * scale;
            }

            // Round to the nearest integer, and clamp. Overflow-safe.
            int64_t longValue = (output * 0xFFFF) + 0.5;
            int intValue = std::max<int64_t>(0, std::min<int64_t>(0xFFFF, longValue));

            // Store LUT entry, little-endian order.
            packet->data[byteOffset++] = uint8_t(intValue);
            packet->data[byteOffset++] = uint8_t(intValue >> 8);
            if (byteOffset >= sizeof packet->data) {
                byteOffset = firstByteOffset;
                packet++;
            }
        }
    }

    // Start asynchronously sending the LUT.
    submitTransfer(new Transfer(this, &mColorLUT, sizeof mColorLUT));
}

void FT232HDevice::writeFramebuffer()
{
    /*
     * Asynchronously write the current framebuffer.
     *
     * TODO: Currently if this gets ahead of what the USB device is capable of,
     *       we always drop frames. Alternatively, it would be nice to have end-to-end
     *       flow control so that the client can produce frames slower.
     */

    if (mNumFramesPending >= MAX_FRAMES_PENDING) {
        // Too many outstanding frames. Wait to submit until a previous frame completes.
        mFrameWaitingForSubmit = true;
        return;
    }

    if (submitTransfer(new Transfer(this, &mFramebuffer, sizeof mFramebuffer, FRAME))) {
        mFrameWaitingForSubmit = false;
        mNumFramesPending++;
    }
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
        int numPixels = pixels.Size() / 3;
        if (numPixels > NUM_PIXELS)
            numPixels = NUM_PIXELS;

        for (int i = 0; i < numPixels; i++) {
            uint8_t *out = fbPixel(i);

            const Value &r = pixels[i*3 + 0];
            const Value &g = pixels[i*3 + 1];
            const Value &b = pixels[i*3 + 2];

            out[0] = std::max(0, std::min(255, r.IsInt() ? r.GetInt() : 0));
            out[1] = std::max(0, std::min(255, g.IsInt() ? g.GetInt() : 0));
            out[2] = std::max(0, std::min(255, b.IsInt() ? b.GetInt() : 0));
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
            firstOut = std::min<unsigned>(firstOut, unsigned(NUM_PIXELS));
            count = std::min<unsigned>(count, msgPixelCount - firstOPC);
            count = std::min<unsigned>(count,
                    direction > 0 ? NUM_PIXELS - firstOut : firstOut + 1);

            // Copy pixels
            const uint8_t *inPtr = msg.data + (firstOPC * 3);
            unsigned outIndex = firstOut;
            while (count--) {
                uint8_t *outPtr = fbPixel(outIndex);
                outIndex += direction;
                outPtr[0] = 0xe0 + 10;
                outPtr[1] = inPtr[1];
                outPtr[2] = inPtr[2];
                outPtr[3] = inPtr[0];
                inPtr += 3;
            }

            // for (int i = 0; i < 100; i++) {
            //     printf("%02X ", mFramebuffer.data[i]);
            // }
            // printf("\n\n");

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
        s << " (Serial# " << mSerialString << ", Version " << mVersionString << ")";
    }
    return s.str();
}

void FT232HDevice::describe(rapidjson::Value &object, Allocator &alloc)
{
    USBDevice::describe(object, alloc);
    object.AddMember("version", mVersionString, alloc);
    object.AddMember("bcd_version", mDD.bcdDevice, alloc);
}
