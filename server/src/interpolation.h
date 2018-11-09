#pragma once
#include <math.h>
#include <algorithm>
#include <chrono>
#include <stdlib.h>

#define LEDS_TOTAL              600
#define CHANNELS_TOTAL          (LEDS_TOTAL * 3)

#define LUT_CH_SIZE             257
#define LUT_TOTAL_SIZE          (LUT_CH_SIZE * 3)

#define FCP_INTERPOLATION       1
#define FCP_DITHERING           1

/*
 * Residuals for temporal dithering. Usually 8 bits is enough, but
 * there are edge cases when it isn't, and we don't have the spare CPU cycles
 * to saturate values before storing. So, 16-bit it is.
 */
typedef int16_t residual_t;

/*
 * A buffer of references to USB packets.
 */

static inline uint32_t millis() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(high_resolution_clock::now().time_since_epoch()).count();
}

struct fcFramebuffer
{
    uint8_t *data;
    uint32_t timestamp;

    fcFramebuffer()
    {
        // Number of lights plus start and end frames
        uint32_t bufferSize = sizeof(uint32_t) * (LEDS_TOTAL + 2); 
        data = (uint8_t*) malloc(bufferSize);
        memset(data, 0, bufferSize);
    }

    // void store(uint8_t *newData)
    // {
    //     uint8_t *prev = data;
    //     data = newData;
    //     // free(prev);
    // }

    uint8_t *pixel(unsigned index)
    {
        return &data[4 + index * 4];
    }
};


/*
 * Data type for current color LUT
 */

union fcLinearLUT
{
    uint16_t entries[LUT_TOTAL_SIZE];
    struct {
        uint16_t r[LUT_CH_SIZE];
        uint16_t g[LUT_CH_SIZE];
        uint16_t b[LUT_CH_SIZE];
    };
};

class Interpolation
{
public:
    fcFramebuffer *fbPrev;      // Frame we're interpolating from
    fcFramebuffer *fbNext;      // Frame we're interpolating to
    fcFramebuffer *fbNew;       // Partial frame, getting ready to become fbNext

    fcFramebuffer fb[3];       // Triple-buffered video frames

    fcLinearLUT *mColorLut;    // Active LUT, linearized for efficiency
    unsigned int mNumLeds;
    

    Interpolation(unsigned numLeds):mNumLeds(numLeds)
    {
        // fcFramebuffer fb[3] = {{mNumLeds}, {mNumLeds}, {mNumLeds}}; 
        fbPrev = &fb[0];
        fbNext = &fb[1];
        fbNew = &fb[2];

        // uint32_t bufferSize = sizeof(residual_t) * (mNumLeds * 3); 
        // residual = (residual_t*) malloc(bufferSize);
    }

    // Main loop context
    void update(uint32_t *out);
    void updateColorLut(fcLinearLUT *colorLut);
    void updateKeyframe(uint8_t *frameBuffer);

private:
    residual_t residual[CHANNELS_TOTAL];
    uint32_t calculateInterpCoefficient();
    uint32_t updatePixel(uint32_t icPrev, uint32_t icNext,
                     const uint8_t *pixelPrev, const uint8_t *pixelNext, residual_t *pResidual);
    uint32_t lutInterpolate(const uint16_t *lut, uint32_t arg);
};
