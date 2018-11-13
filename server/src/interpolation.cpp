#include "interpolation.h"

static int clamp(int val, int min, int max) {
    val = min ^ (( min ^ val ) & -( min < val ));
    val = max ^ (( val ^ max ) & -( val < max ));
    // unsigned char val8bit = 0xFF & (( 0xFFFF & val ) >> 8 );
    return val;
}

uint32_t Interpolation::calculateInterpCoefficient()
{
    /*
     * Calculate our interpolation coefficient. This is a value between
     * 0x0000 and 0x10000, representing some point in between fbPrev and fbNext.
     *
     * We timestamp each frame at the moment its final packet has been received.
     * In other words, fbNew has no valid timestamp yet, and fbPrev/fbNext both
     * have timestamps in the recent past.
     *
     * fbNext's timestamp indicates when both fbPrev and fbNext entered their current
     * position in the keyframe queue. The difference between fbPrev and fbNext indicate
     * how long the interpolation between those keyframes should take.
     */

    uint32_t now = millis();
    uint32_t tsPrev = fbPrev->timestamp;
    uint32_t tsNext = fbNext->timestamp;
    uint32_t tsDiff = tsNext - tsPrev;
    uint32_t tsElapsed = now - tsNext;

    if(tsDiff == 0){
        return 0x10000;
    }

    // Careful to avoid overflows if the frames stop coming...
    return (std::min<uint32_t>(tsElapsed, tsDiff) << 16) / tsDiff;
}

void Interpolation::updateKeyframe(uint8_t *frameBuffer) {
    fbNew->data = frameBuffer;
    fbNew->timestamp = millis();

    fcFramebuffer *recycle = fbPrev;
    fbPrev = fbNext;
    fbNext = fbNew;
    fbNew = recycle;
}

void Interpolation::updateColorLut(fcLinearLUT *colorLut) {
    mColorLut = colorLut;
    // for(unsigned i=0; i < LUT_CH_SIZE; i++){
    //     printf("%u-", mColorLut->r[i]);
    // }
    // printf("\n");
}

void Interpolation::update(uint32_t *out) {
    /*
     * Interpolation coefficients, including a multiply by 257 to convert 8-bit color to 16-bit color.
     * You'd think that it would save clock cycles to calculate icPrev in updatePixel(), but this doesn't
     * seem to be the case.
     *
     * icPrev in range [0, 0x1010000]
     * icNext in range [0, 0x1010000]
     * icPrev + icNext = 0x1010000
     */
    uint32_t interpCoefficient = calculateInterpCoefficient();
    uint32_t icPrev = 257 * (0x10000 - interpCoefficient);
    uint32_t icNext = 257 * interpCoefficient;

    /*
     * Pointer to the residual buffer for this pixel. Calculating this here rather than in updatePixel
     * saves a lot of clock cycles, since otherwise updatePixel() immediately needs to do a load from
     * constant pool and some multiplication.
     */

    residual_t *pResidual = residual;

    for (int i = 0; i < mNumLeds; i++, pResidual += 3) {

        uint32_t pixel = updatePixel(icPrev, icNext,
            fbPrev->pixel(i),
            fbNext->pixel(i),
            pResidual);
        
        out[i + 1] = pixel;
    }

}

uint32_t Interpolation::updatePixel(uint32_t icPrev, uint32_t icNext,
    const uint8_t *pixelPrev, const uint8_t *pixelNext, residual_t *pResidual)
{
    // printf("%u, %u, %u\n", pixelNext[0], pixelNext[1], pixelNext[2]);

    /*
     * Update pipeline for one pixel:
     *
     *    1. Interpolate framebuffer
     *    2. Interpolate LUT
     *    3. Dithering
     *
     * icPrev in range [0, 0x1010000]
     * icNext in range [0, 0x1010000]
     * icPrev + icNext = 0x1010000
     */

#if FCP_INTERPOLATION
    // Per-channel linear interpolation and conversion to 16-bit color.
    // Result range: [0, 0xFFFF] 
    int iR = (pixelPrev[3] * icPrev + pixelNext[3] * icNext) >> 16;
    int iG = (pixelPrev[2] * icPrev + pixelNext[2] * icNext) >> 16;
    int iB = (pixelPrev[1] * icPrev + pixelNext[1] * icNext) >> 16;
#else
    int iR = pixelNext[3] * 0x101;
    int iG = pixelNext[2] * 0x101;
    int iB = pixelNext[1] * 0x101;
#endif

    // Pass through our color LUT
    // Result range: [0, 0xFFFF] 
    iR = lutInterpolate(mColorLut->r, iR);
    iG = lutInterpolate(mColorLut->g, iG);
    iB = lutInterpolate(mColorLut->b, iB);

#if FCP_DITHERING
    // Incorporate the residual from last frame
    iR += pResidual[0];
    iG += pResidual[1];
    iB += pResidual[2];
#endif

    /*
     * Round to the nearest 8-bit value. Clamping is necessary!
     * This value might be as low as -128 prior to adding 0x80
     * for rounding. After this addition, the result is guaranteed
     * to be >= 0, but it may be over 0xffff.
     *
     * This rules out clamping using the UQADD16 instruction,
     * since the addition itself needs to allow overflow. Instead,
     * we clamp using a separate USAT instruction.
     */

    int r8 = clamp(iR + 0x80, 0, 0xffff) >> 8;
    int g8 = clamp(iG + 0x80, 0, 0xffff) >> 8;
    int b8 = clamp(iB + 0x80, 0, 0xffff) >> 8;

#if FCP_DITHERING
    // Compute the error, after expanding the 8-bit value back to 16-bit.
    pResidual[0] = iR - (r8 * 257);
    pResidual[1] = iG - (g8 * 257);
    pResidual[2] = iB - (b8 * 257);
#endif

    // Pack the result, in BGR order.
    // return (0xFF << 24) | (0xAD << 16) | (0xBE << 8) | (0xEF << 0);
    return (r8 << 24) | (g8 << 16) | (b8 << 8) | pixelNext[0];
}

uint32_t Interpolation::lutInterpolate(const uint16_t *lut, uint32_t arg)
{
    /*
     * Using our color LUT for the indicated channel, convert the
     * 16-bit intensity "arg" in our input colorspace to a corresponding
     * 16-bit intensity in the device colorspace.
     *
     * Remember that our LUT is 257 entries long. The final entry corresponds to an
     * input of 0x10000, which can't quite be reached.
     *
     * 'arg' is in the range [0, 0xFFFF]
     *
     * This operation is equivalent to the following:
     *
     *      unsigned index = arg >> 8;          // Range [0, 0xFF]
     *      unsigned alpha = arg & 0xFF;        // Range [0, 0xFF]
     *      unsigned invAlpha = 0x100 - alpha;  // Range [1, 0x100]
     *
     *      // Result in range [0, 0xFFFF]
     *      return (lut[index] * invAlpha + lut[index + 1] * alpha) >> 8;
     *
     * This is easy to understand, but it turns out to be a serious bottleneck
     * in terms of speed and memory bandwidth, as well as register pressure that
     * affects the compilation of updatePixel().
     *
     * To speed this up, we try and do the lut[index] and lut[index+1] portions
     * in parallel using the SMUAD instruction. This is a pair of 16x16 multiplies,
     * and the results are added together. We can combine this with an unaligned load
     * to grab two adjacent entries from the LUT. The remaining complications are:
     *
     *   1. We wanted unsigned, not signed
     *   2. We still need to generate the input values efficiently.
     *
     * (1) is easy to solve if we're okay with 15-bit precision for the LUT instead
     * of 16-bit, which is fine. During LUT preparation, we right-shift each entry
     * by 1, keeping them within the positive range of a signed 16-bit int. 
     *
     * For (2), we need to quickly put 'alpha' in the high halfword and invAlpha in
     * the low halfword, or vice versa. One fast way to do this is (0x01000000 + x - (x << 16).
     */

#if FCP_INTERPOLATION

    // uint32_t index = arg >> 8;          // Range [0, 0xFF]

    // // Load lut[index] into low halfword, lut[index+1] into high halfword.
    // uint32_t pair = *(const uint32_t*)(lut + index);

    // unsigned alpha = arg & 0xFF;        // Range [0, 0xFF]

    // // Reversed halfword order
    // uint32_t pairAlpha = (0x01000000 + alpha - (alpha << 16));

    // return __SMUADX(pairAlpha, pair) >> 7;

    unsigned index = arg >> 8;          // Range [0, 0xFF]
    unsigned alpha = arg & 0xFF;        // Range [0, 0xFF]
    unsigned invAlpha = 0x100 - alpha;  // Range [1, 0x100]

    return (lut[index] * invAlpha + lut[index + 1] * alpha) >> 8;

#else
    // Simpler non-interpolated version
    return lut[arg >> 8];
#endif
}
