/* SWSPI, Software SPI library
 * Copyright (c) 2012-2014, David R. Van Wagner, http://techwithdave.blogspot.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
 
#include "SWSPI.h"
#include "libs/Kernel.h"
#include "libs/StreamOutput.h"
#include "libs/StreamOutputPool.h"
 
SWSPI::SWSPI(PinName mosi_pin, PinName miso_pin, PinName sclk_pin) : 
    mosi(mosi_pin), miso(miso_pin),sclk(sclk_pin), mbed::SPI()
{
    mosi.output();
    miso.input();
    sclk.output();
    
    /*_bits = 8;
    _mode = 0;
    _hz = 1000000;
    spi_format(&_spi, _bits, _mode, 0);
    spi_frequency(&_spi, _hz);
    
    format(8);
    frequency();*/
}
 
SWSPI::~SWSPI()
{
}
 
void SWSPI::format(int bits, int mode)
{
    THEKERNEL->streams->printf("MotorDriverControl INFO: format bits = 0x%x, mode = 0x%x\r\n",bits,mode);
    _bits = bits;
    _mode = mode;
    polarity = (mode >> 1) & 1;
    phase = mode & 1;
    sclk = (polarity);
}
 
void SWSPI::frequency(int hz)
{
    THEKERNEL->streams->printf("MotorDriverControl INFO: frequency %d\r\n", hz);
    _hz = hz;
}
 
int SWSPI::write(int value)
{
    //THEKERNEL->streams->printf("MotorDriverControl INFO: write(%x)\r\n", value);
    int read = 0;
    for (int bit = _bits-1; bit >= 0; --bit)
    {
        mosi = (((value >> bit) & 0x01) != 0);
 
        if (phase == 0)
        {
            if (miso.get())
                read |= (1 << bit);
        }
 
        sclk = (!polarity);
 
        wait(1.0/_hz/2);
 
        if (phase == 1)
        {
            if (miso.get())
                read |= (1 << bit);
        }
 
        sclk = (polarity);
 
        wait(1.0/_hz/2);
    }
    
    //THEKERNEL->streams->printf("MotorDriverControl INFO: result = %x\r\n", read);
    return read;
}