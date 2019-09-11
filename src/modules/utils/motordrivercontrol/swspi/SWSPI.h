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
 
#ifndef SWSPI_H
#define SWSPI_H

#include "platform.h"

#if DEVICE_SPI

#include "mbed.h" // for SPI


#include "gpio.h"

// A software implemented SPI that can use any digital pins
class SWSPI: public mbed::SPI {
private:
    GPIO mosi;
    GPIO miso;
    GPIO sclk;
    //int port;
    int polarity; // idle clock value
    int phase; // 0=sample on leading (first) clock edge, 1=trailing (second)
    
public:
    /** Create SWSPI object
     *
     *  @param mosi_pin
     *  @param miso_pin
     *  @param sclk_pin
     */
    SWSPI(PinName mosi_pin, PinName miso_pin, PinName sclk_pin);
    
    /** Destructor */
    ~SWSPI();
    
    /** Specify SPI format
     *
     *  @param bits  8 or 16 are typical values
     *  @param mode  0, 1, 2, or 3 phase (bit1) and idle clock (bit0)
     */
    void format(int bits, int mode = 0);
    
    /** Specify SPI clock frequency
     *
     *  @param hz  frequency (optional, defaults to 10000000)
     */
    void frequency(int hz = 10000000);
    
    /** Write data and read result
     *
     *  @param value  data to write (see format for bit size)
     *  returns value read from device
     */
    int write(int value);
};


#endif

#endif // SWSPI_H