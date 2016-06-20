/* mbed Microcontroller Library - PortInOut
 * Copyright (c) 2006-2011 ARM Limited. All rights reserved.
 */ 
 
#ifndef MBED_PORTINOUT_H
#define MBED_PORTINOUT_H

#include "device.h"

#if DEVICE_PORTINOUT

#include "PortNames.h"
#include "PinNames.h"

namespace mbed {

/** A multiple pin digital in/out used to set/read multiple bi-directional pins
 */
class PortInOut {
public:

    /** Create an PortInOut, connected to the specified port
     *
     *  @param port Port to connect to (Port0-Port5)
     *  @param mask A bitmask to identify which bits in the port should be included (0 - ignore)
     */ 
    PortInOut(PortName port, int mask = 0xFFFFFFFF);

    /** Write the value to the output port
     *
     *  @param value An integer specifying a bit to write for every corresponding port pin
     */    
    void write(int value);

    /** Read the value currently output on the port
     *
     *  @returns
     *    An integer with each bit corresponding to associated port pin setting
     */
    int read();

    /** Set as an output
     */
    void output();

    /** Set as an input
     */
    void input();

    /** Set the input pin mode
     *
     *  @param mode PullUp, PullDown, PullNone, OpenDrain
     */
    void mode(PinMode mode);

    /** A shorthand for write()
     */    
    PortInOut& operator= (int value) { 
        write(value);
        return *this;
    }
    
    PortInOut& operator= (PortInOut& rhs) { 
        write(rhs.read());
        return *this;
    }
    
    /** A shorthand for read()
     */
    operator int() { 
        return read();
    }

private:
#if defined(TARGET_LPC1768) || defined(TARGET_LPC2368)
    LPC_GPIO_TypeDef    *_gpio;
#endif
    PortName            _port;
    uint32_t            _mask;
};

} // namespace mbed

#endif

#endif
