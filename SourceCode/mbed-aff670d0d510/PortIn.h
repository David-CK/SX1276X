/* mbed Microcontroller Library - PortInOut
 * Copyright (c) 2006-2011 ARM Limited. All rights reserved.
 */ 
 
#ifndef MBED_PORTIN_H
#define MBED_PORTIN_H

#include "device.h"

#if DEVICE_PORTIN

#include "PortNames.h"
#include "PinNames.h"

namespace mbed {

/** A multiple pin digital input
 *
 * Example:
 * @code
 * // Switch on an LED if any of mbed pins 21-26 is high
 *
 * #include "mbed.h"
 *
 * PortIn     p(Port2, 0x0000003F);   // p21-p26
 * DigitalOut ind(LED4);
 *
 * int main() {
 *     while(1) {
 *         int pins = p.read();
 *         if(pins) {
 *             ind = 1;
 *         } else {
 *             ind = 0;
 *         }
 *     }
 * }
 * @endcode
 */
class PortIn {
public:

    /** Create an PortIn, connected to the specified port
     *
     *  @param port Port to connect to (Port0-Port5)
     *  @param mask A bitmask to identify which bits in the port should be included (0 - ignore)
     */ 
    PortIn(PortName port, int mask = 0xFFFFFFFF);

    /** Read the value currently output on the port
     *
     *  @returns
     *    An integer with each bit corresponding to associated port pin setting
     */
    int read();

    /** Set the input pin mode
     *
     *  @param mode PullUp, PullDown, PullNone, OpenDrain
     */
    void mode(PinMode mode);
    
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
