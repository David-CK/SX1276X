/* mbed Microcontroller Library - DigitalOut
 * Copyright (c) 2006-2011 ARM Limited. All rights reserved.
 */ 
 
#ifndef MBED_DIGITALOUT_H
#define MBED_DIGITALOUT_H

#include "platform.h"
#include "PinNames.h"
#include "PeripheralNames.h"
#include "Base.h"

namespace mbed {

/** A digital output, used for setting the state of a pin
 *
 * Example:
 * @code
 * // Toggle a LED
 * #include "mbed.h"
 * 
 * DigitalOut led(LED1);
 * 
 * int main() {
 *     while(1) {
 *         led = !led;
 *         wait(0.2);
 *     }
 * }
 * @endcode
 */
class DigitalOut : public Base {

public:

    /** Create a DigitalOut connected to the specified pin
     *
     *  @param pin DigitalOut pin to connect to
     */
    DigitalOut(PinName pin, const char* name = NULL);

    /** Set the output, specified as 0 or 1 (int)
     *
     *  @param value An integer specifying the pin output value, 
     *      0 for logical 0, 1 (or any other non-zero value) for logical 1 
     */
    void write(int value) {

#if defined(TARGET_LPC1768) || defined(TARGET_LPC2368)

        if(value) {
            _gpio->FIOSET = _mask;
        } else {
            _gpio->FIOCLR = _mask;
        }

#elif defined(TARGET_LPC11U24)

        if(value) {
            LPC_GPIO->SET[_index] = _mask;
        } else {
            LPC_GPIO->CLR[_index] = _mask;
        }
#endif

    }

    /** Return the output setting, represented as 0 or 1 (int)
     *
     *  @returns
     *    an integer representing the output setting of the pin, 
     *    0 for logical 0, 1 for logical 1
     */
    int read() {
#if defined(TARGET_LPC1768) || defined(TARGET_LPC2368)
        return ((_gpio->FIOPIN & _mask) ? 1 : 0);
#elif defined(TARGET_LPC11U24)
        return ((LPC_GPIO->PIN[_index] & _mask) ? 1 : 0);
#endif

    }


#ifdef MBED_OPERATORS
    /** A shorthand for write()
     */
    DigitalOut& operator= (int value) {
        write(value);
        return *this;
    }

    DigitalOut& operator= (DigitalOut& rhs) {
        write(rhs.read());
        return *this;
    }

    
    /** A shorthand for read()
     */
    operator int() {
        return read();
    }

#endif

#ifdef MBED_RPC
    virtual const struct rpc_method *get_rpc_methods();
    static struct rpc_class *get_rpc_class();
#endif

protected:

    PinName             _pin;

#if defined(TARGET_LPC1768) || defined(TARGET_LPC2368)
    LPC_GPIO_TypeDef    *_gpio;
#elif defined(TARGET_LPC11U24)
    int _index;
#endif

    uint32_t            _mask;


};

} // namespace mbed

#endif 
