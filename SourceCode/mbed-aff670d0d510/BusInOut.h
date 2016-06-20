/* mbed Microcontroller Library - BusInOut
 * Copyright (c) 2009 ARM Limited. All rights reserved.
 */
 
#ifndef MBED_BUSINOUT_H
#define MBED_BUSINOUT_H

#include "platform.h" 
#include "PinNames.h"
#include "PeripheralNames.h"
#include "Base.h"
#include "DigitalInOut.h"

namespace mbed {

/** A digital input output bus, used for setting the state of a collection of pins
 */
class BusInOut : public Base {

public:

    /** Create an BusInOut, connected to the specified pins
     *
     *  @param p<n> DigitalInOut pin to connect to bus bit p<n> (p5-p30, NC)
     *
     *  @note
     *    It is only required to specify as many pin variables as is required
     *    for the bus; the rest will default to NC (not connected)
     */ 
    BusInOut(PinName p0, PinName p1 = NC, PinName p2 = NC, PinName p3 = NC,
             PinName p4 = NC, PinName p5 = NC, PinName p6 = NC, PinName p7 = NC,
             PinName p8 = NC, PinName p9 = NC, PinName p10 = NC, PinName p11 = NC,
             PinName p12 = NC, PinName p13 = NC, PinName p14 = NC, PinName p15 = NC, 
             const char *name = NULL);

    BusInOut(PinName pins[16], const char *name = NULL);

    virtual ~BusInOut();

    /* Group: Access Methods */
        
    /** Write the value to the output bus
     *
     *  @param value An integer specifying a bit to write for every corresponding DigitalInOut pin
     */
    void write(int value);

        
    /** Read the value currently output on the bus
     *
     *  @returns
     *    An integer with each bit corresponding to associated DigitalInOut pin setting
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
     *  @param mode PullUp, PullDown, PullNone
     */
    void mode(PinMode pull);
 
#ifdef MBED_OPERATORS
     /** A shorthand for write()
     */
    BusInOut& operator= (int v);
    BusInOut& operator= (BusInOut& rhs);

    /** A shorthand for read()
     */
    operator int();
#endif

#ifdef MBED_RPC
    virtual const struct rpc_method *get_rpc_methods();
    static struct rpc_class *get_rpc_class();
#endif

protected:

    DigitalInOut* _pin[16];

#ifdef MBED_RPC
    static void construct(const char *arguments, char *res);
#endif
            
};

} // namespace mbed

#endif

