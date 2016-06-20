/* mbed Microcontroller Library - Ticker
 * Copyright (c) 2007-2009 ARM Limited. All rights reserved.
 */ 
 
#ifndef MBED_TICKER_H
#define MBED_TICKER_H

#include "TimerEvent.h"
#include "FunctionPointer.h"

namespace mbed {

/** A Ticker is used to call a function at a recurring interval
 *
 *  You can use as many seperate Ticker objects as you require. 
 *
 * Example:
 * @code
 * // Toggle the blinking led after 5 seconds
 *
 * #include "mbed.h"
 * 
 * Ticker timer;
 * DigitalOut led1(LED1);
 * DigitalOut led2(LED2);
 * 
 * int flip = 0;
 * 
 * void attime() {
 *     flip = !flip;
 * }
 *
 * int main() {
 *     timer.attach(&attime, 5);
 *     while(1) {
 *         if(flip == 0) {
 *             led1 = !led1;
 *         } else {
 *             led2 = !led2;
 *         }
 *         wait(0.2);
 *     }
 * }
 * @endcode
 */
class Ticker : public TimerEvent {

public:

    /** Attach a function to be called by the Ticker, specifiying the interval in seconds
     *     
     *  @param fptr pointer to the function to be called
     *  @param t the time between calls in seconds
     */
    void attach(void (*fptr)(void), float t) {
        attach_us(fptr, t * 1000000.0f);
    }
    
    /** Attach a member function to be called by the Ticker, specifiying the interval in seconds
     *     
     *  @param tptr pointer to the object to call the member function on
     *  @param mptr pointer to the member function to be called
     *  @param t the time between calls in seconds
     */
    template<typename T>
    void attach(T* tptr, void (T::*mptr)(void), float t) {
        attach_us(tptr, mptr, t * 1000000.0f);
    }
    
    /** Attach a function to be called by the Ticker, specifiying the interval in micro-seconds
     *     
     *  @param fptr pointer to the function to be called
     *  @param t the time between calls in micro-seconds
     */
    void attach_us(void (*fptr)(void), unsigned int t) {
        _function.attach(fptr);
        setup(t);
    }

    /** Attach a member function to be called by the Ticker, specifiying the interval in micro-seconds
     *     
     *  @param tptr pointer to the object to call the member function on
     *  @param mptr pointer to the member function to be called
     *  @param t the time between calls in micro-seconds
     */    
    template<typename T>
    void attach_us(T* tptr, void (T::*mptr)(void), unsigned int t) {
        _function.attach(tptr, mptr);
        setup(t);
    }
    
    /** Detach the function
     */        
    void detach();

protected:

    void setup(unsigned int t);
    virtual void handler();

    unsigned int _delay;
    FunctionPointer _function;

};

} // namespace mbed

#endif
