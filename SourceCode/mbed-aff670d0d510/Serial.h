/* mbed Microcontroller Library - Serial
 * Copyright (c) 2007-2011 ARM Limited. All rights reserved.
 */ 
 
#ifndef MBED_SERIAL_H
#define MBED_SERIAL_H

#include "device.h"

#if DEVICE_SERIAL

#include "platform.h"
#include "PinNames.h"
#include "PeripheralNames.h"
#include "Stream.h"
#include "FunctionPointer.h"

namespace mbed {

/** A serial port (UART) for communication with other serial devices
 *
 *  Can be used for Full Duplex communication, or Simplex by specifying 
 *  one pin as NC (Not Connected)
 *
 * Example:
 * @code
 * // Print "Hello World" to the PC
 *
 * #include "mbed.h"
 *
 * Serial pc(USBTX, USBRX);
 *
 * int main() {
 *     pc.printf("Hello World\n");
 * }
 * @endcode
 */
class Serial : public Stream {

public:

    /** Create a Serial port, connected to the specified transmit and receive pins
     *
     *  @param tx Transmit pin 
     *  @param rx Receive pin
     *
     *  @note
     *    Either tx or rx may be specified as NC if unused
     */
    Serial(PinName tx, PinName rx, const char *name = NULL);

    /** Set the baud rate of the serial port
     *  
     *  @param baudrate The baudrate of the serial port (default = 9600).
     */
    void baud(int baudrate);

    enum Parity {
        None = 0
        , Odd
        , Even
        , Forced1    
        , Forced0
    };

    enum IrqType {
        RxIrq = 0
        , TxIrq
    };

    /** Set the transmission format used by the Serial port
     *
     *  @param bits The number of bits in a word (5-8; default = 8)
     *  @param parity The parity used (Serial::None, Serial::Odd, Serial::Even, Serial::Forced1, Serial::Forced0; default = Serial::None)
     *  @param stop The number of stop bits (1 or 2; default = 1)
     */
    void format(int bits = 8, Parity parity = Serial::None, int stop_bits = 1); 

#if 0 // Inhereted from Stream, for documentation only

    /** Write a character
     *
     *  @param c The character to write to the serial port
     */
    int putc(int c);

    /** Reads a character from the serial port. This will block until 
     *  a character is available. To see if a character is available, 
     *  see readable()
     *
     *  @returns
     *    The character read from the serial port
     */
    int getc();

    /** Write a formated string
     *
     *  @param format A printf-style format string, followed by the 
     *    variables to use in formating the string.
     */
    int printf(const char* format, ...);

    /** Read a formated string 
     *
     *  @param format A scanf-style format string,
     *    followed by the pointers to variables to store the results. 
     */
    int scanf(const char* format, ...);
 
#endif
 
    /** Determine if there is a character available to read
     *
     *  @returns
     *    1 if there is a character available to read,
     *    0 otherwise
     */
    int readable();

    /** Determine if there is space available to write a character
     * 
     *  @returns
     *    1 if there is space to write a character,
     *    0 otherwise
     */
    int writeable();

    /** Attach a function to call whenever a serial interrupt is generated
     *
     *  @param fptr A pointer to a void function, or 0 to set as none
     *  @param type Which serial interrupt to attach the member function to (Seriall::RxIrq for receive, TxIrq for transmit buffer empty)
     */
    void attach(void (*fptr)(void), IrqType type = RxIrq);

    /** Attach a member function to call whenever a serial interrupt is generated
     *     
     *  @param tptr pointer to the object to call the member function on
     *  @param mptr pointer to the member function to be called
     *  @param type Which serial interrupt to attach the member function to (Seriall::RxIrq for receive, TxIrq for transmit buffer empty)
     */
    template<typename T>
    void attach(T* tptr, void (T::*mptr)(void), IrqType type = RxIrq) {
        if((mptr != NULL) && (tptr != NULL)) {
            _irq[type].attach(tptr, mptr);
            setup_interrupt(type);
        }
    }

#ifdef MBED_RPC
    virtual const struct rpc_method *get_rpc_methods();
    static struct rpc_class *get_rpc_class();
#endif

protected:

    void setup_interrupt(IrqType type);
    void remove_interrupt(IrqType type);

    virtual int _getc();
    virtual int _putc(int c);

    UARTName _uart;
    FunctionPointer _irq[2];
    int _uidx;

};

} // namespace mbed

#endif

#endif
