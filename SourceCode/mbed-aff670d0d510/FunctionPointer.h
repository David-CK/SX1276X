/* mbed Microcontroller Library - FunctionPointer
 * Copyright (c) 2007-2009 ARM Limited. All rights reserved.
 */ 
 
#ifndef MBED_FUNCTIONPOINTER_H
#define MBED_FUNCTIONPOINTER_H

#include <string.h>

namespace mbed { 

/** A class for storing and calling a pointer to a static or member void function
 */
class FunctionPointer {

public:

    /** Create a FunctionPointer, attaching a static function
     * 
     *  @param function The void static function to attach (default is none)
     */
    FunctionPointer(void (*function)(void) = 0);

    /** Create a FunctionPointer, attaching a member function
     * 
     *  @param object The object pointer to invoke the member function on (i.e. the this pointer)
     *  @param function The address of the void member function to attach 
     */
    template<typename T>    
    FunctionPointer(T *object, void (T::*member)(void)) {
        attach(object, member);
    }

    /** Attach a static function
     * 
     *  @param function The void static function to attach (default is none)
     */
    void attach(void (*function)(void) = 0);
    
    /** Attach a member function
     * 
     *  @param object The object pointer to invoke the member function on (i.e. the this pointer)
     *  @param function The address of the void member function to attach 
     */
    template<typename T>
    void attach(T *object, void (T::*member)(void)) {
        _object = static_cast<void*>(object);
        memcpy(_member, (char*)&member, sizeof(member));
        _membercaller = &FunctionPointer::membercaller<T>;
        _function = 0;
    }

    /** Call the attached static or member function
     */        
    void call();
        
private:

    template<typename T>
    static void membercaller(void *object, char *member) {    
        T* o = static_cast<T*>(object);
        void (T::*m)(void);
        memcpy((char*)&m, member, sizeof(m));
        (o->*m)();
    }
    
    /** Static function pointer - 0 if none attached
     */
    void (*_function)(void);
    
    /** Object this pointer - 0 if none attached
     */
    void *_object;
    
    /** Raw member function pointer storage - converted back by registered _membercaller
     */
    char _member[16];
    
    /** Registered membercaller function to convert back and call _member on _object
     */
    void (*_membercaller)(void*, char*);
    
};

} // namespace mbed

#endif
