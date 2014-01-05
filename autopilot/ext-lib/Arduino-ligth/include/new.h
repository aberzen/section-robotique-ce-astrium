/* Header to define new/delete operators as they aren't provided by avr-gcc by default
   Taken from http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=59453 
 */

#ifndef NEW_H
#define NEW_H

#include <stdlib.h>

#ifdef __cplusplus

/** C++ new operator */
void * operator new(size_t size);

/** C++ delete operator */
void operator delete(void * ptr); 

#endif /* __cplusplus */

__extension__ typedef int __guard __attribute__((mode (__DI__)));

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

int __cxa_guard_acquire(__guard *);
void __cxa_guard_release (__guard *);
void __cxa_guard_abort (__guard *);
void __cxa_pure_virtual(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif

