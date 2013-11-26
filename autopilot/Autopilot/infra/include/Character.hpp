/*
 WCharacter.h - Character utility functions for Wiring & Arduino
 Copyright (c) 2010 Hernando Barragan.  All right reserved.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef Character_h
#define Character_h

#include <ctype.h>
#include <string.h>

// prog_char_t is used as a wrapper type for prog_char, which is
// a character stored in flash. By using this wrapper type we can
// auto-detect at compile time if a call to a string function is using
// a flash-stored string or not
typedef struct {
    char c;
} prog_char_t;

// The following is strictly for type-checking arguments to printf_P calls
// in conjunction with a suitably modified Arduino IDE; never define for
// production as it generates bad code.
//
#if PRINTF_FORMAT_WARNING_DEBUG
 # undef PSTR
 # define PSTR(_x)               _x             // help the compiler with printf_P
 # define float double                  // silence spurious format warnings for %f
#else
// This is a workaround for GCC bug c++/34734.
//
// The C++ compiler normally emits many spurious warnings for the use
// of PSTR (even though it generates correct code).  This workaround
// has an equivalent effect but avoids the warnings, which otherwise
// make finding real issues difficult.
//
 #ifdef DESKTOP_BUILD
  # undef PROGMEM
  # define PROGMEM __attribute__(())
 #else
  # undef PROGMEM
  # define PROGMEM __attribute__(( section(".progmem.data") ))
 #endif

 # undef PSTR
  /* Need const type for progmem - new for avr-gcc 4.6 */
  # if __AVR__ && __GNUC__ == 4 && __GNUC_MINOR__ > 5
 # define PSTR(s) (__extension__({static const prog_char __c[] PROGMEM = (s); \
                                  (const prog_char_t *)&__c[0]; }))
  #else
 # define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); \
                                  (prog_char_t *)&__c[0]; }))
  #endif
#endif

// a varient of PSTR() for progmem strings passed to %S in printf()
// this gets the gcc __format__ checking right
#define FPSTR(s) (wchar_t *)(s)

/// Define a constant string in program memory.  This is a little more obvious
/// and less error-prone than typing the declaration out by hand.  It's required
/// when passing PROGMEM strings to static object constructors because the PSTR
/// hack can't be used at global scope.
///
#define PROGMEM_STRING(_v, _s)  static const char _v[] PROGMEM = _s



// Character.hpp prototypes
inline int strcasecmp_P(const char *str1, const prog_char_t *pstr);
inline int strcmp_P(const char *str1, const prog_char_t *pstr);
inline size_t strlen_P(const prog_char_t *pstr);
inline void* memcpy_P(void *dest, const prog_char_t *src, size_t n);
inline size_t strlcat_P(char *d, const prog_char_t *s, size_t bufsize);
inline char* strncpy_P(char *buffer, const prog_char_t *pstr, size_t buffer_size);
inline uintptr_t pgm_read_pointer(const void *s);

inline bool isAlphaNumeric(int c) __attribute__((always_inline));
inline bool isAlpha(int c) __attribute__((always_inline));
inline bool isAscii(int c) __attribute__((always_inline));
inline bool isWhitespace(int c) __attribute__((always_inline));
inline bool isControl(int c) __attribute__((always_inline));
inline bool isDigit(int c) __attribute__((always_inline));
inline bool isGraph(int c) __attribute__((always_inline));
inline bool isLowerCase(int c) __attribute__((always_inline));
inline bool isPrintable(int c) __attribute__((always_inline));
inline bool isPunct(int c) __attribute__((always_inline));
inline bool isSpace(int c) __attribute__((always_inline));
inline bool isUpperCase(int c) __attribute__((always_inline));
inline bool isHexadecimalDigit(int c) __attribute__((always_inline));
inline int toAscii(int c) __attribute__((always_inline));
inline int toLowerCase(int c) __attribute__((always_inline));
inline int toUpperCase(int c)__attribute__((always_inline));


inline int        strcasecmp_P(const char *str1, const prog_char_t *pstr)
{
    return strcasecmp_P(str1, (const prog_char *)pstr);
}

inline int        strcmp_P(const char *str1, const prog_char_t *pstr)
{
    return strcmp_P(str1, (const prog_char *)pstr);
}

inline size_t        strlen_P(const prog_char_t *pstr)
{
    return strlen_P((const prog_char *)pstr);
}

inline void *      memcpy_P(void *dest, const prog_char_t *src, size_t n)
{
    return memcpy_P(dest, (const prog_char *)src, n);
}

// strlcat_P() in AVR libc seems to be broken
inline size_t        strlcat_P(char *d, const prog_char_t *s, size_t bufsize)
{
    size_t          len1 = strlen(d);
    size_t          len2 = strlen_P(s);
    size_t          ret = len1 + len2;

    if (len1+len2 >= bufsize) {
        if (bufsize < (len1+1)) {
            return ret;
        }
        len2 = bufsize - (len1+1);
    }
    if (len2 > 0) {
        memcpy_P(d+len1, s, len2);
        d[len1+len2] = 0;
    }
    return ret;
}

inline char *      strncpy_P(char *buffer, const prog_char_t *pstr, size_t buffer_size)
{
    return strncpy_P(buffer, (const prog_char *)pstr, buffer_size);
}


// read something the size of a pointer. This makes the menu code more
// portable
inline uintptr_t        pgm_read_pointer(const void *s)
{
    if (sizeof(uintptr_t) == sizeof(uint16_t)) {
        return (uintptr_t)pgm_read_word(s);
    } else {
        union {
            uintptr_t p;
            uint8_t a[sizeof(uintptr_t)];
        } u;
        uint8_t        i;
        for (i=0; i< sizeof(uintptr_t); i++) {
            u.a[i] = pgm_read_byte(i + (const prog_char *)s);
        }
        return u.p;
    }
}


// Checks for an alphanumeric character. 
// It is equivalent to (isalpha(c) || isdigit(c)).
inline bool isAlphaNumeric(int c)
{
  return ( isalnum(c) == 0 ? false : true);
}


// Checks for an alphabetic character. 
// It is equivalent to (isupper(c) || islower(c)).
inline bool isAlpha(int c)
{
  return ( isalpha(c) == 0 ? false : true);
}


// Checks whether c is a 7-bit unsigned char value 
// that fits into the ASCII character set.
inline bool isAscii(int c)
{
  return ( isascii (c) == 0 ? false : true);
}


// Checks for a blank character, that is, a space or a tab.
inline bool isWhitespace(int c)
{
  return ( isblank (c) == 0 ? false : true);
}


// Checks for a control character.
inline bool isControl(int c)
{
  return ( iscntrl (c) == 0 ? false : true);
}


// Checks for a digit (0 through 9).
inline bool isDigit(int c)
{
  return ( isdigit (c) == 0 ? false : true);
}


// Checks for any printable character except space.
inline bool isGraph(int c)
{
  return ( isgraph (c) == 0 ? false : true);
}


// Checks for a lower-case character.
inline bool isLowerCase(int c)
{
  return (islower (c) == 0 ? false : true);
}


// Checks for any printable character including space.
inline bool isPrintable(int c)
{
  return ( isprint (c) == 0 ? false : true);
}


// Checks for any printable character which is not a space 
// or an alphanumeric character.
inline bool isPunct(int c)
{
  return ( ispunct (c) == 0 ? false : true);
}


// Checks for white-space characters. For the avr-libc library, 
// these are: space, formfeed ('\f'), newline ('\n'), carriage 
// return ('\r'), horizontal tab ('\t'), and vertical tab ('\v').
inline bool isSpace(int c)
{
  return ( isspace (c) == 0 ? false : true);
}


// Checks for an uppercase letter.
inline bool isUpperCase(int c)
{
  return ( isupper (c) == 0 ? false : true);
}


// Checks for a hexadecimal digits, i.e. one of 0 1 2 3 4 5 6 7 
// 8 9 a b c d e f A B C D E F.
inline bool isHexadecimalDigit(int c)
{
  return ( isxdigit (c) == 0 ? false : true);
}


// Converts c to a 7-bit unsigned char value that fits into the 
// ASCII character set, by clearing the high-order bits.
inline int toAscii(int c)
{
  return toascii (c);
}


// Warning:
// Many people will be unhappy if you use this function. 
// This function will convert accented letters into random 
// characters.

// Converts the letter c to lower case, if possible.
inline int toLowerCase(int c)
{
  return tolower (c);
}


// Converts the letter c to upper case, if possible.
inline int toUpperCase(int c)
{
  return toupper (c);
}

#endif
