#ifndef INC_COMMON_DEFINES_H_
#define INC_COMMON_DEFINES_H_

#include <stdint.h>

#ifdef NDEBUG
#define ASSERT(expr) ((void)0) // No operation if NDEBUG is defined
#else
#include "assert.h"
#define ASSERT(expr)              \
    ((expr) ? (void)0 : assert_failed(__FILE__, __LINE__, __FUNCTION__, #expr))

    void assert_failed(const char* file, int line, const char* function, const char* expr);

#endif // NDEBUG

#endif