#ifndef PLANEGCS_NATIVE_CONSOLE_H
#define PLANEGCS_NATIVE_CONSOLE_H

// Native replacement for planegcs/headers/Console.h (which depends on Emscripten).
// Put this directory before planegcs/headers in the include path.
//
// PlaneGCS writes some diagnostics through Console::Log unconditionally, so the
// default build keeps the library quiet. Configure with
// -DPLANEGCS_NATIVE_VERBOSE=ON to mirror the messages to stderr.

#include <cstdarg>
#include <cstdio>

class Console
{
public:
    static void Log(const char* format, ...)
    {
#ifdef PLANEGCS_NATIVE_VERBOSE
        va_list args;
        va_start(args, format);
        std::vfprintf(stderr, format, args);
        va_end(args);
#else
        (void)format;
#endif
    }
};

#endif  // PLANEGCS_NATIVE_CONSOLE_H
