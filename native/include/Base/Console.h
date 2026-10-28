#ifndef PLANEGCS_NATIVE_BASE_CONSOLE_H
#define PLANEGCS_NATIVE_BASE_CONSOLE_H

// Native replacement for FreeCAD's Base/Console.h.
//
// The upstream planegcs sources write diagnostics through
// Base::Console().log()/warning() with std::format-style placeholders. The
// native build keeps the library silent unless PLANEGCS_NATIVE_VERBOSE is
// defined.

#include <cstdio>
#include <format>
#include <string>
#include <utility>

namespace Base
{

class Console
{
public:
    template<typename... Args>
    void log(std::format_string<Args...> format, Args&&... args) const
    {
#ifdef PLANEGCS_NATIVE_VERBOSE
        const std::string message = std::format(format, std::forward<Args>(args)...);
        std::fputs(message.c_str(), stderr);
#else
        (void)format;
        ((void)args, ...);
#endif
    }

    template<typename... Args>
    void warning(std::format_string<Args...> format, Args&&... args) const
    {
#ifdef PLANEGCS_NATIVE_VERBOSE
        const std::string message = std::format(format, std::forward<Args>(args)...);
        std::fputs(message.c_str(), stderr);
#else
        (void)format;
        ((void)args, ...);
#endif
    }
};

}  // namespace Base

#endif  // PLANEGCS_NATIVE_BASE_CONSOLE_H
