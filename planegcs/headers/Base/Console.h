#ifndef PLANEGCS_BASE_CONSOLE_H
#define PLANEGCS_BASE_CONSOLE_H

// Minimal replacement for FreeCAD's Base/Console.h used by the non-native
// (Emscripten) build. The native build has its own shim in native/include,
// which takes precedence on the include path.

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
        const std::string message = std::format(format, std::forward<Args>(args)...);
        std::fputs(message.c_str(), stdout);
    }

    template<typename... Args>
    void warning(std::format_string<Args...> format, Args&&... args) const
    {
        const std::string message = std::format(format, std::forward<Args>(args)...);
        std::fputs(message.c_str(), stderr);
    }
};

}  // namespace Base

#endif  // PLANEGCS_BASE_CONSOLE_H
