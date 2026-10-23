#ifndef PLANEGCS_NATIVE_PRELUDE_H
#define PLANEGCS_NATIVE_PRELUDE_H

// The upstream sources were prepared for Emscripten/libstdc++, which pulls in
// these headers transitively. MSVC does not, so force-include them for the
// upstream translation units (/FI on MSVC).

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numbers>
#include <vector>

#endif  // PLANEGCS_NATIVE_PRELUDE_H
