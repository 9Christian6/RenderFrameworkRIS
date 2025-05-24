#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <cstdlib>
#include <cstdint>
#include <random>

#define UNUSED(x) (void)(x)

#if defined(__GNUC__) || defined(__clang__)
#define likely(x) (__builtin_expect((x), true))
#define unlikely(x) (__builtin_expect((x), false))
#else
#define likely(x) (x)
#define unlikely(x) (x)
#endif

static constexpr float pi = 3.14159265359f;

/// Converts degrees to radians.
inline float radians(float x) {
    return x * pi / 180.0f;
}

/// Converts radians to degrees.
inline float degrees(float x) {
    return x * 180.0f / pi;
}

/// Clamps a between b and c.
template <typename T>
inline T clamp(T a, T b, T c) {
    return (a < b) ? b : ((a > c) ? c : a);
}

/// Returns the integer that is greater or equal to the logarithm base 2 of the argument.
template <typename T>
inline T closest_log2(T i) {
    T p = 1, q = 0;
    while (i > p) p <<= 1, q++;
    return q;
}

/// Reinterprets a floating point number as an integer.
inline int32_t float_as_int(float f) {
    union { float vf; int32_t vi; } v;
    v.vf = f;
    return v.vi;
}

/// Reinterprets an integer as a floating point number.
inline float int_as_float(int32_t i) {
    union { float vf; int32_t vi; } v;
    v.vi = i;
    return v.vf;
}

/// Returns the x with the sign of the product x * y.
inline float prodsign(float x, float y) {
    return int_as_float(float_as_int(x) ^ (float_as_int(y) & 0x80000000));
}

/// Linearly interpolates between two values.
template <typename T, typename U>
T lerp(T a, T b, U u) {
    return a * (1 - u) + b * u;
}

/// Linearly interpolates between three values.
template <typename T, typename U>
T lerp(T a, T b, T c, U u, U v) {
    return a * (1 - u - v) + b * u + c * v;
}

/// Reflects the vector w.r.t the given plane.
template <typename T>
T reflect(T v, T n) {
    return (2 * dot(n, v)) * n - v;
}

inline void error() {
    std::cerr << std::endl;
}

/// Outputs an error message in the console.
template <typename T, typename... Args>
inline void error(T t, Args... args) {
#if COLORIZE_LOG
    std::cerr << "\033[1;31m";
#endif
    std::cerr << t;
#if COLORIZE_LOG
    std::cerr << "\033[0m";
#endif
    error(args...);
}

inline void info() {
    std::cout << std::endl;
}

/// Outputs an information message in the console.
template <typename T, typename... Args>
inline void info(T t, Args... args) {
    std::cout << t;
    info(args...);
}

inline void warn() {
    std::clog << std::endl;
}

/// Outputs an warning message in the console.
template <typename T, typename... Args>
inline void warn(T t, Args... args) {
#if COLORIZE_LOG
    std::clog << "\033[1;33m";
#endif
    std::clog << t;
#if COLORIZE_LOG
    std::clog << "\033[0m";
#endif
    warn(args...);
}

#define assert_normalized(x) check_normalized(x, __FILE__, __LINE__)

template <typename T>
inline void check_normalized(const T& n, const char* file, size_t line) {
#ifdef CHECK_NORMALS
    const float len = length(n);
    const float tolerance = 0.001f;
    if (len < 1.0f - tolerance || len > 1.0f + tolerance) {
        error("Vector not normalized in \'", file, "\', line ", line);
        abort();
    }
#endif
}

/// Class that represents a value that has to be treated atomically.
template <typename T>
struct Atom {
    T value;
    Atom(const T& t) : value(t) {}
};

/// Wraps a value into an Atom structure.
template <typename T>
Atom<T> atomically(const T& t) {
    return Atom<T>(t);
}

#endif // COMMON_H
