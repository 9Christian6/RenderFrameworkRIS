#ifndef HASH_H
#define HASH_H

/// Returns the initializer for Bernstein's hash function
inline uint32_t bernstein_init() { return 5381; }

/// Hashes 4 bytes using Bernstein's hash
inline uint32_t bernstein_hash(uint32_t h, uint32_t d) {
    h = (h * 33) ^ ( d        & 0xFF);
    h = (h * 33) ^ ((d >>  8) & 0xFF);
    h = (h * 33) ^ ((d >> 16) & 0xFF);
    h = (h * 33) ^ ((d >> 24) & 0xFF);
    return h;
}

/// Returns the initializer for the FNV hash function
inline uint32_t fnv_init() { return 0x811C9DC5; }

/// Hashes 4 bytes using FNV
inline uint32_t fnv_hash(uint32_t h, uint32_t d) {
    h = (h * 16777619) ^ ( d        & 0xFF);
    h = (h * 16777619) ^ ((d >>  8) & 0xFF);
    h = (h * 16777619) ^ ((d >> 16) & 0xFF);
    h = (h * 16777619) ^ ((d >> 24) & 0xFF);
    return h;
}

/// Returns a seed for a sampler object, based on the current pixel id and iteration count
inline uint32_t sampler_seed(uint32_t pixel, uint32_t iter) {
    return fnv_hash(fnv_hash(fnv_init(), pixel), iter);
}

#endif // HASH_H
