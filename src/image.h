#ifndef IMAGE_H
#define IMAGE_H

#include <string>
#include <vector>

#include "color.h"

struct Image {
    Image() {}
    Image(size_t w, size_t h)
        : pixels(w * h), width(w), height(h)
    {}

    const rgba& operator () (size_t x, size_t y) const { return pixels[y * width + x]; }
    rgba& operator () (size_t x, size_t y) { return pixels[y * width + x]; }

    const rgba* row(size_t y) const { return &pixels[y * width]; }
    rgba* row(size_t y) { return &pixels[y * width]; }

    void resize(size_t w, size_t h) {
        width = w;
        height = h;
        pixels.resize(w * h);
    }

    void clear() {
        std::fill(pixels.begin(), pixels.end(), rgba(0.0f, 0.0f, 0.0f, 0.0f));
    }

    std::vector<rgba> pixels;
    size_t width, height;
};

/// Loads an image from a PNG file.
bool load_png(const std::string& png_file, Image& image);
/// Stores an image as a PNG file.
bool save_png(const std::string& png_file, const Image& image);

/// Loads an image from a TGA file.
bool load_tga(const std::string& tga_file, Image& image);

/// Loads an image from a JPEG file.
bool load_jpeg(const std::string& jpeg_file, Image& image);

/// Loads an image from a TIFF file.
bool load_tiff(const std::string& tiff_file, Image& image);

/// Loads an image from an EXR file.
bool load_exr(const std::string& exr_file, Image& image);
/// Stores an image as an EXR file.
bool save_exr(const std::string& exr_file, const Image& image);

#endif // IMAGE_H
