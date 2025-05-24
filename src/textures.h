#ifndef TEXTURES_H
#define TEXTURES_H

#include "color.h"
#include "image.h"

/// Base class for all textures
class Texture {
public:
    virtual ~Texture() {}
    virtual rgb operator () (float u, float v) const = 0;
};

/// Constant texture, returns the same value everywhere.
class ConstantTexture : public Texture {
public:
    ConstantTexture(const rgb& c) : color(c) {}
    rgb operator () (float, float) const override final { return color; }

private:
    rgb color;
};

/// Image-based texture, using bilinear filtering.
class ImageTexture : public Texture {
public:
    ImageTexture(Image&& img) : img(img) {}

    rgb operator () (float u, float v) const override final {
        u = u - (int)u;
        u = u < 0.0f ? 1.0f + u : u;
        v = v - (int)v;
        v = v < 0.0f ? 1.0f + v : v;
        v = 1.0f - v;
        auto kx = u * img.width;;
        auto ky = v * img.height;
        auto ix = size_t(kx);
        auto iy = size_t(ky);
        auto fx = kx - ix;
        auto fy = ky - iy;
        auto x0 = clamp(ix, size_t(0), img.width - 1);
        auto y0 = clamp(iy, size_t(0), img.height - 1);
        auto x1 = x0 + 1 >= img.width  ? 0 : x0 + 1;
        auto y1 = y0 + 1 >= img.height ? 0 : y0 + 1;
        return lerp(lerp(rgb(img(x0, y0)), rgb(img(x1, y0)), fx),
                    lerp(rgb(img(x0, y1)), rgb(img(x1, y1)), fx),
                    fy);
    }

    const Image& image() const { return img; }

private:
    Image img;
};

#endif // TEXTURES_h
