#include <fstream>
#include <cassert>
#include <cstring>
#include <memory>
#include <exception>
#include <vector>

#include <png.h>
#include <jpeglib.h>
#include <tiffio.h>
#define TINYEXR_IMPLEMENTATION
#include <tinyexr.h>

#include "image.h"

static void png_read_from_stream(png_structp png_ptr, png_bytep data, png_size_t length) {
    png_voidp a = png_get_io_ptr(png_ptr);
    ((std::istream*)a)->read((char*)data, length);
}

static void png_write_to_stream(png_structp png_ptr, png_bytep data, png_size_t length) {
    png_voidp a = png_get_io_ptr(png_ptr);
    ((std::ostream*)a)->write((const char*)data, length);
}

static void png_flush_stream(png_structp) {
    // Nothing to do
}

bool load_png(const std::string& png_file, Image& image) {
    std::ifstream file(png_file, std::ifstream::binary);
    if (!file)
        return false;

    // Read signature
    char sig[8];
    file.read(sig, 8);
    if (!png_check_sig((unsigned char*)sig, 8))
        return false;

    png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png_ptr)
        return false;

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        png_destroy_read_struct(&png_ptr, nullptr, nullptr);
        return false;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
        return false;
    }

    png_set_sig_bytes(png_ptr, 8);
    png_set_read_fn(png_ptr, (png_voidp)&file, png_read_from_stream);
    png_read_info(png_ptr, info_ptr);

    size_t width  = png_get_image_width(png_ptr, info_ptr);
    size_t height = png_get_image_height(png_ptr, info_ptr);

    png_uint_32 color_type = png_get_color_type(png_ptr, info_ptr);
    png_uint_32 bit_depth  = png_get_bit_depth(png_ptr, info_ptr);

    // Expand paletted and grayscale images to RGB
    if (color_type == PNG_COLOR_TYPE_PALETTE) {
        png_set_palette_to_rgb(png_ptr);
    } else if (color_type == PNG_COLOR_TYPE_GRAY ||
               color_type == PNG_COLOR_TYPE_GRAY_ALPHA) {
        png_set_gray_to_rgb(png_ptr);
    }

    // Transform to 8 bit per channel
    if (bit_depth == 16)
        png_set_strip_16(png_ptr);

    // Get alpha channel when there is one
    if (png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS))
        png_set_tRNS_to_alpha(png_ptr);

    // Otherwise add an opaque alpha channel
    else
        png_set_filler(png_ptr, 0xFF, PNG_FILLER_AFTER);

    image.resize(width, height);
    std::vector<png_byte> row_bytes(width * 4);
    for (size_t y = 0; y < height; y++) {
        png_read_row(png_ptr, row_bytes.data(), nullptr);
        rgba* img_row = image.row(y);
        for (size_t x = 0; x < width; x++) {
            img_row[x].x = row_bytes[x * 4 + 0] / 255.0f;
            img_row[x].y = row_bytes[x * 4 + 1] / 255.0f;
            img_row[x].z = row_bytes[x * 4 + 2] / 255.0f;
            img_row[x].w = row_bytes[x * 4 + 3] / 255.0f;
        }
    }

    png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);

    return true;
}

bool save_png(const std::string& png_file, const Image& image) {
    std::ofstream file(png_file, std::ofstream::binary);
    if (!file)
        return false;

    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png_ptr)
        return false;

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        png_destroy_read_struct(&png_ptr, nullptr, nullptr);
        return false;
    }

    std::vector<uint8_t> row(image.width * 4);
    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_write_struct(&png_ptr, &info_ptr);
        return false;
    }

    png_set_write_fn(png_ptr, &file, png_write_to_stream, png_flush_stream);

    png_set_IHDR(png_ptr, info_ptr, image.width, image.height,
                 8, PNG_COLOR_TYPE_RGBA, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

    png_write_info(png_ptr, info_ptr);

    for (size_t y = 0; y < image.height; y++) {
        const rgba* input = image.row(y);
        for (size_t x = 0; x < image.width; x++) {
            row[x * 4 + 0] = clamp(input[x].x, 0.0f, 1.0f) * 255.0f;
            row[x * 4 + 1] = clamp(input[x].y, 0.0f, 1.0f) * 255.0f;
            row[x * 4 + 2] = clamp(input[x].z, 0.0f, 1.0f) * 255.0f;
            row[x * 4 + 3] = clamp(input[x].w, 0.0f, 1.0f) * 255.0f;
        }
        png_write_row(png_ptr, row.data());
    }

    png_write_end(png_ptr, info_ptr);
    png_destroy_write_struct(&png_ptr, &info_ptr);

    return true;
}

struct TgaHeader
{
    unsigned short width;
    unsigned short height;
    unsigned char bpp;
    unsigned char desc;
};

enum TgaType {
    TGA_NONE,
    TGA_RAW,
    TGA_COMP
};

inline TgaType check_signature(const char* sig) {
    const char raw_sig[12] = {0,0,2, 0,0,0,0,0,0,0,0,0};
    const char comp_sig[12] = {0,0,10,0,0,0,0,0,0,0,0,0};

    if (!std::memcmp(sig, raw_sig, sizeof(char) * 12))
        return TGA_RAW;

    if (!std::memcmp(sig, comp_sig, sizeof(char) * 12))
        return TGA_COMP;

    return TGA_NONE;
}

inline void copy_pixels24(rgba* img, const unsigned char* pixels, int n) {
    for (int i = 0; i < n; i++) {
        img[i].z = pixels[i * 3 + 0] / 255.0f;
        img[i].y = pixels[i * 3 + 1] / 255.0f;
        img[i].x = pixels[i * 3 + 2] / 255.0f;
        img[i].w = 1.0f;
    }
}

inline void copy_pixels32(rgba* img, const unsigned char* pixels, int n) {
    for (int i = 0; i < n; i++) {
        img[i].z = pixels[i * 4 + 0] / 255.0f;
        img[i].y = pixels[i * 4 + 1] / 255.0f;
        img[i].x = pixels[i * 4 + 2] / 255.0f;
        img[i].w = pixels[i * 4 + 3] / 255.0f;
    }
}

static void load_raw_tga(const TgaHeader& tga, std::istream& stream, Image& image) {
    assert(tga.bpp == 24 || tga.bpp == 32);

    if (tga.bpp == 24) {
        std::vector<char> tga_row(3 * tga.width);
        for (int y = 0; y < tga.height; y++) {
            rgba* row = image.row(tga.height - y - 1);
            stream.read(tga_row.data(), tga_row.size());
            copy_pixels24(row, (unsigned char*)tga_row.data(), tga.width);
        }
    } else {
        std::vector<char> tga_row(4 * tga.width);
        for (int y = 0; y < tga.height; y++) {
            rgba* row = image.row(tga.height - y - 1);
            stream.read(tga_row.data(), tga_row.size());
            copy_pixels32(row, (unsigned char*)tga_row.data(), tga.width);
        }
    }
}

static void load_compressed_tga(const TgaHeader& tga, std::istream& stream, Image& image) {
    assert(tga.bpp == 24 || tga.bpp == 32);

    const int pix_count = tga.width * tga.height;
    int cur_pix = 0;
    while (cur_pix < pix_count) {
        unsigned char chunk;
        stream.read((char*)&chunk, 1);

        if (chunk < 128) {
            chunk++;

            char pixels[4 * 128];
            stream.read(pixels, chunk * (tga.bpp / 8));
            if (cur_pix + chunk > pix_count) chunk = pix_count - cur_pix;

            if (tga.bpp == 24) {
                copy_pixels24(image.pixels.data() + cur_pix, (unsigned char*)pixels, chunk);
            } else {
                copy_pixels32(image.pixels.data() + cur_pix, (unsigned char*)pixels, chunk);
            }

            cur_pix += chunk;
        } else {
            chunk -= 127;

            unsigned char tga_pix[4];
            tga_pix[3] = 255;
            stream.read((char*)tga_pix, (tga.bpp / 8));

            if (cur_pix + chunk > pix_count) chunk = pix_count - cur_pix;

            rgba* pix = image.pixels.data() + cur_pix;
            const rgba c(tga_pix[2] / 255.0f,
                         tga_pix[1] / 255.0f,
                         tga_pix[0] / 255.0f,
                         tga_pix[3] / 255.0f);
            for (int i = 0; i < chunk; i++)
                pix[i] = c;

            cur_pix += chunk;
        }
    }
}

bool load_tga(const std::string& tga_file, Image& image) {
    std::ifstream file(tga_file, std::ifstream::binary);
    if (!file)
        return false;

    // Read signature
    char sig[12];
    file.read(sig, 12);
    TgaType type = check_signature(sig);
    if (type == TGA_NONE)
        return false;

    TgaHeader header;
    file.read((char*)&header, sizeof(TgaHeader));
    if (!file) return false;

    if (header.width <= 0 || header.height <= 0 ||
        (header.bpp != 24 && header.bpp !=32)) {
        return false;
    }

    image.resize(header.width, header.height);

    if (type == TGA_RAW) {
        load_raw_tga(header, file, image);
    } else {
        load_compressed_tga(header, file, image);
    }

    return true;
}

struct enhanced_jpeg_decompress_struct : jpeg_decompress_struct {
    jmp_buf jmp;
    std::istream* is;
    JOCTET src_buf[1024];
};

static void jpeg_error_exit(j_common_ptr cinfo) {
    cinfo->err->output_message(cinfo);
    longjmp(reinterpret_cast<enhanced_jpeg_decompress_struct*>(cinfo)->jmp, 1);
}

static void jpeg_output_message(j_common_ptr) {}

static void jpeg_no_op(j_decompress_ptr) {}

static boolean jpeg_fill_input_buffer(j_decompress_ptr cinfo) {
    auto enhanced = static_cast<enhanced_jpeg_decompress_struct*>(cinfo);
    enhanced->is->read((char*)enhanced->src_buf, 1024);
    cinfo->src->bytes_in_buffer = enhanced->is->gcount();
    cinfo->src->next_input_byte = enhanced->src_buf;
    return TRUE;
}

static void jpeg_skip_input_data(j_decompress_ptr cinfo, long num_bytes) {
    auto enhanced = static_cast<enhanced_jpeg_decompress_struct*>(cinfo);
    if (num_bytes != 0) {
        if (num_bytes < long(cinfo->src->bytes_in_buffer)) {
            cinfo->src->next_input_byte += num_bytes;
            cinfo->src->bytes_in_buffer -= num_bytes;
        } else {
            enhanced->is->seekg(num_bytes - cinfo->src->bytes_in_buffer, std::ios_base::cur);
            cinfo->src->bytes_in_buffer = 0;
        }
    }
}

bool load_jpeg(const std::string& jpeg_file, Image& image) {
    std::ifstream file(jpeg_file, std::ifstream::binary);
    if (!file)
        return false;

    enhanced_jpeg_decompress_struct cinfo;
    cinfo.is = &file;
    jpeg_error_mgr jerr;

    cinfo.err           = jpeg_std_error(&jerr);
    jerr.error_exit     = jpeg_error_exit;
    jerr.output_message = jpeg_output_message;
    jpeg_create_decompress(&cinfo);

    if (setjmp(cinfo.jmp)) {
        jpeg_abort_decompress(&cinfo);
        jpeg_destroy_decompress(&cinfo);
        return false;
    }

    jpeg_source_mgr src;
    src.init_source       = jpeg_no_op;
    src.fill_input_buffer = jpeg_fill_input_buffer;
    src.skip_input_data   = jpeg_skip_input_data;
    src.resync_to_restart = jpeg_resync_to_restart;
    src.term_source       = jpeg_no_op;
    src.bytes_in_buffer   = 0;
    cinfo.src = &src;

    jpeg_read_header(&cinfo, true);
    jpeg_start_decompress(&cinfo);
    image.resize(cinfo.output_width, cinfo.output_height);
    size_t channels = cinfo.output_components;

    std::unique_ptr<JSAMPLE[]> row(new JSAMPLE[image.width * channels]);
    for (size_t y = 0; y < image.height; y++) {
        auto src_ptr = row.get();
        auto dst_ptr = &image.pixels[y * image.width].x;
        jpeg_read_scanlines(&cinfo, &src_ptr, 1);
        for (size_t x = 0; x < image.width; ++x, src_ptr += channels, dst_ptr += 4) {
            for (size_t c = 0; c < channels; c++)
                dst_ptr[c] = src_ptr[c] * (1.0f / ((1 << BITS_IN_JSAMPLE) - 1));
        }
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    return true;
}

tsize_t tiff_read_from_stream(thandle_t st, tdata_t buffer, tsize_t size) {
    auto stream = reinterpret_cast<std::istream*>(st);
    stream->read((char*)buffer, size);
    return stream->gcount();
}

tsize_t tiff_write_to_stream(thandle_t, tdata_t, tsize_t) { return 0; }
int tiff_close(thandle_t) { return 0; }

toff_t tiff_seek(thandle_t st, toff_t pos, int whence) {
    auto stream = reinterpret_cast<std::istream*>(st);
    auto from = std::ios::beg;
    if (whence == SEEK_CUR)
        from = std::ios::cur;
    else if (whence == SEEK_END)
        from = std::ios::end;
    stream->seekg(pos, from);
    return stream->tellg();
}

toff_t tiff_size(thandle_t st) {
    auto stream = reinterpret_cast<std::istream*>(st);
    auto old = stream->tellg();
    stream->seekg(0);
    auto size = stream->tellg();
    stream->seekg(old);
    return size;;
}

int tiff_map(thandle_t, tdata_t*, toff_t*) { return 0; }
void tiff_unmap(thandle_t, tdata_t, toff_t) {}

void tiff_error_handler(const char*, const char*, va_list) {}

bool load_tiff(const std::string& tiff_file, Image& image) {
    std::ifstream file(tiff_file, std::ifstream::binary);
    if (!file)
        return false;

    TIFFSetErrorHandler(tiff_error_handler);
    TIFFSetWarningHandler(tiff_error_handler);
    auto tif = TIFFClientOpen(tiff_file.c_str(), "r", (thandle_t)&file,
                              tiff_read_from_stream, tiff_write_to_stream,
                              tiff_seek, tiff_close, tiff_size, tiff_map, tiff_unmap);
    if (!tif)
        return false;

    uint32_t width, height;
	TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &width);
	TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &height);
    image.resize(width, height);

	std::unique_ptr<uint32_t[]> pixels(new uint32_t[width * height]);
	TIFFReadRGBAImageOriented(tif, width, height, pixels.get(), ORIENTATION_TOPLEFT, 0);
    TIFFClose(tif);

    for (size_t y = 0; y < height; y++) {
        for (size_t x = 0; x < width; x++) {
            auto pixel = pixels[y * width + x];
            auto r = (pixel         & 0xFF) * (1.0f / 255.0f);
            auto g = ((pixel >>  8) & 0xFF) * (1.0f / 255.0f);
            auto b = ((pixel >> 16) & 0xFF) * (1.0f / 255.0f);
            auto a = ((pixel >> 24) & 0xFF) * (1.0f / 255.0f);
            image.pixels[y * width + x] = rgba(r, g, b, a);
        }
    }
    return true;
}

bool load_exr(const std::string& exr_file, Image& image) {
    std::ifstream file(exr_file, std::ifstream::binary);
    if (!file)
        return false;

    std::vector<unsigned char> buffer(std::istreambuf_iterator<char>(file), {});
    EXRVersion exr_version;
    if (ParseEXRVersionFromMemory(&exr_version, buffer.data(), buffer.size()))
        return false;

    const char* err = nullptr;
    EXRHeader exr_header;
    EXRImage exr_image;
    int channel_r = 0;
    int channel_g = 0;
    int channel_b = 0;
    int channel_a = 0;

    InitEXRHeader(&exr_header);
    InitEXRImage(&exr_image);
    if (ParseEXRHeaderFromMemory(&exr_header, &exr_version, buffer.data(), buffer.size(), &err) || exr_header.num_channels == 0)
        goto error;
    if (LoadEXRImageFromMemory(&exr_image, &exr_header, buffer.data(), buffer.size(), &err))
        goto error;

    image.resize(exr_image.width, exr_image.height);
    for (int i = 0; i < exr_header.num_channels; ++i) {
        const char* name = exr_header.channels[i].name;
        if (!strcmp(name, "r") || !strcmp(name, "R"))
            channel_r = i;
        else if (!strcmp(name, "g") || !strcmp(name, "G"))
            channel_g = i;
        else if (!strcmp(name, "b") || !strcmp(name, "B"))
            channel_b = i;
        else if (!strcmp(name, "a") || !strcmp(name, "A"))
            channel_a = i;
    }
    if (exr_image.images) {
        for (size_t y = 0; y < image.height; ++y) {
            for (size_t x = 0; x < image.width; ++x) {
                float r = ((float*)exr_image.images[channel_r])[y * image.width + x];
                float g = ((float*)exr_image.images[channel_g])[y * image.width + x];
                float b = ((float*)exr_image.images[channel_b])[y * image.width + x];
                float a = ((float*)exr_image.images[channel_a])[y * image.width + x];
                image.pixels[y * image.width + x] = rgba(r, g, b, a);
            }
        }
    } else {
        for (int i = 0; i < exr_image.num_tiles; ++i) {
            auto& tile = exr_image.tiles[i];
            for (int y = 0; y < tile.height; ++y) {
                for (int x = 0; x < tile.width; ++x) {
                    float r = ((float*)tile.images[channel_r])[y * tile.width + x];
                    float g = ((float*)tile.images[channel_g])[y * tile.width + x];
                    float b = ((float*)tile.images[channel_b])[y * tile.width + x];
                    float a = ((float*)tile.images[channel_a])[y * tile.width + x];
                    image.pixels[(y + tile.offset_y) * image.width + (x + tile.offset_x)] = rgba(r, g, b, a);
                }
            }
        }
    }
    FreeEXRHeader(&exr_header);
    FreeEXRImage(&exr_image);
    return true;

error:
    FreeEXRImage(&exr_image);
    FreeEXRHeader(&exr_header);
    FreeEXRErrorMessage(err);
    return false;
}

bool save_exr(const std::string& exr_file, const Image& image) {
    std::ofstream file(exr_file, std::ofstream::binary);
    if (!file)
        return false;

    std::unique_ptr<float[]> channels[4];
    for (size_t i = 0; i < 4; ++i)
        channels[i].reset(new float[image.width * image.height]);

    for (size_t y = 0; y < image.height; ++y) {
        for (size_t x = 0; x < image.width; ++x) {
            auto& pix = image.pixels[y * image.width + x];
            channels[0][y * image.width + x] = pix.w;
            channels[1][y * image.width + x] = pix.z;
            channels[2][y * image.width + x] = pix.y;
            channels[3][y * image.width + x] = pix.x;
        }
    }

    EXRHeader exr_header;
    EXRImage exr_image;
    InitEXRHeader(&exr_header);
    InitEXRImage(&exr_image);
    float* images[4] = { channels[0].get(), channels[1].get(), channels[2].get(), channels[3].get() };
    exr_image.images = (unsigned char**)images;
    exr_image.width = image.width;
    exr_image.height = image.height;

    std::unique_ptr<EXRChannelInfo[]> channel_infos(new EXRChannelInfo[4]);
    exr_header.num_channels = 4;
    exr_header.channels = channel_infos.get();
    strcpy(exr_header.channels[0].name, "A");
    strcpy(exr_header.channels[1].name, "B");
    strcpy(exr_header.channels[2].name, "G");
    strcpy(exr_header.channels[3].name, "R");

    std::unique_ptr<int[]> pixel_types(new int[4]);
    for (size_t i = 0; i < 4; ++i)
        pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT;
    exr_header.pixel_types = pixel_types.get();
    exr_header.requested_pixel_types = pixel_types.get();

    const char* err = nullptr;
    unsigned char* mem = nullptr;
    size_t size = SaveEXRImageToMemory(&exr_image, &exr_header, &mem, &err);
    if (size == 0) {
        FreeEXRErrorMessage(err);
        return false;
    }
    file.write((const char*)mem, size);
    free(mem);
    return true;
}
