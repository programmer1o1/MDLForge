// Minimal cross-platform shims for studiomdl to avoid hard Windows dependencies.
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>

#pragma push_macro("min")
#pragma push_macro("max")
#undef min
#undef max
#include <thread>
#include <chrono>
#pragma pop_macro("max")
#pragma pop_macro("min")

#ifndef _WIN32

using BYTE = unsigned char;
using WORD = std::uint16_t;
using DWORD = std::uint32_t;
using ULONG = std::uint32_t;

#ifndef MAX_PATH
#define MAX_PATH 260
#endif

// Windows-style debug string helper; on non-Windows just write to stderr.
inline void OutputDebugString( const char *msg )
{
    if ( msg )
    {
        std::fputs( msg, stderr );
        std::fflush( stderr );
    }
}

// Bitmap structures normally provided by windows.h.
#define BI_RGB 0L

#pragma pack(push, 1)
struct BITMAPFILEHEADER
{
    WORD    bfType;
    DWORD   bfSize;
    WORD    bfReserved1;
    WORD    bfReserved2;
    DWORD   bfOffBits;
};

struct BITMAPINFOHEADER
{
    DWORD   biSize;
    std::int32_t biWidth;
    std::int32_t biHeight;
    WORD    biPlanes;
    WORD    biBitCount;
    DWORD   biCompression;
    DWORD   biSizeImage;
    std::int32_t biXPelsPerMeter;
    std::int32_t biYPelsPerMeter;
    DWORD   biClrUsed;
    DWORD   biClrImportant;
};

struct RGBQUAD
{
    BYTE rgbBlue;
    BYTE rgbGreen;
    BYTE rgbRed;
    BYTE rgbReserved;
};
#pragma pack(pop)

#else
#include <windows.h>
#endif
