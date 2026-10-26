/* Host portability only; firmware uses ESP-IDF/newlib's endian.h. */
#pragma once
#ifdef __APPLE__
#include <libkern/OSByteOrder.h>
#define le16toh(x) OSSwapLittleToHostInt16(x)
#define le32toh(x) OSSwapLittleToHostInt32(x)
#define le64toh(x) OSSwapLittleToHostInt64(x)
#else
#include_next <endian.h>
#endif
